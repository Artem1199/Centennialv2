#![deny(unsafe_code)]
#![deny(warnings)]
#![allow(non_snake_case)]
#![no_main]
#![no_std]

use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f3xx_hal as hal;
use hal::gpio::{AF5, Output, Edge, PushPull, OpenDrain, Input, PE1, PE3, PE10, PA5, PA6, PA7,
                PB12, PB13, PB14, PB15,
                PE4, PB6, PB7, AF4};
use hal::spi::{Spi};
use hal::i2c::{I2c};
use hal::pac::{SPI1,I2C1};
use hal::prelude::*;
use hal::pwm::{PwmChannel, WithPins, Tim3Ch1, Tim3Ch4, tim3};
use systick_monotonic::{fugit::Duration, Systick}; //used for timers
use l3gd20::{L3gd20, Odr, Scale, I2Mode,FIFOToggle, FIFOMode};
use lsm303dlhc::{Lsm303dlhc, AccelOdr, Sensitivity, I1ModeA, FIFOToggleA, FIFOModeA};
use movavg::MovAvg;
use l298n::L298N;

// defines the watermark level and buff size when reading from the l3gd20
// reducing the size will help with responsiveness of the gyro
// but will create more interrupts and increase error
// (max value = 31), not sure why 32 does not work; look into in the future
// TODO: setting the gyro to read at 780hz and having a 16+ row size; can result in
// the averaging function to overflow; need to use larger units
const GYRO_WTM: usize = 6;
const ACC_WTM: usize = 6;


#[rtic::app(device = stm32f3xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {
    use super::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led: PE10<Output<PushPull>>,  //Setup PushPull for blinking
        state: u8,
        // gyro setting
        l3gd20_int2: PE1<Input>,
        l3gd20_driver: L3gd20<Spi
                                <SPI1,
                                    (PA5<AF5<PushPull>>,
                                    PA6<AF5<PushPull>>,
                                    PA7<AF5<PushPull>>,),>,
                                        PE3<Output<PushPull>>
                                 >,
        // accel setting
        lsm303dlhc_int1: PE4<Input>,
        lsm303dlhc_driver: Lsm303dlhc<I2c
                                        <I2C1,
                                            (PB6<AF4<OpenDrain>>,
                                             PB7<AF4<OpenDrain>>,),>
                                        >,

        // moto pwm settings
        motor_driver: L298N<PB12<Output<PushPull>>,
                          PB13<Output<PushPull>>,
                          PB14<Output<PushPull>>,
                          PB15<Output<PushPull>>,
                          PwmChannel<Tim3Ch1, WithPins>,
                          PwmChannel<Tim3Ch4, WithPins>
                          >,

    }

    #[monotonic(binds = SysTick, default = true)] //?
    type MonoTimer = Systick<5000>;  //Systick<TIMER_HZ> 1000 hz, 1ms ganularity
    //Actual rate of the timer is combionation of syslck and TIMER_HZ
    //TIMER_HZ is a settable rate
    //monotic has a binds = InterruptName
    //default = true enables a shorthand API when spawning and accessing
    //monotinics::now() vs. monotonics::MyMono::now()
    //default piority is max, prefer to denote this if necessary

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        rtt_init_print!();
        rprintln!("init");

        let mut flash = cx.device.FLASH.constrain();  //?
        let mut rcc = cx.device.RCC.constrain(); //?
        let mut syscfg = cx.device.SYSCFG.constrain(&mut rcc.apb2);  // configuring to enable interrupts later on
        let mut exti = cx.device.EXTI;  // Configuring for interrupt use

        // this initializes the monotonic
        let mono = Systick::new(cx.core.SYST, 36_000_000); //?
        // monotonics are scheduled in #[init]
        // must be returned in the init::Monotics(...) tuple, see later
        // the tulple ativates the monotics


        let clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(36.MHz()) // speed at which SysTick runs
            .pclk1(36.MHz())
            .freeze(&mut flash.acr);


        // splitting out used ports
        // Configuration Ports so far:
        // A, B, E
        let mut gpioa = cx.device.GPIOA.split(&mut rcc.ahb);
        let mut gpiob = cx.device.GPIOB.split(&mut rcc.ahb);
        let mut gpioe = cx.device.GPIOE.split(&mut rcc.ahb);


        let mut led = gpioe
            .pe10
            .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
        led.set_high().unwrap();

        
        // ****** Start of Gyroscope configuration ****** //

        let spi_pa5 = gpioa.pa5.into_af_push_pull::<5>(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);  // Notice ::<5>, in the source code the
             // the value <const A: u8> was expected, as such the ::<5> was supplied
             // Another note, ending was originally &mut gpio.afrh, changed to gpio.afrl, afrl are low registers offset e.g. offset 0x20, and afrh are high registers
             // mostly used for mapping to specific addresses
        let spi_pa6 = gpioa.pa6.into_af_push_pull::<5>(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
        let spi_pa7 = gpioa.pa7.into_af_push_pull::<5>(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
        let spi_cs_pe3 = gpioe.pe3.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);

        let l3d20_spi = Spi::new(cx.device.SPI1, (spi_pa5, spi_pa6, spi_pa7), 1.MHz(), clocks, &mut rcc.apb2);  // using clock from before
        // also noticed that freqquency can be up to 3.Mhz and Clock can also be set faster; look into this for improvements

        let mut l3gd20_int2 = gpioe.pe1.into_pull_down_input(&mut gpioe.moder, &mut gpioe.pupdr);
        syscfg.select_exti_interrupt_source(&l3gd20_int2);

        l3gd20_int2.enable_interrupt(&mut exti);
        l3gd20_int2.trigger_on_edge(&mut exti, Edge::Rising);
        let mut l3gd20_driver = L3gd20::new(l3d20_spi, spi_cs_pe3).unwrap();

        // -- l3GD20 Register Configuration
        l3gd20_driver.set_odr(Odr::Hz380).unwrap();
        l3gd20_driver.set_scale(Scale::Dps2000).unwrap();  //Todo: setup a print to verify these values after they are set
        l3gd20_driver.set_fifo_mode(FIFOMode::Stream).unwrap();
        l3gd20_driver.set_wtm_tr(GYRO_WTM).unwrap();
        l3gd20_driver.set_int2_mode(I2Mode::I2_WTM).unwrap();

        l3gd20_driver.set_fifo_toggle(FIFOToggle::FIFO_EN).unwrap();
        // ****** End of gyroscope configuration ****** //

        // ****** Start of Accelerometer configuration ****** //

        // -- LSM03DHLC INT1 Interrupt Setup & Configuration
        let mut lsm303dlhc_int1 = gpioe.pe4.into_pull_down_input(&mut gpioe.moder, &mut gpioe.pupdr);
        syscfg.select_exti_interrupt_source(&lsm303dlhc_int1);
        lsm303dlhc_int1.enable_interrupt(&mut exti);
        lsm303dlhc_int1.trigger_on_edge(&mut exti, Edge::Rising);

        // -- LSM303DLHC Driver Setup
        let i2c1_pb6 = gpiob.pb6.into_af_open_drain::<4>(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
        let i2c1_pb7 = gpiob.pb7.into_af_open_drain::<4>(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
        let lsm303dlhc_i2c = I2c::new(cx.device.I2C1, (i2c1_pb6, i2c1_pb7), 400000.Hz(), clocks, &mut rcc.apb1);
        let mut lsm303dlhc_driver = Lsm303dlhc::new(lsm303dlhc_i2c).unwrap();

        // -- LSM303DLHC Register Configuration
        lsm303dlhc_driver.accel_odr(AccelOdr::Hz400).unwrap();
        lsm303dlhc_driver.set_accel_sensitivity(Sensitivity::G1).unwrap();

        // ** LSM303DLHC reset sequence, from: https://github.com/SealHAT/LSM303/blob/master/LSM303AGR.c **//
        const WTM_RESET: usize = 0;
        lsm303dlhc_driver.set_accel_fifo_mode(FIFOModeA::Bypass).unwrap();  // Clear FIFO
        lsm303dlhc_driver.set_accel_wtm_tr(WTM_RESET).unwrap();  // Set WTM to 0 to reset any WTM interrupts
        lsm303dlhc_driver.set_accel_fifo_toggle(FIFOToggleA::FIFO_EN).unwrap();
        lsm303dlhc_driver.set_accel_fifo_mode(FIFOModeA::Stream).unwrap();
        lsm303dlhc_driver.set_accel_wtm_tr(ACC_WTM).unwrap();
        lsm303dlhc_driver.set_accel_int1_mode(I1ModeA::I1_WTM).unwrap();
        
        // ****** End of Accelerometer configuration ****** //


        // ----- Start Motor Configurations ----- //
        let motora_pb12 = gpiob.pb12.into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
        let motora_pb13 = gpiob.pb13.into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
        
        let motorb_pb14 = gpiob.pb14.into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
        let motorb_pb15 = gpiob.pb15.into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);

        let motora_pb4  = gpiob.pb4.into_af_push_pull::<2>(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
        let motorb_pb1  = gpiob.pb1.into_af_push_pull::<2>(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);

        // get channels from tim3 (Tim3Chx, x = 1 to 4)
        // , resolution of duty cycle, period, clocks
        // resolution of duty cycle is 0 to 65535
        // Motor starts up much faster with 15hz, but has more vibrations
        // using 15 hz for now
        // TODO: noticed the motor does not always run far enough
        let tim3_channels = tim3(cx.device.TIM3, 1024, 15.Hz(), &clocks);

        let motora_pwm = tim3_channels.0.output_to_pb4(motora_pb4);
        let motorb_pwm = tim3_channels.3.output_to_pb1(motorb_pb1);

        let motor_driver = L298N::new(motora_pb12, motora_pb13, motora_pwm, motorb_pb14, motorb_pb15, motorb_pwm);

        rprintln!("Max duty cycle: {}", motor_driver.a.get_max_duty());

        // ****** End of Motor configuration ****** //


        if l3gd20_int2.is_interrupt_pending()  {
            rprintln!("Interrupt pending on L3GD20 int2 pin!");
        } else {
            rprintln!("Interrupt not pending on L3GD20 int2");
        }

        if lsm303dlhc_int1.is_interrupt_pending()  {
            rprintln!("Interrupt pending on LSM303DLHC int1 pin!");
        } else {
            rprintln!("Interrupt not pending on LSM303DLHC int1");
        }

        // schedule blink to run after:
        // <T, const NOM: u32, const DENUM: u32> duration in seconds:
        // NOM / DENOM * ticks
        // 
        blink::spawn_after(Duration::<u64, 1, 5000>::from_ticks(1000)).unwrap();
        //spawn_after must use rtic_monotonic::Monotonic trait
        //Some timer that handls timing of the system
        
        rprintln!("Leaving init");

        
        let accel = lsm303dlhc_driver.accel_fifo::<{ACC_WTM*6+1},ACC_WTM>().unwrap();

        for i in 0..ACC_WTM {

            rprintln!("accel val    x: {}", accel.i16x3buf[i].x);
        };

        (Shared {},
         Local {led, l3gd20_driver, motor_driver, l3gd20_int2, lsm303dlhc_driver, lsm303dlhc_int1, state: 0},
         init::Monotonics(mono),
        )




    }

    // tasks uses the locals led & state
    #[task(local = [led, motor_driver, state])]
    fn blink(cx: blink::Context) {

        cx.local.motor_driver.a.set_duty(300);
        cx.local.motor_driver.b.set_duty(305);
        

        if *cx.local.state == 0 {  //if local state (note deference)
            
            cx.local.led.set_high().unwrap();  // the the LED HIGH
            cx.local.motor_driver.a.forward();
            cx.local.motor_driver.b.forward();

            *cx.local.state = 1;

            rprintln!("Running forward");

            blink::spawn_after(Duration::<u64, 1, 5000>::from_ticks(950)).unwrap();
            
        } else if *cx.local.state == 1 {
            cx.local.motor_driver.a.set_duty(1024);
            cx.local.motor_driver.b.set_duty(1024);

            cx.local.led.set_low().unwrap(); //set low
            cx.local.motor_driver.a.stop();
            cx.local.motor_driver.b.stop();

            *cx.local.state = 2;

            rprintln!("Braking");
            blink::spawn_after(Duration::<u64, 1, 5000>::from_ticks(7000)).unwrap();
        }
        else if *cx.local.state == 2{

            cx.local.led.set_low().unwrap(); //set low
            cx.local.motor_driver.a.forward();
            cx.local.motor_driver.b.forward();

            *cx.local.state = 3;
            rprintln!("Running foward");
            blink::spawn_after(Duration::<u64, 1, 5000>::from_ticks(950)).unwrap();

        } else if *cx.local.state == 3{
            cx.local.motor_driver.a.set_duty(1024);
            cx.local.motor_driver.b.set_duty(1024);

            cx.local.led.set_low().unwrap(); //set low
            cx.local.motor_driver.a.stop();
            cx.local.motor_driver.b.stop();

            *cx.local.state = 0;
            rprintln!("Braking");

            blink::spawn_after(Duration::<u64, 1, 5000>::from_ticks(7000)).unwrap();
        }

    }



    #[task(binds = EXTI4, local = [lsm303dlhc_driver, lsm303dlhc_int1])]
    fn on_lsm303dlhc (cx: on_lsm303dlhc::Context){

        let accel_fifo = cx.local.lsm303dlhc_driver.accel_fifo::<{ACC_WTM*6+1},ACC_WTM>().unwrap();

        let mut avg_x: MovAvg<i32, i32, ACC_WTM> = MovAvg::new();
        let mut avg_y: MovAvg<i32, i32, ACC_WTM> = MovAvg::new();
        let mut avg_z: MovAvg<i32, i32, ACC_WTM> = MovAvg::new();

        for i in 0..GYRO_WTM {
            avg_x.feed(accel_fifo.i16x3buf[i].x as i32);
            avg_y.feed(accel_fifo.i16x3buf[i].y as i32);
            avg_z.feed(accel_fifo.i16x3buf[i].z as i32);
        }

        rprintln!("gyro val     x: {},   y: {},   z: {}", 
            avg_x.get(),
            avg_y.get(),
            avg_z.get(),);


        cx.local.lsm303dlhc_int1.clear_interrupt();
    }

     // Gyroscope L3GD20 Interrupt Handler
     #[task(binds = EXTI1, local = [l3gd20_driver, l3gd20_int2])]
     fn on_l3gd20(cx: on_l3gd20::Context) {

        // ROWS sets up the watermark level for the buffer
        // grabs 16 values from the buffer and averages them out
        //let ROWS = GYRO_WTM;
        let gyro_fifo = cx.local.l3gd20_driver.gyro_fifo::<{GYRO_WTM*6+1}, GYRO_WTM>().unwrap();

        // TODO: moving average may-not be the most efficient
        // consider look into a faster avg. method
        // - Another toml option is available for faster averaging in this library
        // look into if necessary
        let mut avg_x: MovAvg<i16, i16, GYRO_WTM> = MovAvg::new();
        let mut avg_y: MovAvg<i16, i16, GYRO_WTM> = MovAvg::new();
        let mut avg_z: MovAvg<i16, i16, GYRO_WTM> = MovAvg::new();

        for i in 0..GYRO_WTM {
            avg_x.feed(gyro_fifo.i16x3buf[i].x);
            avg_y.feed(gyro_fifo.i16x3buf[i].y);
            avg_z.feed(gyro_fifo.i16x3buf[i].z);
        }
        rprintln!("gyro val     x: {},   y: {},   z: {}", 
            avg_x.get(),
            avg_y.get(),
            avg_z.get(),);

        cx.local.l3gd20_int2.clear_interrupt();
     }


}