#![deny(unsafe_code)]
#![deny(warnings)]
#![allow(non_snake_case)]
#![no_main]
#![no_std]

use panic_rtt_target as _;
// use rtic::app;
use rtt_target::{rprintln, rtt_init_print};
use stm32f3xx_hal as hal;
use hal::gpio::{AF5, Output, PushPull, PE3, PE10, PA5, PA6, PA7};
use hal::spi::{Spi};
use hal::prelude::*;
use hal::pac::{SPI1,};
use systick_monotonic::{fugit::Duration, Systick}; //used for timers
use l3gd20::{L3gd20, Odr, Scale};

#[rtic::app(device = stm32f3xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {
    use super::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
      led: PE10<Output<PushPull>>,  //Setup PushPull for blinking
      state: bool,
      // l3d20_spi_pins: (PA5<AF5<PushPull>>,
       //                  PA6<AF5<PushPull>>,
       //                 PA7<AF5<PushPull>>,),  //maybe redundant to have this pins section, since we will never call to it again after SPI is setup
       //  l3d20_spi: Spi<SPI1,(PA5<AF5<PushPull>>,PA6<AF5<PushPull>>,PA7<AF5<PushPull>>,),>  // setup a type(?) for l3d20_spi
        // note, had to use pac::SPI1 for this to work, pac::SPI1 is a public struct.  Suspect this is implying that we will
        // the SPI1 peripheral from pac, in essence preventing others from using it anywhwere else
        // pins are the same pins as the pins type, 
        l3d20_driver: L3gd20<Spi
                                <
                                    SPI1,
                                    (PA5<AF5<PushPull>>,PA6<AF5<PushPull>>,PA7<AF5<PushPull>>,),>,
                                    PE3<Output<PushPull>>
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

        // this initializes the monotonic
        let mono = Systick::new(cx.core.SYST, 36_000_000); //?
        // monotonics are scheduled in #[init]
        // must be returned in the init::Monotics(...) tuple, see later
        // the tulple ativates the monotics


        let _clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(36.MHz()) // speed at which SysTick runs
            .pclk1(36.MHz())
            .freeze(&mut flash.acr);

        let mut gpioe = cx.device.GPIOE.split(&mut rcc.ahb);
        let mut led = gpioe
            .pe10
            .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
        led.set_high().unwrap();

        let mut gpioa = cx.device.GPIOA.split(&mut rcc.ahb);
        let spi_pa5 = gpioa.pa5.into_af_push_pull::<5>(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);  // Notice ::<5>, in the source code the
             // the value <const A: u8> was expected, as such the ::<5> was supplied
             // Another note, ending was originally &mut gpio.afrh, changed to gpio.afrl, afrl are low registers offset e.g. offset 0x20, and afrh are high registers
             // mostly used for mapping to specific addresses
        let spi_pa6 = gpioa.pa6.into_af_push_pull::<5>(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
        let spi_pa7 = gpioa.pa7.into_af_push_pull::<5>(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
        let spi_cs_pe3 = gpioe.pe3.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);

        let l3d20_spi = Spi::new(cx.device.SPI1, (spi_pa5, spi_pa6, spi_pa7), 1.MHz(), _clocks, &mut rcc.apb2);  // using clock from before
        // also noticed that freqquency can be up to 3.Mhz and _Clock can also be set faster; look into this for improvements

        let mut l3d20_driver = L3gd20::new(l3d20_spi, spi_cs_pe3).unwrap();
        l3d20_driver.set_odr(Odr::Hz380).unwrap();
        l3d20_driver.set_scale(Scale::Dps2000).unwrap();  //Todo: setup a print to verify these values after they are set


        // schedule blink to run after:
        // <T, const NOM: u32, const DENUM: u32> duration in seconds:
        // NOM / DENOM * ticks
        // 
        blink::spawn_after(Duration::<u64, 1, 5000>::from_ticks(1000)).unwrap();
        //spawn_after must use rtic_monotonic::Monotonic trait
        //Some timer that handls timing of the system
        
        rprintln!("Leaving init");

        (Shared {},
         Local {led, l3d20_driver, state: false},
         init::Monotonics(mono),
        )

    }


    // tasks uses the locals led & state
    #[task(local = [led, state, l3d20_driver])]
    fn blink(cx: blink::Context) {
        if *cx.local.state {  //if local state (note deference)
            cx.local.led.set_high().unwrap();  // the the LED HIGH
            *cx.local.state = false; // Change state
        } else {
            cx.local.led.set_low().unwrap(); //set low
            *cx.local.state = true; //change state
        }
        // NOM and DENOM can't be changed unless TIMER_HZ is changed
        // from_ticks can be changed

        let gyro_all = cx.local.l3d20_driver.all().unwrap().gyro;

        rprintln!("X values: {}", gyro_all.x);
        blink::spawn_after(Duration::<u64, 1, 5000>::from_ticks(1000)).unwrap();
    }
}