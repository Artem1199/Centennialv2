import serial
from cobs import cobs
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from collections import deque
from threading import Thread
import copy
import collections
import csv

# Values shall match Rust Code
accel_gravity = 0.00059848449
gyro_degrees = 0.0175
f = open("C:\\Users\\Artem\\Desktop\\results.csv", 'w')
writer = csv.writer(f)
header = ['n_samples', 'gyr_x', 'gyr_y', 'gyr_z', 'acc_x', 'acc_y', 'acc_z']
writer.writerow(header)


class SensorData:
    def __init__(self, name, value):
        self.name = name
        self.value = value
    def __str__(self):
        return f"{self.name}: {self.value}"
    

# takes in decoded data string, output sensor data list (T -> AZ)
def data_split(decoded_data):
    #print(decoded_data)
    try: 
        buf_len = len(decoded_data)
        #print("buffer length: ", buf_len)
        if buf_len < 28: # should make this part of data sent by robo
            # return 0 # try to return exception instead
            raise ValueError("Data provided length is less than", 28) # todo 28 should, be variable
    except:
        # return 0 # same here excption instead
        raise ValueError("Data provided is less than can be decoded")
    # length of decoded data
    set_cnt = int(len(decoded_data)/4)
    sensordata_list = []
    datacsv =[]
    
    for i in range(0, set_cnt):
        #print(i)
        k = i*4
        #decode bits to string
        name = decoded_data[k:k+2].decode("utf-8")
        value = int.from_bytes(decoded_data[k+2:k+4], "big", signed = True)
        #data.append(str(value))
        if 4 > i > 0:
            # transform gyro values
            value = value * gyro_degrees
            sensordata_list.append(SensorData(name,value))
            datacsv.append(str(value))
        elif i > 3:
            value = value * accel_gravity
            sensordata_list.append(SensorData(name,value))
            datacsv.append(str(value))
        else:# timer data
            sensordata_list.append(SensorData(name,value))
            global n_samples
            datacsv.append(str(n_samples))
            n_samples = n_samples + 1 # increment number of samples
        # write the data to an array to get saved
    # write data to log file
    writer.writerow(datacsv)
    #return array of data with naming
    return sensordata_list

        
sensor_data = 0
previousTimer = 0
n_samples = 1

running = True
ser = serial.Serial()

maxPlotLength = 300 # number of points in x-axis of rtp
numPlots = 3 # number of plots in 1 graph
numGraphs = 2 # number of graphs

datag = []
for i in range(numPlots):
    datag.append(collections.deque([0] * maxPlotLength, maxlen=maxPlotLength))

dataa = []
for i in range(numPlots):
    dataa.append(collections.deque([0] * maxPlotLength, maxlen=maxPlotLength))

def data_collect():
    while running:
        # Read the raw data until 0x00 deliminter shows up
        data_raw = ser.read_until(b'\x00')
        # holder for cleaned up data
        data = 0
        # remove deliminter from data
        try:
            data = cobs.decode(data_raw[:-1])
        except Exception as e:
            print("Data_collect error when performing decode", e)
        # turn sensor data bytes into structure
        try: 
            sensor_data_temp = data_split(data)
        except Exception as e:
        # if error, skip the data, print error
            print("Data_collect error when performing split", e)
        else:
            global sensor_data
            #print("Printing sensor_data_tempt", sensor_data_temp)
            sensor_data = sensor_data_temp
            #print(sensor_data[1].name, sensor_data[1].value)

def get_data(frame, linesg, linesa, lineValueTextg, lineValueTexta, timeText):
    currentTimer = time.perf_counter()
    global previousTimer
    # first values will wrong
    plotTimer = int((currentTimer - previousTimer) * 1000)
    previousTimer = currentTimer
    timeText.set_text('Plot Interval = ' + str(plotTimer) + 'ms')
    global sensor_data
    privateData = copy.deepcopy(sensor_data)
    #Puts cleaned up data into each line
    for i in range(numPlots):
        value = privateData[i+1].value
        datag[i].append(value) # append latest data
        linesg[i].set_data(range(maxPlotLength), datag[i])
        lineValueTextg[i].set_text('[' + privateData[i+1].name +'] = ' + str(round(value, 2)))
    for i in range(numPlots):
        value = privateData[i+1+3].value
        dataa[i].append(value) # append latest data
        linesa[i].set_data(range(maxPlotLength), dataa[i])
        lineValueTexta[i].set_text('[' + privateData[i+1+3].name +'] = ' + str(round(value, 2)))
    return linesg[0], linesg[1], linesg[2], linesa[0], linesa[1], linesa[2], lineValueTextg[0], lineValueTextg[1], lineValueTextg[2],lineValueTexta[0], lineValueTexta[1], lineValueTexta[2], timeText,
    
def makefigure(xLimit, yLimit):
    print("Making figure")
    xming, xmaxg = xLimit[0]
    yming, ymaxg = yLimit[0]
    xmina, xmaxa = xLimit[1]
    ymina, ymaxa = yLimit[1]
    
    fig, axs = plt.subplots(1, numGraphs, sharex=True)
    #fig = plt.figure()
    fig.suptitle("Gyroscope and Accelerometer Live Data")
    fig.supxlabel("Time")
    axs[0].set_ylabel("Gyroscope Output")
    axs[1].set_ylabel("Accelerometer Output")
    axs[0].set_xlim(xming, xmaxg)
    axs[0].set_ylim(int(yming - (ymaxg - yming) / 10), int(ymaxg + (ymaxg - yming) / 10))
    axs[1].set_xlim(xmina, xmaxa)
    axs[1].set_ylim(int(ymina - (ymaxa - ymina) / 10), int(ymaxa + (ymaxa - ymina) / 10))
    
    #axs[0] = plt.axes(xlim=(xming, xmaxg), ylim=(int(yming - (ymaxg - yming) / 10), int(ymaxg + (ymaxg - yming) / 10)))
    #axs[1] = plt.axes(xlim=(xmina, xmaxa), ylim=(int(ymina - (ymaxa - ymina) / 10), int(ymaxa + (ymaxa - ymina) / 10)))
    
    return fig, axs


def main():
    ser.baudrate = 230400
    ser.port = 'COM8'
    ser.timeout = 10 #specify timoue when reading readline()
    
    # Bluetooth connection trys an retries
    try:
        time.sleep(.3)
        ser.open()
        time.sleep(.3)
    except Exception as e:
        print("Received error:", e)
        ser.close()
        time.sleep(.3)
        # longer retry if connection was not made the first time
        for x in range(0, 100):
            print("Serial port connection retry attempting...")
            try:
                ser.open()
                error = None
            except Exception as e:
                error = str(e)
                print(error)
                pass
            if error:
                wait = 5
                print("Waiting", wait,"seconds to try again")
                ser.close()
                time.sleep(10)
            else:
                break
            
    # alot of this borrowed from: https://thepoorengineer.com/en/arduino-python-plot/
    # start bluetooth reading thread
    thread = Thread(target = data_collect)
    thread.start()

    # plotting starts here

    pltInterval = 50 # Period at which the plot animates
    #fig = plt.figure(figsize=(10,8))
    # finds limints for the axis note, "float" prob not critical
    # ax = plt.axes(xlim=(xmin, xmax), ylim=(float(yming - (ymaxg - yming) / 10), float(ymaxg + (ymaxg - yming) / 10)))
    # ax.set_title('Rusty Gyro')
    
    # initializing elements
    lineLabelg = ['GX', 'GY', 'GZ']
    lineLabela = ['AX', 'AY', 'AZ']
    style = ['r-', 'c-', 'b-'] # linestyles for the different plots
    xLimits = [(0, maxPlotLength), (0, maxPlotLength)]  # time limit
    yLimits = [(-300,300), (-20, 20)] #  gyr and accel limits
    linesg = []
    lineValueTextg = []
    linesa = []
    lineValueTexta = []
    
    fig, axs = makefigure(xLimits, yLimits)
    timeText = axs[0].text(0.70, 0.95, "", transform=axs[0].transAxes)
    timeText = axs[1].text(0.70, 0.95, "", transform=axs[1].transAxes)
    
    for i in range (numPlots):
        print("init Plots")
        linesg.append(axs[0].plot([], [], style[i], label=lineLabelg[i])[0])
        linesa.append(axs[1].plot([], [], style[i], label=lineLabela[i])[0])
        lineValueTextg.append(axs[0].text(0.70, 0.90-i*0.05, "", transform=axs[0].transAxes))
        lineValueTexta.append(axs[1].text(0.70, 0.90-i*0.05, "", transform=axs[1].transAxes))
        
    anim = animation.FuncAnimation(fig, get_data, fargs = (linesg, linesa, lineValueTextg, lineValueTexta, timeText), interval = pltInterval, blit=True)
    axs[0].legend(loc = "upper left")
    axs[1].legend(loc = "upper left")
    
    plt.show()
    
    #add a close function
    
if __name__ == '__main__':
    main()


