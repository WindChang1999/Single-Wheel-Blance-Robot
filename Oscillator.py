import serial
import serial.tools.list_ports
import numpy as np
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
import matplotlib.animation as animation


class Scope(object):
    def __init__(self, axs, maxpoint=200):
        self.axs = axs
        self.pointCNT = 0
        self.maxpoint = maxpoint
        self.time = []
        self.Encoder_BottomWheel = []
        self.Encoder_InertiaWheel = []
        self.AngleX = []
        self.AngleY = []
        self.line0 = Line2D(self.time, self.Encoder_BottomWheel, color='g', linewidth=1.0, label='ENC_BW')
        self.axs[0].add_line(self.line0)
        self.line2 = Line2D(self.time, self.AngleX, color='r', linewidth=1.0, label='AngleX')
        self.axs[1].add_line(self.line2)
        self.line1 = Line2D(self.time, self.Encoder_InertiaWheel, color='b', linewidth=1.0, label='ENC_IW')
        self.axs[0].add_line(self.line1)
        self.line3 = Line2D(self.time, self.AngleY, color='y', linewidth=1.0, label='AngleY')
        self.axs[1].add_line(self.line3)
        #
        self.axs[0].set_ylim(-3000, 3000)
        self.axs[1].set_ylim(-50, 50)
        self.axs[0].set_xlim(0, self.maxpoint)
        self.axs[1].set_xlim(0, self.maxpoint)
        self.axs[0].legend(loc=1)
        self.axs[1].legend(loc=1)
        # self.axs[1].set_xlim(0, self.maxpoint)

    def update(self, ReceiveData):
        if len(ReceiveData) == 0:
            return self.line0, self.line1, self.line2, self.line3,
        self.pointCNT += 1
        if self.pointCNT >= self.maxpoint:  # 到最大点后数组左移
            # self.axs[1].set_xlim(firstt, self.maxpoint + firstt)
            firstt = self.time.pop(0)
            self.Encoder_BottomWheel.pop(0)
            self.Encoder_InertiaWheel.pop(0)
            self.AngleX.pop(0)
            self.AngleY.pop(0)
            self.axs[0].set_xlim(firstt, self.maxpoint + firstt)
            self.axs[1].set_xlim(firstt, self.maxpoint + firstt)
            self.axs[0].figure.canvas.draw()
            self.axs[1].figure.canvas.draw()

        self.time.append(self.pointCNT)
        self.Encoder_BottomWheel.append(ReceiveData[1])
        self.Encoder_InertiaWheel.append(ReceiveData[2])
        self.AngleX.append(ReceiveData[3])
        self.AngleY.append(ReceiveData[4])
        self.line0.set_data(self.time, self.Encoder_BottomWheel)
        self.line1.set_data(self.time, self.Encoder_InertiaWheel)
        self.line2.set_data(self.time, self.AngleX)
        self.line3.set_data(self.time, self.AngleY)
        return self.line0, self.line1, self.line2, self.line3,
        # return self.line0,


def readSerialPort():
    while True:
        if ser.in_waiting != 0:
            RecieveData = ser.readline().decode('gbk')
            if len(RecieveData) > 10:
                RecieveData = RecieveData.split(' ')
                print(RecieveData)
                RecieveData[-1] = RecieveData[-1][:-1]
                RecieveData = np.array([float(data) for data in RecieveData])
                yield RecieveData
            else:
                yield []


portlist = list(serial.tools.list_ports.comports())
if len(portlist) == 0:
    print("NONE")
else:
    for port in portlist:
        print(port)


ser = serial.Serial()
ser.baudrate = 38400
ser.port = "/dev/cu.usbmodem144403"
ser.timeout = 10
ser.open()

if  ser.is_open == False:
    print("Open Serial failed")
else:
    fig = plt.figure()
    ax0 = plt.subplot(211)
    ax1 = plt.subplot(212)
    axs = [ax0, ax1]
    scope = Scope(axs)
    ani = animation.FuncAnimation(fig, scope.update, readSerialPort, interval=50, blit=True)
    plt.show()
print("Stop Recieve Data!")