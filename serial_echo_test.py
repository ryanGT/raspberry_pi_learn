from matplotlib.pyplot import *
from numpy import *
import numpy, time

import time, copy, os

import serial_utils

#righthand side for me
#portname = '/dev/tty.usbmodem1411'
#lefthand side for me
#portname = '/dev/tty.usbmodem1421'

portname = '/dev/ttyACM0'

ser = serial_utils.Open_Serial(portname)
#from myserial import ser
ser.flushInput()
ser.flushOutput()

debug_line = serial_utils.Read_Line(ser)
line_str = ''.join(debug_line)

#time.sleep(0.1)

dt = 1.0/150#<---------- is this true for your choice of OCR1A?

N = 200

nvect = zeros(N,dtype=int)
v1 = arange(0,N)
v_echo = zeros_like(nvect)

serial_utils.WriteByte(ser, 2)#start new test
              
for i in range(N):
    serial_utils.WriteByte(ser, 1)#new n and voltage are coming
    serial_utils.WriteInt(ser, i)
    serial_utils.WriteInt(ser, v1[i])

    nvect[i] = serial_utils.Read_Two_Bytes(ser)
    v_echo[i] = serial_utils.Read_Two_Bytes_Twos_Comp(ser)
    nl_check = serial_utils.Read_Byte(ser)
    assert nl_check == 10, "newline problem"


time.sleep(0.1)
serial_utils.WriteByte(ser, 3)#stop test
time.sleep(0.1)
serial_utils.WriteByte(ser, 3)#stop test

serial_utils.Close_Serial(ser)

print(line_str)

figure(1)
clf()
plot(nvect, v1**2, nvect, v_echo, 'ro')


t = dt*nvect

data = array([t, v1, v_echo]).T


def save_data(filename, datain):
    #provide filename extension if there isn't one
    fno, ext = os.path.splitext(filename)
    if not ext:
        ext = '.csv'
    filename = fno + ext

    labels = ['#t','v','theta']

    data_str = datain.astype(str)
    data_out = numpy.append([labels],data_str, axis=0)

    savetxt(filename, data_out, delimiter=',')
    

show()
