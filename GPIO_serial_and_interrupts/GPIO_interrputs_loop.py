#!/usr/bin/env python2.7
# script by Alex Eames http://RasPi.tv/
# http://raspi.tv/2013/how-to-use-interrupts-with-python-on-the-raspberry-pi-and-rpi-gpio
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)


import smbus
import time
import subprocess
# for RPI version 1, use "bus = smbus.SMBus(0)"
bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
address = 0x04

def send_list(listin):
    bus.write_i2c_block_data(address, listin[0], listin[1:])

import serial

port_case = 1

# Question: is the latency different between these two?
#
# - if so, this seems like a clear indicator that GPIO serial should be better
# - if not, run the echo testing using GPIO interrupts and see what happens
if port_case == 1:
    portname = '/dev/ttyAMA0'#GPIO
elif port_case == 2:
    portname = '/dev/ttyACM0'#USB

ser = serial.Serial(portname, 115200, timeout=1)
ser.open()

case = 2
# GPIO 23 set up as input. It is pulled up to stop false signals
pat1 = "\n %s edge detected. Now your program can continue with"
if case == 1:
    up_down = GPIO.PUD_UP
    my_edge = GPIO.FALLING
    msg1 = pat1 % 'Falling'
elif case == 2:
    up_down = GPIO.PUD_DOWN
    my_edge = GPIO.RISING
    msg1 = pat1 % 'Rising'

my_test_pin = 24
GPIO.setup(23, GPIO.IN, pull_up_down=up_down)
GPIO.setup(my_test_pin, GPIO.OUT)
GPIO.output(my_test_pin, 0)

print "Make sure you have a button connected so that when pressed"
print "it will connect GPIO port 23 (pin 16) to GND (pin 6)\n"
raw_input("Press Enter when ready\n>")

print "Waiting for falling edge on port 23"
# now the program will do nothing until the signal on port 23 
# starts to fall towards zero. This is why we used the pullup
# to keep the signal high and prevent a false interrupt

print "During this waiting time, your computer is not" 
print "wasting resources by polling for a button press.\n"
print "Press your button when ready to initiate a falling edge interrupt."

N = 200

nvect = zeros(N,dtype=int)
v1 = arange(0,N)
v_echo = zeros_like(nvect)

serial_utils.WriteByte(ser, 2)#start new test


for i in range(N):
    try:
        GPIO.wait_for_edge(23, my_edge)
        GPIO.output(my_test_pin, 1)
        serial_utils.WriteByte(ser, 1)#new n and voltage are coming
        serial_utils.WriteInt(ser, i)
        serial_utils.WriteInt(ser, v1[i])

        nvect[i] = serial_utils.Read_Two_Bytes(ser)
        v_echo[i] = serial_utils.Read_Two_Bytes_Twos_Comp(ser)
        nl_check = serial_utils.Read_Byte(ser)
        assert nl_check == 10, "newline problem"

        time.sleep(0.0001)
        GPIO.output(my_test_pin, 0)
        #ser.write(i)
        #send_list([1,2,3,4])
    except KeyboardInterrupt:
        GPIO.cleanup()       # clean up GPIO on CTRL+C exit


time.sleep(0.1)
serial_utils.WriteByte(ser, 3)#stop test
time.sleep(0.1)
serial_utils.WriteByte(ser, 3)#stop test


GPIO.cleanup()           # clean up GPIO on normal exit

ser.close()
