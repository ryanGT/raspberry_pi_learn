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

port_case = 2

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
    
GPIO.setup(23, GPIO.IN, pull_up_down=up_down)


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
try:
    GPIO.wait_for_edge(23, my_edge)
    #ser.write("test")
    send_list([1,2,3,4])
    print msg1
    print "whatever was waiting for a button press."
except KeyboardInterrupt:
    GPIO.cleanup()       # clean up GPIO on CTRL+C exit
GPIO.cleanup()           # clean up GPIO on normal exit

ser.close()
