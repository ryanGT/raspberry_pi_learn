#!/usr/bin/env python2.7
# script by Alex Eames http://RasPi.tv/
# http://raspi.tv/2013/how-to-use-interrupts-with-python-on-the-raspberry-pi-and-rpi-gpio
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)

import serial
ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
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
    ser.write("test")
    print msg1
    print "whatever was waiting for a button press."
except KeyboardInterrupt:
    GPIO.cleanup()       # clean up GPIO on CTRL+C exit
GPIO.cleanup()           # clean up GPIO on normal exit

ser.close()
