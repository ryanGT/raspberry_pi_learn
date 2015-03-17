#!/user/bin/env python
"""
gpiooutput.py
Modified from pg 195 of "Raspberry Pi Users Guide"

Make a LED flash on\off.
Hardware Setup:
* LED+ lead hooked to GPIO 17 (board pin 11)
* LED- lead connected to resistor.
* Resistor connected to ground (board pin 6)
"""
import time
import RPi.GPIO as GPIO

# This is the GPIO number.  The actual board pin is number 11.
#PINOUT = 17
PINOUT = 24

GPIO.setmode(GPIO.BCM) # Set to use GPIO nums, rather than physical board nums
GPIO.setup(PINOUT, GPIO.OUT)
print "Blink begins!  Press ctrl+c to exit"
try:
    i = 1
    while True:
        GPIO.output(PINOUT, True)
        print "Blink %s ON!!"%i
        time.sleep(2)
        GPIO.output(PINOUT, False)
        print "Blink %s OFF!!"%i
        time.sleep(2)
        i += 1
except KeyboardInterrupt:
    GPIO.output(PINOUT, False)
    GPIO.cleanup() 
    print "\nBlink DONE!"

