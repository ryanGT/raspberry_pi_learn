import wiringpi2 as wiringpi
from time import sleep       # allows us a time delay
wiringpi.wiringPiSetupGpio()
wiringpi.pinMode(24, 0)      # sets GPIO 24 to input
wiringpi.pullUpDnControl(24, 1)#pull down (2 is up, 0 is float)
rising_edge = wiringpi.GPIO.INT_EDGE_RISING



state = 0
try:
    while True:
        if state == 0:
            wiringpi.digitalWrite(24, 1) # switch on LED. Sets port 24 to 1 (3V3, on)
            state = 1
        else:
            wiringpi.digitalWrite(24, 0) # switch off LED. Sets port 24 to 0 (0V, off)
            state = 0
        sleep(0.5)                      # delay 0.05s

finally:  # when you CTRL+C exit, we clean up
    wiringpi.digitalWrite(24, 0) # sets port 24 to 0 (0V, off)
    wiringpi.pinMode(24, 0)      # sets GPIO 24 back to input Mode
    # GPIO 25 is already an input, so no need to change anything
