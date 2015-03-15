import wiringpi2 as wiringpi
from time import sleep       # allows us a time delay
wiringpi.wiringPiSetupGpio()
wiringpi.pinMode(24, 0)      # sets GPIO 24 to input
wiringpi.pullUpDnControl(24, 1)#pull down (2 is up, 0 is float)
rising_edge = wiringpi.GPIO.INT_EDGE_RISING

def myfunc():
    print('sup?')


wiringpi.wiringPiISR(24, rising_edge, myfunc)


wiringpi.waitForInterrupt(24, -1)

print('after interrupt')
    
