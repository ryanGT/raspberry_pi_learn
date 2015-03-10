import smbus
import time
# for RPI version 1, use "bus = smbus.SMBus(0)"
bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
address = 0x04

def writeNumber(value):
    bus.write_byte(address, value)
    # bus.write_byte_data(address, 0, value)
    return -1

def readNumber():
    number = bus.read_byte(address)
    # number = bus.read_byte_data(address, 1)
    return number


def read_N_bytes(N):
    list_out = []
    for i in range(N):
        number = bus.read_byte(address)
        list_out.append(number)

    return list_out


def send_list(listin):
    for item in listin:
        writeNumber(item)

## while True:
##     var = input("Enter 1 - 9: ")
##     if not var:
##         continue

##     writeNumber(var)
##     print "RPI: Hi Arduino, I sent you ", var
##     # sleep one second
##     time.sleep(1)

##     number = readNumber()
##     print "Arduino: Hey RPI, I received a digit ", number
##     print
