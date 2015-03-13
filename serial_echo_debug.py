import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
portname = '/dev/ttyAMA0'
import serial
ser = serial.Serial(portname, 115200, timeout=1)
ser.open()
