import RPi.GPIO as GPIO
import time

pin = 7
GPIO.setmode(GPIO.BOARD)
GPIO.setup(pin, GPIO.IN)

while(True):
    if(GPIO.input(pin)):
        print("HIGH")
    else:
        print("LOW")