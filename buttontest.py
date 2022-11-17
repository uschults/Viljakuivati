import RPi.GPIO as GPIO
import time

pin = 7
GPIO.setmode(GPIO.BOARD)
GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

while(True):
    if(GPIO.input(pin)):
        print("HIGH")
    else:
        print("LOW")
    time.sleep(0.1)