import RPi.GPIO as GPIO
import time

buttonpin = 11 #7
outputpin = 8
GPIO.setmode(GPIO.BOARD)
GPIO.setup(buttonpin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(outputpin, GPIO.OUT)

while(True):
    if(GPIO.input(buttonpin)):
        print("HIGH")
        GPIO.ouput(outputpin, 1)
    else:
        print("LOW")
        GPIO.ouput(outputpin, 0)
    time.sleep(0.1)