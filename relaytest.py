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
        GPIO.output(outputpin, 0)
    else:
        print("LOW")
        GPIO.output(outputpin, 1)
    time.sleep(0.1)