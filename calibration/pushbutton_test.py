# by default the pin reads 1, changes to 0 when pressed.
import os
import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
button_pin = 18
GPIO.setup(button_pin, GPIO.IN)
cur_time = time.time()
while(1):
    if cur_time + 0.5 < time.time():
        cur_time = time.time()
        print(GPIO.input(button_pin))

