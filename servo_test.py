import time
import board
import busio
import math
import numpy as np
import csv

import RPi.GPIO as gpio

servo_pin = 17  #pin 11, GPIO 17
gpio.setmode(gpio.BCM)
gpio.setup(servo_pin, gpio.OUT)

p = gpio.PWM(servo_pin, 50) # GPIO 17 for PWM with 50Hz

dur = 2.5

mini = 7.5
maxi = 8.8

p.start(2.5) # Initialization
try:
    while True:
        print("min")
        p.ChangeDutyCycle(mini)
        time.sleep(0.2)
        p.ChangeDutyCycle(0)
        time.sleep(dur)
        
        p.ChangeDutyCycle(0.5*(mini+maxi))
        time.sleep(0.2)
        p.ChangeDutyCycle(0)
        time.sleep(dur)
        
        print("max")
        p.ChangeDutyCycle(maxi)
        time.sleep(0.2)
        p.ChangeDutyCycle(0)
        time.sleep(dur)
        
        p.ChangeDutyCycle(0.5*(mini+maxi))
        time.sleep(0.2)
        p.ChangeDutyCycle(0)
        time.sleep(dur)

#     print("5")
#     p.ChangeDutyCycle(5)
#     time.sleep(dur)
#     
#     print("7.5")
#     p.ChangeDutyCycle(7.5)
#     time.sleep(dur)
#     
#     print("10")
#     p.ChangeDutyCycle(10)
#     time.sleep(dur)
#     
#     print("12.5")
#     p.ChangeDutyCycle(12.5)
#     time.sleep(dur)
#     
#     print("10")
#     p.ChangeDutyCycle(10)
#     time.sleep(dur)
#     
#     print("7.5")
#     p.ChangeDutyCycle(7.5)
#     time.sleep(dur)
#     
#     print("5")
#     p.ChangeDutyCycle(5)
#     time.sleep(dur)
#     
#     print("2.5")
#     p.ChangeDutyCycle(2.5)
#     time.sleep(dur)
except KeyboardInterrupt:
  p.stop()
  gpio.cleanup()