#!/usr/bin/env python
# coding=utf-8
'''
Author: Yinfeng
'''

import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD) #Set Pi to use board numbering when referencing GPIO pins
P_magnet = 16 #GPIO23
GPIO.setup(P_magnet, GPIO.OUT)

def Control_magnet(signal=None):
    while True:
        signal = int(input("Please enter 1 to turn on the electromagnet or 0 to turn off the electromagnet"))
        if signal == 1:
            GPIO.output(P_magnet, GPIO.HIGH)
            print("the electromagnet turn on")
            time.sleep(1)
        elif signal == 0:
            GPIO.output(P_magnet, GPIO.LOW)
            print("the electromagnet turn off")
            time.sleep(1)
        elif signal == 2:
            GPIO.cleanup()
            print("the GPIOs clean up")
        else:
            print("Please enter 1 or 0 to control the electromagnet")

Control_magnet()

