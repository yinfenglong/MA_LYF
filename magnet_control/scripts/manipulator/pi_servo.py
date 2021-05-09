#!/usr/bin/env python
# coding=utf-8
import RPi.GPIO as GPIO
import time

P_SERVO = 12 #GPIO18
fpwm = 50 #Hz
a = 6
b = 2

def setup():
    global pwm
    GPIO.setmode(GPIO.BOARD) #Set Pi to use board numbering when referencing GPIO pins
    GPIO.setup(P_SERVO, GPIO.OUT)
    pwm = GPIO.PWM(P_SERVO,fpwm)
    # pwm.start(0)

print("starting")
setup()

def Control_180(angle=None):
    global pwm
    try:
        while True:
            angle = float(input("Please enter the rotation angle"))
            if angle < 0 or angle > 180:
                print("Please enter the correct rotation angle between 0 and 180")
            elif angle > 0 and angle <= 180:
                setup()
                pwm.start(0)
                duty = (a/180) * angle +b #convert angle to duty cycle
                pwm.ChangeDutyCycle(duty) #set duty cycle
                print("angle = %f -> duty = %f" %(angle,duty))
                time.sleep(1.5)
                pwm.stop()
            elif angle == 0:
                pwm.stop()
                print("PWM stop")
    finally:
        pwm.stop()
        GPIO.cleanup()
        print("GPIOs clean up")

Control_180()
