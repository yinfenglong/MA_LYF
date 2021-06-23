#!/usr/bin/env python3
# coding=utf-8
'''
Author: Yinfeng
'''
import _thread, time
import rospy
import sys 
import RPi.GPIO as GPIO
from std_srvs.srv import Trigger, TriggerResponse

pubMagnet = False

# GPIO.setmode(GPIO.BOARD)
P_magnet = 16 #GPIO23
# GPIO.setup(P_magnet, GPIO.OUT)

def magnet_thread():
    while True:
        if pubMagnet:
            # GPIO.setmode(GPIO.BOARD) #Set Pi to use board numbering when referencing GPIO pins 
            # global P_magnet
            # P_magnet = 16 #GPIO23
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(P_magnet, GPIO.OUT)
            GPIO.output(P_magnet, GPIO.HIGH)
        else:
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(P_magnet, GPIO.OUT)
            GPIO.output(P_magnet, GPIO.LOW)
            time.sleep(0.1)
            GPIO.cleanup()
            
        time.sleep(1)           

def magnetCallback(req):
    global pubMagnet
    pubMagnet = bool(1-pubMagnet)

    rospy.loginfo("Publish Magnet state! [%d]", pubMagnet)

    return TriggerResponse(1, "Magnet!")

def magnet_server():
    rospy.init_node('magnet_server')
    s = rospy.Service('/magnet_control', Trigger, magnetCallback)
    GPIO.setmode(GPIO.BOARD)
    print("Ready to receive magnet comman.")
    _thread.start_new_thread(magnet_thread, ())
    rospy.spin()

if __name__ == "__main__":
    magnet_server()
