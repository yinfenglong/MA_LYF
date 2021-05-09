#!/usr/bin/env python
# coding=utf-8

import time

b = 10
def test_keyboard(a=None):
    while True:
        a = int(input("PLease enter a number"))
        if a < 0 or a > 180:
            print("please enter the correct number")
        elif a > 0 and a <= 180:
            c = a + b
            print("a=%d, b=%d, c=%d" %(a,b,c))
            time.sleep(1)
        elif a == 0:
            print("done")

test_keyboard()
