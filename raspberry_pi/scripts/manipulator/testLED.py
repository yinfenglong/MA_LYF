#!/usr/bin/env python
# coding=utf-8

import time

def test_key(a=None):
    try:
        while True:
            a = int(input("Please enter 0 or 1 or 2"))
            if a == 1:
                print("magnet turn on")
            elif a == 0:
                print("magnet turn off")
            elif a == 2:
                print("done")
    finally:
        print("done")

test_key()
