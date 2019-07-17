#!/usr/bin/env python

import time
from idmind_motors import MotorBoard

print "========== Wheels Calibration =========="
m = MotorBoard("/dev/idmind-motorsboard", baudrate=57600, timeout=0.5)

ticks = {"front_left": 0., "front_right": 0., "back_left": 0., "back_right": 0.}
print "Started counting ticks. Press Ctrl+C to stop."
while True:
    try:
        if m.get_velocity_motor_ticks():
            ticks["front_left"] = ticks["front_left"] + m.ticks["front_left"]
            ticks["front_right"] = ticks["front_right"] + m.ticks["front_right"]
            ticks["back_left"] = ticks["back_left"] + m.ticks["back_left"]
            ticks["back_right"] = ticks["back_right"] + m.ticks["back_right"]
        time.sleep(0.1)
    except KeyboardInterrupt:
        print "Stopped counting ticks."
        break

print "Ticks/Turn:"
print "\tFront left: {}".format(ticks["front_left"]/10)
print "\tFront right: {}".format(ticks["front_right"]/10)
print "\tBack left: {}".format(ticks["back_left"]/10)
print "\tBack right: {}".format(ticks["back_right"]/10)

print "========================================"