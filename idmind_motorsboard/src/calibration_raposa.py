#!/usr/bin/env python

import rospy
from idmind_motorsboard.msg import WheelsMB


class Calibration:
    def __init__(self):
        self.ticks = {"front_left": 0., "front_right": 0., "back_left": 0., "back_right": 0.}
        rospy.Subscriber("/idmind_motors/ticks", WheelsMB, self.update_odom)
        self.mp = rospy.Publisher("/idmind_motors/set_velocities", WheelsMB, queue_size=10)

    def update_odom(self, msg):
        self.ticks["front_left"] += msg.front_left
        self.ticks["front_right"] += msg.front_right

    def start(self):
        r = rospy.Rate(20)
        msg = WheelsMB()
        msg.header.frame_id = "odom"
        msg.header.stamp = rospy.Time.now()
        msg.front_right = 0.1
        msg.front_left = -0.1

        while not rospy.is_shutdown():
            self.mp.publish(msg)
            print(self.ticks)
            r.sleep()