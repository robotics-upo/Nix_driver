#!/usr/bin/env python

import rospy

rospy.init_node("sensor_test")
r = rospy.Rate(20)

while not rospy.is_shutdown():
    try:
        r.sleep()
    except KeyboardInterrupt:
        break
