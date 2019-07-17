#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Int32
from idmind_motorsboard.msg import WheelsMB

rospy.init_node("driver_test")
r = rospy.Rate(20)
mp = rospy.Publisher("/idmind_motors/set_velocities", WheelsMB, queue_size=10)
ap = rospy.Publisher("/idmind_motors/set_arm", Int32, queue_size=10)


while not rospy.is_shutdown():
    try:
        msg = WheelsMB()
        msg.header.frame_id = "odom"
        msg.header.stamp = rospy.Time.now()
        msg.front_right = 0.1
        msg.front_left = -0.1
        mp.publish(msg)

        # msg_arm = Int32()
        # msg_arm.data = 532
        # ap.publish(msg_arm)

        time.sleep(0.1)
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    except Exception as ros_err:
        print ros_err
