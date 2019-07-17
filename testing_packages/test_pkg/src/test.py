#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from idmind_motorsboard.msg import WheelsMB

class Test:
    def __init__(self):
        self.y = WheelsMB()
        rospy.Subscriber("/joy", Joy, self.joy_callback)
        self.pub = rospy.Publisher("/idmind_motors/set_velocities", WheelsMB, queue_size=10)
        pass

    def joy_callback(self, msg):
        vel = msg.axes[5]
        wheels = WheelsMB()
        wheels.front_right = vel
        wheels.front_left = vel
        wheels.header.stamp = rospy.Time.now()
        self.y = wheels



    def start(self):
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            try:
                rospy.loginfo("start - try")
                self.y.header.stamp = rospy.Time.now()
                self.pub.publish(self.y)

                r.sleep()
            except KeyboardInterrupt:
                rospy.loginfo("msg")
                break


if __name__=="__main__":
    # rospy.init_node("new_node")
    # t = Test()
    # t.start()
    footprint = [[0.45,0.2],[0.45,-0.2],[-0.3,-0.2],[-0.3,0.2]]
    edges = [0, 0, 0, 0]
    for p in footprint:
        edges[0] = p[0] if p[0] > edges[0] else edges[0]
        edges[2] = p[0] if p[0] < edges[2] else edges[2]
        edges[1] = p[1] if p[1] < edges[1] else edges[1]
        edges[3] = p[1] if p[1] > edges[3] else edges[3]

    print "%s" % edges
