#! /usr/bin/env python



import rospy
from sensor_msgs.msg import Imu
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf import transformations
import math


class Node(object):

    def __init__(self):
        self.br = tf2_ros.TransformBroadcaster()
        rospy.Subscriber("imu", Imu, self.on_data)

    def on_data(self, msg):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "base_link_imu"
        t.child_frame_id = "imu"

        t.transform.rotation.x = msg.orientation.x
        t.transform.rotation.y = msg.orientation.y
        t.transform.rotation.z = msg.orientation.z
        t.transform.rotation.w = msg.orientation.w

        self.br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node("viz_imu")
    node = Node()
    rospy.loginfo("viz_imu: running")
    rospy.spin()