#! /usr/bin/env python



from serial import Serial
import math
import rospy
from tf import transformations


class Node(object):

    def __init__(self):
        self.board = Serial("/dev/ttyACM0", baudrate=115200)

    def run(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            rate.sleep()
            line = self.board.readline()
            orientation = line.split("|")[0]
            w, x, y, z = [float(el) for el in orientation[3:-1].split(" ")]
            euler = transformations.euler_from_quaternion((x, y, z, w))
            print "%.2f" % math.degrees(euler[2])


    def cleanup(self):
        self.board.close()


if __name__ == '__main__':
    rospy.init_node("imu_raw_reader")
    node = Node()
    node.run()
    node.cleanup()
    rospy.spin()