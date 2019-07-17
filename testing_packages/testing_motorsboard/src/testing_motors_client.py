#!/usr/bin/env python
import sys
import rospy
from rospy_tutorials.srv import AddTwoInts
from testing_motorsboard.srv import SetMotorVelocity


def set_velocities(velocity_right, velocity_left):
    rospy.wait_for_service('/testing_motorsboard/set_motor_velocity')
    try:
        velocity = rospy.ServiceProxy('/testing_motorsboard/set_motor_velocity', SetMotorVelocity)

        print "setting velocity: %s, %s" % (velocity_right, velocity_left)
        return velocity(velocity_right, velocity_left)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


if __name__ == "__main__":
    if len(sys.argv) == 3:
        velocity_right = int(sys.argv[1])
        velocity_left = int(sys.argv[2])
    else:
        print "%s [velocity_right velocity_left]" % sys.argv[0]
        sys.exit(1)

    is_velocity_set = set_velocities(velocity_right, velocity_left)
