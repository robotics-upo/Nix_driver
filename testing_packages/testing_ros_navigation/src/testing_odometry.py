#!/usr/bin/env python
import rospy
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from tf2_geometry_msgs import PoseStamped
from geometry_msgs.msg import TransformStamped, Quaternion
from tf_conversions import transformations
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from numpy import pi, cos, sin
from testing_motorsboard.msg import MotorsState

WHEEL_RADIUS = 0.09
BASE_FRAME = "base_link"
TICKS_FRONT_RIGHT = 120000
TICKS_FRONT_LEFT = 120000


class TestingOdometry:
    def __init__(self):
        self.tf_buffer = Buffer()
        self.transform_listener = TransformListener(self.tf_buffer)

        self.dx_linear, self.dy_linear = 0, 0

        self.odom_time = rospy.Time.now()
        self.wheel_perimeter = 2 * pi * WHEEL_RADIUS
        self.odom_broadcast = TransformBroadcaster()
        self.odom_publisher = rospy.Publisher("/odom", Odometry, queue_size=10)

        self.imu = Imu()
        self.imu_orientation_offset = Quaternion()
        self.imu_orientation_offset.w = 1.

        self.x, self.y = 0, 0
        self.theta = 0
        self.v_x, self.v_y = 0, 0
        self.v_theta = 0

    def calibrate_imu(self):
        rate = rospy.Rate(20)
        is_calibrated = False

        print ("starting imu calibration ...")
        while not is_calibrated and not rospy.is_shutdown():
            print ("waiting for imu message ...")
            rospy.wait_for_message('/testing_imu/imu', Imu)
            print "imu message received"

            try:
                orientation = self.imu.orientation
                # print "calibrate_imu -> orientation = %s" % str(orientation)
                imu_pose = PoseStamped()
                imu_pose.header.frame_id = "base_link_imu"
                # imu_pose.pose.orientation = orientation
                imu_pose.pose.orientation = Quaternion(x=orientation.x, y=orientation.y, z=orientation.z,
                                                       w=orientation.w)
                # print "calibrate_imu -> imu_pose.pose.orientation = %s" % str(orientation)

                imu_real = self.tf_buffer.transform(imu_pose, BASE_FRAME)
                # print "imu_real = %s" % str(imu_real)
                self.imu_orientation_offset.x = imu_real.pose.orientation.x
                self.imu_orientation_offset.y = imu_real.pose.orientation.y
                self.imu_orientation_offset.z = imu_real.pose.orientation.z
                self.imu_orientation_offset.w = -imu_real.pose.orientation.w

                # print "imu_orientation_offset = %s" % str(self.imu_orientation_offset)

                imu_offset_euler = transformations.euler_from_quaternion([self.imu_orientation_offset.x,
                                                                          self.imu_orientation_offset.y,
                                                                          self.imu_orientation_offset.z,
                                                                          self.imu_orientation_offset.w])
                print "imu_orientation_offset = [%s, %s, %s, %s]" % (self.imu_orientation_offset.x,
                                                                     self.imu_orientation_offset.y,
                                                                     self.imu_orientation_offset.z,
                                                                     self.imu_orientation_offset.w)
                print "imu_euler_offset = (%s, %s, %s)" % (imu_offset_euler[0], imu_offset_euler[1], imu_offset_euler[2])
                # self.theta = imu_offset_euler[2]

                is_calibrated = True
            except Exception as e:
                print("error calibrating IMU, will try again\t e = " + str(e))

            rate.sleep()

    def update_motors(self, motors_state):
        # print "motors_state.right_wheel_velocity = %s" % motors_state.right_wheel_velocity
        # print "wheel_perimeter = %s" % self.wheel_perimeter
        # print "TICKS_FRONT_RIGHT = %s" % TICKS_FRONT_RIGHT
        # print "math = %s" % (self.wheel_perimeter * motors_state.right_wheel_velocity / TICKS_FRONT_RIGHT)

        right_wheel = self.wheel_perimeter * motors_state.right_wheel_velocity / TICKS_FRONT_RIGHT
        left_wheel = self.wheel_perimeter * motors_state.left_wheel_velocity / TICKS_FRONT_LEFT

        # print "right_wheel = %s" % right_wheel
        # print "left_wheel = %s" % left_wheel
        # wheels_odom['right'] += (wheel_perimeter * right_wheel/TICKS_FRONT_RIGHT)
        # wheels_odom['left'] += (wheel_perimeter * left_wheel/TICKS_FRONT_LEFT)

        # right and left wheels have reverse values, right_wheel is positive when the robot is moving forward
        self.dx_linear = (right_wheel - left_wheel) / 2
        # print "dx_linear = %s" % self.dx_linear

    def update_imu(self, imu_msg):
        self.imu = imu_msg

    def handle_updates(self):
        orientation = self.imu.orientation
        imu_pose = PoseStamped()
        imu_pose.header.frame_id = "base_link_imu"
        imu_pose.pose.orientation = Quaternion(x=orientation.x, y=orientation.y, z=orientation.z, w=orientation.w)
        imu_real = self.tf_buffer.transform(imu_pose, BASE_FRAME)

        orientation_base = imu_real.pose.orientation
        orientation_offset = self.imu_orientation_offset

        orientation_rectified = transformations.quaternion_multiply(
            [orientation_base.x, orientation_base.y, orientation_base.z, orientation_base.w],
            [orientation_offset.x, orientation_offset.y, orientation_offset.z, orientation_offset.w])

        imu_euler = transformations.euler_from_quaternion([orientation_rectified[0], orientation_rectified[1],
                                                           orientation_rectified[2], orientation_rectified[3]])
        dtheta = imu_euler[2] - self.theta
        if dtheta > pi:
            dtheta = dtheta - 2*pi
        if dtheta < -pi:
            dtheta = dtheta + 2*pi

        if self.theta > pi:
            self.theta -= 2*pi
        if self.theta < -pi:
            self.theta += 2*pi
        self.theta += dtheta

        dt = (rospy.Time.now() - self.odom_time).to_sec()
        self.odom_time = rospy.Time.now()
        dx = self.dx_linear * cos(self.theta) - self.dy_linear * sin(self.theta)
        dy = self.dx_linear * sin(self.theta) + self.dy_linear * cos(self.theta)
        self.x += dx
        self.y += dy

        self.v_x = dx / dt
        self.v_y = dy / dt
        self.v_theta = dtheta / dt

        tf_orientation = transformations.quaternion_from_euler(0, 0, self.theta)
        q_orientation = Quaternion(tf_orientation[0], tf_orientation[1], tf_orientation[2], tf_orientation[3])

        # transform message for Broadcast
        transform = TransformStamped()
        transform.header.stamp = self.odom_time
        transform.header.frame_id = "odom"
        transform.child_frame_id = BASE_FRAME
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation = q_orientation
        self.odom_broadcast.sendTransform(transform)

        # odometry message to publish in /odom topic
        odometry = Odometry()
        odometry.header.stamp = self.odom_time
        odometry.header.frame_id = "odom"
        odometry.child_frame_id = BASE_FRAME
        odometry.pose.pose.position = [self.x, self.y, 0]
        odometry.pose.pose.orientation = q_orientation
        odometry.twist.twist.linear = [self.v_x, self.v_y, 0]
        odometry.twist.twist.angular = [0, 0, self.v_theta]
        self.odom_publisher.publish(odometry)

    def start(self):
        rospy.Subscriber('/testing_motors/motors_state', MotorsState, self.update_motors)
        # rospy.Subscriber('/imu', Imu, self.update_imu)
        rospy.Subscriber('/testing_imu/imu', Imu, self.update_imu)
        print "motors and imu topics subscribed"

        self.calibrate_imu()
        print ("IMU calibration finished")
        print ("------------------------------------------------------------------")
        print ("------------------------------------------------------------------\n\n")

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.handle_updates()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node('testing_odometry')

    testing_odometry = TestingOdometry()
    testing_odometry.start()

    print "testing_odometry node finished"