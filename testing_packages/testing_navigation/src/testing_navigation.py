#!/usr/bin/env python
import rospy
from numpy import pi, sin, cos, sign
from math import sqrt
from geometry_msgs.msg import TransformStamped, Quaternion
from sensor_msgs.msg import Imu, LaserScan
from testing_motorsboard.msg import MotorsState
from testing_motorsboard.srv import SetMotorVelocity
from tf2_geometry_msgs import PoseStamped
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from tf_conversions import transformations
from nav_msgs.msg import Odometry

MOVE_WHEELS = False

SENSOR_FRAME = "front_laser"
BASE_FRAME = "base_link"

WHEEL_RADIUS = 0.09
L = 0.6
TICKS_FRONT_RIGHT = 120000
TICKS_FRONT_LEFT = 120000

V = 0
THETA = -pi / 4
MAX_V_THETA = 0.7
SAFE_DISTANCE = .3
MAX_DISTANCE = 0.4


# wheels_odom = {'right': 0, 'left': 0}


class TestingNavigation:
    def __init__(self):
        self.tf_buffer = Buffer()
        self.transform_listener = TransformListener(self.tf_buffer)

        self.odom_time = rospy.Time.now()
        self.wheel_perimeter = 2 * pi * WHEEL_RADIUS
        print "odom_time = %s" % self.odom_time.to_sec()
        # publish and broadcast odometry
        self.odom_broadcast = TransformBroadcaster()
        self.odom_publisher = rospy.Publisher("/odom", Odometry, queue_size=10)

        self.imu = Imu()
        self.imu_orientation_offset = Quaternion()
        self.imu_orientation_offset.w = 1.

        self.laser_scan = LaserScan()

        # f = rospy.get_param("/move_base/local_costmap/footprint")[2:-2].split("],[")
        footprint = [[0.45, 0.2], [0.45, -0.2], [-0.3, -0.2], [-0.3, 0.2]]

        self.robot_edges = [.45, -.2, -.3, .2]
        self.front_dist, self.back_dist, self.right_dist, self.left_dist = 1e6, 1e6, 1e6, 1e6
        self.x, self.y = 0, 0
        self.theta = 0
        self.dx_linear, self.dy_linear = 0, 0
        self.v_x, self.v_y = 0, 0
        self.v_theta = 0

        self.velocity_right, self.velocity_left = 0., 0.

        rospy.Subscriber('/testing_imu/imu', Imu, self.update_imu)
        print "init finished"

    def update_imu(self, imu_msg):
        # print "imu value updated: orientation = [%s, %s, %s, %s]\t ang_velocity = [%s, %s, %s]\t lin_acceleration = [%s, %s, %s]" % (self.imu.orientation.x,
        #                self.imu.orientation.y, self.imu.orientation.z, self.imu.orientation.w,
        #                self.imu.angular_velocity.x, self.imu.angular_velocity.y, self.imu.angular_velocity.z,
        #                self.imu.linear_acceleration.x, self.imu.linear_acceleration.y, self.imu.linear_acceleration.z)
        self.imu = imu_msg

    def calibrate_imu(self):
        rate = rospy.Rate(20)
        is_calibrated = False
        while not is_calibrated and not rospy.is_shutdown():
            rospy.wait_for_message('/imu', Imu)

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

    def update_laser(self, msg):
        if msg.header.frame_id == SENSOR_FRAME:
            if not self.tf_buffer.can_transform(msg.header.frame_id, BASE_FRAME, rospy.Time.now()):
                print "ERROR - invalid laser frame"
            else:
                # print "laser_angle_min, laser_angle_max = %s, %s" % (msg.angle_min, msg.angle_max)
                beam_pose = PoseStamped()
                angle = msg.angle_min
                front_dist = back_dist = right_dist = left_dist = 1e6
                for beam in msg.ranges:
                    if msg.range_min < beam < msg.range_max:
                        beam_pose.header.frame_id = msg.header.frame_id
                        beam_pose.pose.position.x = beam * cos(angle)
                        beam_pose.pose.position.y = beam * sin(angle)

                        beam_real = self.tf_buffer.transform(beam_pose, BASE_FRAME)
                        obs = [beam_real.pose.position.x, beam_real.pose.position.y]

                        # compute distances to obstacles
                        if self.robot_edges[1] < obs[1] < self.robot_edges[3]:
                            if obs[0] > 0.:
                                df = obs[0] - self.robot_edges[0]
                                front_dist = df if df < front_dist else front_dist
                            elif obs[0] < 0.:
                                db = abs(obs[0] - self.robot_edges[2])
                                back_dist = db if db < back_dist else back_dist
                        if self.robot_edges[2] < obs[0] < self.robot_edges[0]:
                            if obs[1] < 0.:
                                dr = abs(obs[1] - self.robot_edges[1])
                                right_dist = dr if dr < right_dist else right_dist
                            elif obs[1] > 0:
                                dl = obs[1] - self.robot_edges[3]
                                left_dist = dl if dl < left_dist else left_dist

                    angle += msg.angle_increment

                self.front_dist = front_dist
                self.back_dist = back_dist
                self.right_dist = right_dist
                self.left_dist = left_dist
                # print "laser - front = %s, back = %s, left = %s, right = %s" % (self.front_dist, self.back_dist, self.right_dist, self.left_dist)

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

    def handle_updates(self):
        # print "handle_updates"
        orientation = self.imu.orientation
        imu_euler_real = transformations.euler_from_quaternion([orientation.x, orientation.y,
                                                                orientation.z, orientation.w])
        # print "imu_euler_real = (%s, %s, %s)" % (imu_euler_real[0], imu_euler_real[1], imu_euler_real[2])

        # transform imu reading from base_link_imu to base_link
        imu_pose = PoseStamped()
        imu_pose.header.frame_id = "base_link_imu"
        # imu_pose.pose.orientation = orientation
        imu_pose.pose.orientation = Quaternion(x=orientation.x, y=orientation.y, z=orientation.z, w=orientation.w)
        imu_real = self.tf_buffer.transform(imu_pose, BASE_FRAME)

        # print "handle_updates -> imu_real = %s" % str(imu_real)

        orientation_base = imu_real.pose.orientation
        orientation_offset = self.imu_orientation_offset

        imu_euler_base_link = transformations.euler_from_quaternion([orientation_base.x, orientation_base.y,
                                                                orientation_base.z, orientation_base.w])
        # print "imu_euler_base_link = (%s, %s, %s)" % (imu_euler_base_link[0], imu_euler_base_link[1], imu_euler_base_link[2])

        # quaternion_multiply(new_rotation, original_rotation)
        orientation_rectified = transformations.quaternion_multiply(
            [orientation_base.x, orientation_base.y, orientation_base.z, orientation_base.w],
            [orientation_offset.x, orientation_offset.y, orientation_offset.z, orientation_offset.w])

        # print "orientation_base = %s" % str(orientation_base)
        # print "orientation_offset = %s" % str(orientation_offset)
        # print "orientation_rectified = %s" % str(orientation_rectified)

        imu_euler = transformations.euler_from_quaternion([orientation_rectified[0], orientation_rectified[1],
                                                           orientation_rectified[2], orientation_rectified[3]])
        # print "imu_euler_rectified = (%s, %s, %s)" % (imu_euler[0], imu_euler[1], imu_euler[2])
        dtheta = imu_euler[2] - self.theta
        if dtheta > pi:
            dtheta = dtheta - 2 * pi
        if dtheta < -pi:
            dtheta = dtheta + 2 * pi

        self.theta += dtheta

        print "theta = %s\t dtheta = %s" % (self.theta, dtheta)

        # print "odom_time = %s, now = %s" % (self.odom_time.to_sec(), rospy.Time.now().to_sec())
        dt = (rospy.Time.now() - self.odom_time).to_sec()
        self.odom_time = rospy.Time.now()

        dx = self.dx_linear * cos(self.theta) - self.dy_linear * sin(self.theta)
        dy = self.dx_linear * sin(self.theta) + self.dy_linear * cos(self.theta)
        self.x += dx
        self.y += dy

        self.v_x = dx / dt
        self.v_y = dy / dt
        self.v_theta = dtheta / dt

        print "x, y, theta = %s, %s, %s" % (self.x, self.y, self.theta)
        print "dt = %s\t dx, dy = %s, %s\t x, y = (%s, %s)" % (dt, dx, dy, self.x, self.y)
        print "vx, vy, vtheta = %s, %s, %s" % (self.v_x, self.v_y, self.v_theta)

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

        theta_goal = THETA - self.theta
        v_theta_goal = MAX_V_THETA  # sign(theta_goal) * min(abs(theta_goal/dt), MAX_V_THETA)
        print "theta_goal, v_theta_goal = %s, %s" % (theta_goal, v_theta_goal)

        if self.front_dist < SAFE_DISTANCE:
            print "obstacles in front, d = %s" % self.front_dist
            self.velocity_right = 0
            self.velocity_left = 0
        elif abs(theta_goal) < 0.01:
            print "goal angle achieved"
            self.velocity_right = 0
            self.velocity_left = 0
        else:
            self.velocity_right = (V + v_theta_goal * L / 2)
            self.velocity_left = -(V - v_theta_goal * L / 2)

        print "velocity_right, velocity_left = %s, %s" % (self.velocity_right, self.velocity_left)

    def get_total_distance(self):
        return sqrt(self.x ** 2 + self.y ** 2)

    def set_constant_velocity(self, v):
        self.velocity_right = v
        self.velocity_left = -v

    def start(self):
        if MOVE_WHEELS:
            # self.set_constant_velocity(250)
            self.velocity_right = 300
            self.velocity_left = -300

        rospy.wait_for_service('/testing_motorsboard/set_motor_velocity')
        print "wheel velocity service is ready"

        velocity_service = rospy.ServiceProxy('/testing_motorsboard/set_motor_velocity', SetMotorVelocity)
        rospy.Subscriber('/testing_motors/motors_state', MotorsState, self.update_motors)
        # rospy.Subscriber('/imu', Imu, self.update_imu)
        rospy.Subscriber('/testing_imu/imu', Imu, self.update_imu)
        print "motors and imu topics subscribed"

        rospy.wait_for_message("/front_scan", LaserScan, timeout=5)
        rospy.Subscriber("/front_scan", LaserScan, self.update_laser)
        print "laser topic subscribed"

        self.calibrate_imu()
        print ("IMU calibration finished")
        print ("------------------------------------------------------------------")
        print ("------------------------------------------------------------------\n\n")

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.handle_updates()
            velocity_service(self.velocity_right, self.velocity_left)
            if self.get_total_distance() > MAX_DISTANCE:
                self.set_constant_velocity(0)

            print "-------------------------------------------------------\n"
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node('testing_navigation')

    testing_navigation = TestingNavigation()
    testing_navigation.start()

    print "testing_navigation node finished"
