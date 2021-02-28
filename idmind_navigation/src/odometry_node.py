#!/usr/bin/env python

import rospy
from idmind_robot.msg import Log
from idmind_motorsboard.msg import WheelsMB
from nav_msgs.msg import Odometry
from tf2_geometry_msgs import PoseStamped
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from tf_conversions import transformations
from numpy import pi, sin, cos
from geometry_msgs.msg import TransformStamped, Quaternion
from threading import Lock

VERBOSE = 5
LOGS = 5


class IDMindOdometry:
    """
    Class responsible for simple navigation functions:
        - Publish odometry, either from Gazebo or calculating from motor ticks and IMU readings
        - Convert Twist messages from different controllers to motor velocities or to Gazebo
    """
    def __init__(self):

        ###############
        #   Logging   #
        ###############
        self.logging = rospy.Publisher("/idmind_logging", Log, queue_size=10)

        ######################
        #  Robot Parameters  #
        ######################
        self.base_width = rospy.get_param("/bot/base_width", default=0.7)
        self.control_freq = rospy.get_param("/move_base/controller_frequency", default=20.)
        self.publish_tf = rospy.get_param("/publish_tf", default=True) # UPO: added to switch between mapping and navigation

            #########################
            #  Hardware connection  #
            #########################
        waiting = True
        while waiting:
            try:
                self.log("Waiting for MotorBoard", 5)
                rospy.wait_for_message("/idmind_motors/wheel_odom", WheelsMB, timeout=1)
                self.wheels_lock = Lock()
                self.wheel_odom = {"front_right": 0., "front_left": 0., "back_right": 0., "back_left": 0.}
                rospy.Subscriber("/idmind_motors/wheel_odom", WheelsMB, self.handle_wheel_odom)
                waiting = False
            except rospy.ROSException:
                self.log("Motor Board not responding, waiting 2 secs", 1, alert="warn")
                rospy.sleep(2)

        ##############
        #  Odometry  #
        ##############
        self.new_encoder = False
        self.odom_time = rospy.Time.now()
        self.last_imu_time = None
        # Current Odom state
        self.x = 0
        self.y = 0
        self.z = 0
        self.th = 0
        self.roll = 0
        self.pitch = 0
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.vth = 0

        # Publish odometry and broadcast odometry
        self.odom_broadcast = TransformBroadcaster()
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)

        self.tf_buffer = Buffer()
        self.transform_listener = TransformListener(self.tf_buffer)

        self.log("Node initialized", 5)

    #################################
    #  Callbacks and other updates  #
    #################################


    def handle_wheel_odom(self, msg):
        """
        Callback to messages received in /idmind_motors/wheel_odom
        :return:
        """
        try:
            self.wheels_lock.acquire()
            self.wheel_odom["front_right"] = self.wheel_odom["front_right"] + msg.front_right
            self.wheel_odom["front_left"] = self.wheel_odom["front_left"] + msg.front_left
            self.wheel_odom["back_right"] = self.wheel_odom["back_right"] + msg.back_right
            self.wheel_odom["back_left"] = self.wheel_odom["back_left"] + msg.back_left
            self.wheels_lock.release()
        except AttributeError as a_err:
            self.log("Lock is probably not defined yet: {}".format(a_err), 7)

    def update_odometry(self, msg):
        # Consider adding noise
        self.simul_odom = msg

    def log(self, msg, msg_level, log_level=-1, alert="info"):
        if VERBOSE >= msg_level:
            if alert == "info":
                rospy.loginfo("{}: {}".format(rospy.get_name(), msg))
            elif alert == "warn":
                rospy.logwarn("{}: {}".format(rospy.get_name(), msg))
            elif alert == "error":
                rospy.logerr("{}: {}".format(rospy.get_name(), msg))
        if LOGS >= (log_level if log_level != -1 else msg_level):
            self.logging.publish(rospy.Time.now().to_sec(), rospy.get_name(), msg)

    ##############
    #  Odometry  #
    ##############
    def broadcast_odometry(self):
        """
        Broadcasts the odometry based in tick readings and imu readings, if available
        Currently supports only 2wd
        :return:
        """
        
        ###############
        #    ROBOT    #
        ###############
        self.log("Broadcasting odometry for Robot", 7)
        last_time = self.odom_time
        self.odom_time = rospy.Time.now()

        try:
            # Start by computing the linear and angular displacement
            # Get state changes
            self.wheels_lock.acquire()
            dleft = self.wheel_odom["front_left"]
            dright = self.wheel_odom["front_right"]
            self.wheel_odom["front_left"] = 0.
            self.wheel_odom["front_right"] = 0.
            self.wheels_lock.release()
            dlinear_x = (dright - dleft) / 2 
            dlinear_y = 0
            dlinear_z =  (dright - dleft) / 2 
            # If IMU is not usable, use simulation odometry readings
            dth = (dright + dleft) / self.base_width
            if dth > pi:
                dth = dth - 2 * pi
            if dth < - pi:
                dth = dth + 2 * pi
            #if self.th > pi:
            #    self.th = self.th - 2 * pi
            #if self.th < - pi:
            #    self.th = self.th + 2 * pi
            dx = dlinear_x * cos(self.th + dth) - dlinear_y * sin(self.th + dth)
            dy = dlinear_x * sin(self.th + dth) + dlinear_y * cos(self.th + dth)
            dz = dlinear_z 
        except Exception as err:
            rospy.logfatal("{}: Exception computing odometry in real robot: {}".format(rospy.get_name(), err))
            return

        # Based on dx, dy and dth, compute, publish and broadcast odometry
        try:
            # Compute odometry positions
            self.x = self.x + dx
            self.y = self.y + dy
            self.z = self.z + dz
            self.th = self.th + dth
            
            self.vx = dx / (self.odom_time - last_time).to_sec()
            self.vy = dy / (self.odom_time - last_time).to_sec()
            self.vz = dz / (self.odom_time - last_time).to_sec()
            self.vth = dth / (self.odom_time - last_time).to_sec()

            # Build Transform message for Broadcast
            transf_msg = TransformStamped()
            transf_msg.header.stamp = self.odom_time
            transf_msg.header.frame_id = "odom"
            transf_msg.child_frame_id = "base_link"

            transf_msg.transform.translation.x = self.x
            transf_msg.transform.translation.y = self.y

            q = transformations.quaternion_from_euler(self.roll, self.pitch, self.th)
            transf_msg.transform.rotation = Quaternion(*q)
            if self.publish_tf:
                self.odom_broadcast.sendTransform(transf_msg)

            # Build Odometry Message
            odom_msg = Odometry()
            odom_msg.header.stamp = self.odom_time
            odom_msg.header.frame_id = "odom"

            odom_msg.child_frame_id = "base_link"
            odom_msg.pose.pose.position.x = self.x
            odom_msg.pose.pose.position.y = self.y
            odom_msg.pose.pose.orientation = Quaternion(*q)
            odom_msg.twist.twist.linear.x = self.vx
            odom_msg.twist.twist.linear.y = self.vy
            odom_msg.twist.twist.angular.z = self.vth

            self.odom_pub.publish(odom_msg)
        except Exception as err:
            rospy.logfatal("{}: Exception in BroadCast Odometry: {}".format(rospy.get_name(), err))
            return

    def start(self):
        r = rospy.Rate(self.control_freq)

        while not rospy.is_shutdown():
            try:
                self.broadcast_odometry()
                r.sleep()
            except KeyboardInterrupt:
                break
            except Exception as err:
                rospy.logerr(rospy.get_name()+" in start():" + str(err))

        rospy.logwarn(rospy.get_name()+": Shutting down")


if __name__ == "__main__":
    rospy.init_node("odometry_node")
    odom = IDMindOdometry()
    odom.start()
