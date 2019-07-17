#!/usr/bin/env python

import rospy
from idmind_robot.msg import Log
from nav_msgs.msg import Odometry
from idmind_motorsboard.msg import WheelsMB
# from idmind_sensorsboard.msg import SystemVoltages
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse

VERBOSE = 5
LOGS = 5


class IDMindNavigation:
    """
    Class responsible for simple navigation functions:
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
        self.name = rospy.get_param("/bot/name", default="bot")
        self.kinematics = rospy.get_param("/bot/kinematics", default="2wd")
        self.base_width = rospy.get_param("/bot/base_width", default=0.26)
        self.max_robot_vel = rospy.get_param("/bot/max_vel", default=0.7)
        self.simulation = rospy.get_param("/simulation", default=False)
        self.controller = rospy.get_param("/controller", default="joystick")
        self.last_controller = self.controller

        if not self.simulation:
            #########################
            #  Hardware connection  #
            #########################
            waiting = True
            while not rospy.is_shutdown() and waiting:
                try:
                    self.log("Waiting for MotorBoard", 5)
                    rospy.wait_for_message("/idmind_motors/wheel_odom", WheelsMB, timeout=1)
                    waiting = False
                except rospy.ROSException:
                    self.log("Motor Board not responding, waiting 2 secs", 1, alert="warn")
                    rospy.sleep(2)

            # waiting = True
            # while not rospy.is_shutdown() and waiting:
            #     try:
            #         self.log("Waiting for Sensors Board", 5)
            #         rospy.wait_for_message("/idmind_sensors/voltages", SystemVoltages, timeout=1)
            #         self.voltages = SystemVoltages()
            #         rospy.Subscriber("/idmind_sensors/voltages", SystemVoltages, self.handle_voltages)
            #         waiting = False
            #     except rospy.ROSException:
            #         self.log("Sensor Board not responding, waiting 2 secs", 1, alert="warn")
            #         rospy.sleep(2)

        else:
            rospy.wait_for_message("/gazebo/odom", Odometry)
            self.log("Connecting to Gazebo", 7)

        ###########################
        #  Navigation Parameters  #
        ###########################
        max_default = 0.9*self.max_robot_vel
        self.control_freq = rospy.get_param("/move_base/controller_frequency", default=20.)
        self.max_vel_x = rospy.get_param("/move_base/TrajectoryPlannerROS/max_vel_x", default=max_default)
        self.min_vel_x = rospy.get_param("/move_base/TrajectoryPlannerROS/min_vel_x", default=0.1)
        if self.kinematics == "omni":
            self.max_vel_y = rospy.get_param("/move_base/TrajectoryPlannerROS/max_vel_y", default=max_default)
            self.min_vel_y = rospy.get_param("/move_base/TrajectoryPlannerROS/min_vel_y", default=0.1)
        else:
            self.max_vel_y = 0
            self.min_vel_y = 0
        self.max_rot_vel = rospy.get_param("/move_base/TrajectoryPlannerROS/max_vel_theta", default=max_default)
        self.min_rot_vel = rospy.get_param("/move_base/TrajectoryPlannerROS/min_vel_theta", default=0.1)
        # Real acceleration is reduced - this gives better results.
        self.max_acc = rospy.get_param("/move_base/TrajectoryPlannerROS/acc_lim_x", default=0.4)
        self.max_rot_acc = rospy.get_param("/move_base/TrajectoryPlannerROS/acc_lim_theta", default=0.4)

        self.max_dacc = -self.max_acc * 2.0
        self.max_rot_dacc = self.max_rot_acc * 2.0  # Stay positive

        #######################
        #  Navigation Topics  #
        #######################
        # Keep current and last twist publications
        self.twist = Twist()
        self.last_twist = self.twist
        self.last_mb = rospy.Time.now()
        self.last_ctrl = rospy.Time.now()

        # Subscribe to 3 sources of velocities: move_base, joystick and controller
        rospy.Subscriber("/cmd_vel", Twist, callback=self.movebase_callback)
        rospy.Subscriber("/cmd_vel_joy", Twist, callback=self.joy_callback)
        rospy.Subscriber("/cmd_vel_idmind", Twist, callback=self.controller_callback)

        # Create publisher for wheels in simulation or robot
        if self.simulation:
            self.gazebo_pub = rospy.Publisher("/gazebo/cmd_vel", Twist, queue_size=10)
        else:
            self.vel_pub = rospy.Publisher("/idmind_motors/set_velocities", WheelsMB, queue_size=10)

        #################
        #  Controllers  #
        #################
        self.current_state = ""

        ###############################################
        #  Relevant External sensors (bumpers, doors) #
        ###############################################

        ###################
        #  Init Complete  #
        ###################
        self.ready = True
        rospy.Service("/idmind_navigation/toggle_joystick", Trigger, self.toggle_joystick)
        rospy.Service("/idmind_navigation/toggle_controller", Trigger, self.toggle_controller)
        rospy.Service("/idmind_navigation/ready", Trigger, self.report_ready)
        rospy.loginfo(rospy.get_name()+": Node initialized")

    #################################
    #  Callbacks and other updates  #
    #################################
    def report_ready(self, _req):
        """ Simple Service callback to show node is ready """
        return TriggerResponse(self.ready, "IDMind Navigation is " + ("ready" if self.ready else "not ready"))

    def toggle_joystick(self, _req):
        if self.controller == "joystick":
            # Special case, when the default controller is already joystick
            if self.last_controller == "joystick":
                self.controller = "move_base"
            else:
                self.controller = self.last_controller
        else:
            self.last_controller = self.controller
            self.controller = "joystick"
        rospy.logwarn("Controller is now {}".format(self.controller))
        return TriggerResponse(True, "Controller is now {}".format(self.controller))

    def toggle_controller(self, _req):
        self.twist = Twist()
        if self.controller == "joystick":
            if self.last_controller == "move_base":
                self.last_controller = "controller"
            else:
                self.last_controller = "move_base"
            rospy.logwarn("Autonomous Controller is now {}".format(self.last_controller))
            return TriggerResponse(True, "Last Controller is now {}".format(self.controller))
        else:
            if self.controller == "move_base":
                self.controller = "controller"
            else:
                self.controller = "move_base"
            rospy.logwarn("Controller is now {}".format(self.controller))
            return TriggerResponse(True, "Controller is now {}".format(self.controller))

    def update_state(self, msg):
        self.current_state = msg.data

    def joy_callback(self, msg):
        """
        Callback to messages received in /cmd_vel_joy
        :return:
        """
        if self.controller == "joystick":
            self.twist = msg

    def movebase_callback(self, msg):
        """
        Callback to messages received in /cmd_vel
        :return:
        """
        if self.controller == "move_base":
            self.last_mb = rospy.Time.now()
            self.twist = msg

    def controller_callback(self, msg):
        """
        Callback to messages received in /cmd_vel_idmind
        :return:
        """
        if self.controller == "controller":
            self.last_ctrl = rospy.Time.now()
            self.twist = msg

    def handle_voltages(self, msg):
        """
        Callback to messages received in /idmind_sensors/voltages
        :return:
        """
        self.voltages = msg

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

    ################
    #  Controller  #
    ################

    def compute_velocity(self):
        """
        This method applies smoothing and limits velocities, based on planner configuration.
        Limits due to hardware are applied on idmind_motorsboard, based on wheel limits
        """
        new_twist = self.twist

        # If there is need to reduce maximum allowed speeds in navigation node...
        max_vel_x = self.max_vel_x
        max_vel_y = self.max_vel_y
        max_vel_z = self.max_rot_vel

        # Apply smoother, based on accelerations and maximum velocities allowed
        dvx = new_twist.linear.x - self.last_twist.linear.x
        if dvx > self.max_acc/self.control_freq:
            new_twist.linear.x = self.last_twist.linear.x + self.max_acc/self.control_freq
        elif dvx < self.max_dacc/self.control_freq:
            new_twist.linear.x = self.last_twist.linear.x + self.max_dacc / self.control_freq
        new_twist.linear.x = max(min(new_twist.linear.x, max_vel_x), -max_vel_x)

        if self.kinematics == "omni":
            dvy = new_twist.linear.y - self.last_twist.linear.y
            if dvy > self.max_acc/self.control_freq:
                new_twist.linear.y = self.last_twist.linear.y + self.max_acc/self.control_freq
            elif dvy < self.max_dacc/self.control_freq:
                new_twist.linear.y = self.last_twist.linear.y + self.max_dacc / self.control_freq
            new_twist.linear.y = max(min(new_twist.linear.y, max_vel_y), -max_vel_y)

        err_z = new_twist.angular.z - self.last_twist.angular.z
        max_rot_acc = self.max_rot_acc/self.control_freq
        max_rot_dacc = self.max_rot_dacc/self.control_freq
        if self.last_twist.angular.z < 0:
            if err_z > max_rot_dacc:
                new_twist.angular.z = min(self.last_twist.angular.z + max_rot_dacc, 0)
            elif err_z < - max_rot_acc:
                new_twist.angular.z = max(self.last_twist.angular.z - max_rot_acc, -max_vel_z)
        elif self.last_twist.angular.z > 0:
            if err_z < - max_rot_dacc:
                new_twist.angular.z = max(self.last_twist.angular.z - max_rot_dacc, 0)
            elif err_z > max_rot_acc:
                new_twist.angular.z = min(self.last_twist.angular.z + max_rot_acc, max_vel_z)
        else:
            new_twist.angular.z = max(min(new_twist.angular.z, max_rot_acc), -max_rot_dacc)

        # print "Vx: " + str(new_twist.linear.x) + " | Vz: " + str(new_twist.angular.z)
        return new_twist

    def send_velocity(self):
        """
        Sends velocities saved as Twist to the motors
        Supports 2wd and 4omni
        :return:
        """
        # If the motorboard remote is active, let it control
        overwrite_vel = False
        if overwrite_vel:
            return
        else:
            if self.twist.linear.x == 0 and self.twist.linear.y == 0 and self.twist.angular.z == 0:
                new_twist = Twist()
            elif self.controller == "move_base" and (rospy.Time.now() - self.last_mb).to_sec() > 2:
                self.log("Lost connection to MoveBase", 7)
                new_twist = Twist()
            elif self.controller == "controller" and (rospy.Time.now() - self.last_ctrl).to_sec() > 2:
                self.log("Lost connection to Controller", 7)
                new_twist = Twist()
            else:
                new_twist = self.compute_velocity()

        if self.simulation:
            self.gazebo_pub.publish(new_twist)
        else:
            linear_vel_x = new_twist.linear.x
            linear_vel_y = new_twist.linear.y
            angular_vel = new_twist.angular.z

            if self.kinematics == "2wd":
                # Convert from robot velocity to wheel linear velocity
                # Right moves forward with positive velocity
                if linear_vel_x == 0. and angular_vel == 0.:
                    l_vel = 0
                    r_vel = 0
                else:
                    l_vel = -(linear_vel_x - angular_vel * self.base_width/2)
                    r_vel = (linear_vel_x + angular_vel * self.base_width/2)
                    # rospy.loginfo("L {} | R {}".format(l_vel, r_vel))

                msg = WheelsMB()
                msg.header.frame_id = "odom"
                msg.header.stamp = rospy.Time.now()
                msg.kinematics = self.kinematics
                msg.front_right = r_vel
                msg.front_left = l_vel
                self.vel_pub.publish(msg)

            elif self.kinematics == "omni":
                linear_vels = [0] * 4
                linear_vels[0] = (linear_vel_x - linear_vel_y - self.base_width * angular_vel)
                linear_vels[1] = (linear_vel_x + linear_vel_y + self.base_width * angular_vel)
                linear_vels[2] = (linear_vel_x + linear_vel_y - self.base_width * angular_vel)
                linear_vels[3] = (linear_vel_x - linear_vel_y + self.base_width * angular_vel)

                msg = WheelsMB()
                msg.header.frame_id = "odom"
                msg.header.stamp = rospy.Time.now()
                msg.kinematics = self.kinematics
                msg.front_right = linear_vels[0]
                msg.front_left = linear_vels[1]
                msg.back_right = linear_vels[2]
                msg.back_left = linear_vels[3]
                self.vel_pub.publish(msg)

        self.last_twist = new_twist

    def start(self):
        r = rospy.Rate(self.control_freq)
        while not rospy.is_shutdown():
            try:
                self.send_velocity()
                r.sleep()
            except KeyboardInterrupt:
                break
            except Exception as err:
                self.log("Exception in start(): " + str(err), 2, alert="error")

        rospy.logwarn(rospy.get_name()+": Shutting down")


if __name__ == "__main__":
    rospy.init_node("navigation_node")
    nav = IDMindNavigation()    
    nav.start()
