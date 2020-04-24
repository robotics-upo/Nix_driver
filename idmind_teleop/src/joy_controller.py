#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerRequest
from idmind_motorsboard.msg import WheelsMB #Added by Fali

# Using D mode
# buttons = {"X": 0, "A": 1, "B": 2, "Y": 3, "LB": 4, "RB": 5, "LT": 6, "RT": 7, "BACK": 8,
#            "START": 9, "LJOY": 10, "RJOY": 11}
# axis = {"XArrows": 0, "YArrows": 1, "XRJoy": 2, "YRJoy": 3, "XLJoy": 4, "YLJoy": 5}

# Using X, Mode Off
buttons = {"A": 0, "B": 1, "X": 2, "Y": 3, "LB": 4, "RB": 5, "BACK": 6, "START": 7, "LOGI": 8, "LJOY": 9, "RJOY": 10}
axis = {"XLJoy": 0, "YLJoy": 1, "LT": 2, "XRJoy": 3, "YRJoy": 4, "RT": 5, "XArrows": 6, "YArrows": 7}

SET_ARM_MAX = 690
SET_ARM_MIN = 444
ARM_INCR = 15

class Teleop:
    """
    Class responsible for interpreting joystick commands
    """
    def __init__(self):
        
        # Robot parameters - load from idmind_robot or from idmind_motorsboard
        self.control_freq = 20
        self.kinematics = rospy.get_param("/bot/kinematics", default="2wd")
        if rospy.has_param("/bot/max_vel"):
            self.max_vel = rospy.get_param("/bot/max_vel")
            self.max_rot = rospy.get_param("/bot/max_rot")
        else:
            self.max_vel = rospy.get_param("/motors/max_vel", default=0.3)
            self.max_rot = rospy.get_param("/bot/max_rot", default=0.7)
        
        self.enable_auto_switch_light = rospy.ServiceProxy('/lights_switcher/switch_auto_lights', Trigger)


        ################
        #  Navigation  #
        ################
        self.twist = Twist()
        self.twist_pub = rospy.Publisher("/cmd_vel_joy", Twist, queue_size=10)
        #try:
        #    rospy.wait_for_service("/idmind_navigation/toggle_joystick", timeout=10)
        #    self.toggle_joy = rospy.ServiceProxy("/idmind_navigation/toggle_joystick", Trigger)
        #except rospy.ROSException:
        #    rospy.logwarn("/idmind_navigation did not respond, assuming direct link to motorsboard")
        self.last_time = rospy.Time.now()
        self.republish = True
        self.joy_received = False
        self.light_srv = rospy.ServiceProxy('/idmind_sensors/switch_lights', Trigger)

        ####################
        #  Robot Features  #
        ####################
        # RAPOSA ARM
        self.arm_position = 512
        self.arm_goal = 512
        rospy.Subscriber("/idmind_motors/arm", Int32, self.handle_arm)
        self.arm_pub = rospy.Publisher("/idmind_motors/set_arm", Int32, queue_size=10)

        # Subscribe should be last, in order to set all required variables
        rospy.Subscriber("/joy", Joy, self.update_joy)

        rospy.loginfo("{} is initialized".format(rospy.get_name()))
    
    def update_joy(self, msg):

        self.last_time = rospy.Time.now()
        self.joy_received = True
        new_vel = Twist()
        new_vel.linear.x = msg.axes[1] * self.max_vel
        
        if self.kinematics == "2wd":
            new_vel.angular.z = msg.axes[0] * self.max_rot
        elif self.kinematics == "omni":
            new_vel.linear.y = msg.axes[3] * self.max_vel
            new_vel.angular.z = msg.axes[0] * self.max_rot
        self.twist = new_vel
    
        self.arm_goal = self.arm_position - 10 * msg.axes[5]
        if msg.buttons[4] and self.arm_goal < SET_ARM_MAX:
            self.arm_goal +=ARM_INCR

        if msg.buttons[2]:
            rospy.wait_for_service('/lights_switcher/switch_auto_lights')  
            try:
                req = TriggerRequest()
                resp1 = self.enable_auto_switch_light(req)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e


        if msg.buttons[7]:
            rospy.wait_for_service('/idmind_sensors/switch_lights')  
            try:
                req = TriggerRequest()
                resp1 = self.light_srv(req)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

        if msg.buttons[5] and self.arm_goal > SET_ARM_MIN:
            self.arm_goal -=ARM_INCR

            #if msg.buttons[9] == 1:
            #    self.toggle_joy(TriggerRequest()) # Esto daba el error que no existia toggle joy
        if msg.buttons[6]:
            if self.republish:
                self.republish = False
                rospy.logwarn("republish to false")
            else:
                self.republish = True
                rospy.logwarn("republish to true")

    def handle_arm(self, msg):
        self.arm_position = msg.data

    def start(self):
        r = rospy.Rate(self.control_freq)
        while not rospy.is_shutdown():
            try:
                if self.republish:
                    self.twist_pub.publish(self.twist)
                    self.arm_pub.publish(self.arm_goal)
                    r.sleep()
            except KeyboardInterrupt:
                break

        rospy.logwarn("{} is shutting down".format(rospy.get_name()))


if __name__ == "__main__":
    rospy.init_node("idmind_teleop")
    t = Teleop()
    t.start()
