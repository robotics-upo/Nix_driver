#!/usr/bin/env python

import rospy
import actionlib
from nav_msgs.msg import Path
from idmind_robot.msg import Log
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger, TriggerResponse
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from idmind_controller.srv import FollowPath, FollowPathResponse, DoAction, DoActionResponse

VERBOSE = 5
LOGS = 5

movebase_status = ["pending", "active", "preempted", "succeeded",
                   "aborted", "rejected", "preempting", "recalling", "recalled", "lost"]


class IDMindNavInterface:
    """
    Class responsible for managing the navigation of the robot through move_base or idmind controller.
    """

    def __init__(self, verbose=True):

        self.verbose = verbose
        self.kinematics = rospy.get_param("/bot/kinematics", default="2wd")
        self.controller = rospy.get_param("/controller", default="controller")
        self.control_freq = rospy.get_param("/move_base/controller_frequency", default=10.)

        # Services required
        self.active = False
        self.nav_feedback = MoveBaseActionFeedback()

        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

        self.idmind_client = actionlib.SimpleActionClient('idmind_controller', MoveBaseAction)
        self.idmind_client.wait_for_server()

        # Services provided
        self.new_path = False
        self.path = Path()
        self.current_goal_pose = PoseStamped()
        rospy.Service("/idmind_controller/follow_path", FollowPath, self.handle_new_path)
        self.action = ""
        rospy.Service("/idmind_controller/action", DoAction, self.handle_new_action)
        rospy.Service("/idmind_controller/cancel_navigation", Trigger, self.cancel_navigation)

        # Topic Subscribers
        rospy.Subscriber("/idmind_controller/rviz_goal", PoseStamped, self.handle_rviz_goal)

        # Topic Publishers
        self.logging = rospy.Publisher("/idmind_logging", Log, queue_size=10)

        # Logs
        self.log("Node Initialized", 5)

    #################
    #   Auxiliary   #
    #################
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

    ###########################
    #  Callbacks and updates  #
    ###########################
    def handle_new_path(self, req):
        """
        Service callback for \idmind_controller\follow_path
        :param req:
        :return:
        """
        self.path = req.path
        self.new_path = True
        return FollowPathResponse(success=True, message="Path accepted")

    def handle_new_action(self, req):
        """
        Service callback for \idmind_controller\action
        :param req:
        :return:
        """
        self.action = req.action
        return DoActionResponse(success=True, message="Action Completed")

    def nav_done_callback(self, ts, r):
        """
        Action client callback for when the action server sets the goal as done (sucessful or not)
        :param ts:
        :param r:
        :return:
        """
        self.log("Goal is {}".format(movebase_status[ts]), 3)
        self.active = False
        try:
            self.log(r, 3)  # It might be empty
        except:
            pass

        if ts == 4:  # Objective Failed
            self.log("Failed to reach goal. Waiting 3 secs and retrying", 3)
            self.path.poses.insert(0, self.current_goal_pose)
            rospy.sleep(3)
        elif ts == 3:
            self.log("Goal reached", 3)
            self.current_goal_pose = PoseStamped()

    def nav_active_callback(self):
        """
        Action client callback for when the action server accepts the goal
        :return:
        """
        self.active = True
        rospy.loginfo("Active Goal")

    def nav_feedback_callback(self, feedback):
        """
        Callback for messages published by the action server as feedback
        :param feedback:
        :return:
        """
        self.nav_feedback = feedback

    def handle_rviz_goal(self, msg):
        """
        Callback for a new 2D Navigation goal published in rviz
        :param msg:
        :return:
        """
        self.path = Path()
        self.path.poses.append(msg)
        self.new_path = True

    def cancel_navigation(self, _req):
        """
        Service provided by navigation_interface to cancel all current goals
        :param _req:
        :return:
        """
        self.path.poses = []
        if self.controller == "move_base":
            self.move_base_client.cancel_all_goals()
        elif self.controller == "controler":
            self.idmind_client.cancel_all_goals()
            self.idmind_client.cancel_goal()
        self.active = False
        return TriggerResponse(success=True, message="Navigation cancelled")

    #################
    #    Actions    #
    #################

    ##################
    #   Controller   #
    ##################
    def send_new_goal(self):
        """
        Sends a new goal for the action server (move_base or idmind_controller)
        :return:
        """
        try:
            goal = MoveBaseGoal()
            goal.target_pose = self.current_goal_pose
            if self.controller == "move_base":
                self.log("Goal sent to move_base", 5)
                self.move_base_client.send_goal(goal, done_cb=self.nav_done_callback,
                                                active_cb=self.nav_active_callback,
                                                feedback_cb=self.nav_feedback_callback)
            elif self.controller == "controller":
                self.log("Goal sent to idmind_controller", 5)
                self.idmind_client.send_goal(goal, done_cb=self.nav_done_callback,
                                             active_cb=self.nav_active_callback,
                                             feedback_cb=self.nav_feedback_callback)

        except Exception as mb_err:
            self.log("Exception sending goal: {}".format(mb_err), 2, alert="err")

    def start(self):
        r = rospy.Rate(self.control_freq)

        while not rospy.is_shutdown():
            try:
                if self.new_path or (self.path.poses and not self.active):
                    self.log("Handling next goal", 3)
                    self.new_path = False
                    self.current_goal_pose = self.path.poses.pop(0)
                    self.send_new_goal()
                r.sleep()
            except KeyboardInterrupt:
                break
            except Exception as err:
                self.log("Exception in start(): {}".format(err), 2, alert="error")

        self.log("Node Shutting down", 3, alert="warn")


if __name__ == "__main__":
    rospy.init_node("controller_interface")
    interface = IDMindNavInterface()
    interface.start()
