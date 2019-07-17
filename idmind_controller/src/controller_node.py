#!/usr/bin/env python

import roslib
import rospy
import actionlib
from threading import Lock
from numpy.linalg import norm
from idmind_robot.msg import Log
from numpy import arctan2, pi, sign
from nav_msgs.srv import GetPlan, GetPlanRequest
from tf_conversions import transformations as trf
from idmind_controller.msg import CloserObstaclesMsg
from idmind_controller.srv import Obstacles, ObstaclesRequest
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from move_base_msgs.msg import MoveBaseAction, MoveBaseFeedback, MoveBaseResult
from tf2_geometry_msgs import PoseStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, LookupException, ConnectivityException, \
    ExtrapolationException, InvalidArgumentException

roslib.load_manifest('idmind_controller')
VERBOSE = 7
LOGS = 7


class IDMindController:
    """
    Class responsible for managing the navigation of the robot, using PID or DWA controller.
    The actionlib goal is registered or cancelled by the navigation_interface
    TODO: Implement DWA controllers
    """

    def __init__(self):
        self.control_freq = rospy.get_param("/move_base/controller_frequency", default=10.)
        self.logging = rospy.Publisher("/idmind_logging", Log, queue_size=10)

        ###################
        #  Action Server  #
        ###################
        self.action_server = actionlib.SimpleActionServer("idmind_controller", MoveBaseAction,
                                                          execute_cb=self.start, auto_start=False)

        #################
        #  TF Listener  #
        #################
        self.tf_buffer = Buffer()
        self.transform_listener = TransformListener(self.tf_buffer)
        self.odom_broadcast = TransformBroadcaster()

        ###############
        #  Auxiliary  #
        ###############
        self.log("Loading footprint", 7)
        self.footprint = []
        self.load_footprint()
        # Compute edges of robot - front, right, back, left
        self.edges = [0, 0, 0, 0]
        for p in self.footprint:
            self.edges[0] = p[0] if p[0] > self.edges[0] else self.edges[0]
            self.edges[2] = p[0] if p[0] < self.edges[2] else self.edges[2]
            self.edges[1] = p[1] if p[1] < self.edges[1] else self.edges[1]
            self.edges[3] = p[1] if p[1] > self.edges[3] else self.edges[3]

        #########
        #  ROS  #
        #########
        # Navigation parameters
        self.kinematics = rospy.get_param("/bot/kinematics", "2wd")
        self.simulation = rospy.get_param("/simulation", False)
        self.max_linear_vel = rospy.get_param("/move_base/DWAPlannerROS/max_vel_x", 0.7)
        self.max_rot_vel = rospy.get_param("/move_base/DWAPlannerROS/max_rot_vel", 0.7)
        self.max_linear_acc = rospy.get_param("/move_base/DWAPlannerROS/acc_lim_x", 0.7)
        self.max_rot_acc = rospy.get_param("/move_base/DWAPlannerROS/acc_lim_theta", 0.7)
        self.max_rot_dacc = 2 * self.max_rot_acc
        self.min_linear_vel = rospy.get_param("/move_base/DWAPlannerROS/min_trans_vel", 0.2)
        self.min_rot_vel = rospy.get_param("/move_base/DWAPlannerROS/min_rot_vel", 0.4)

        self.clearance_dist = rospy.get_param("/move_base/DWAPlannerROS/forward_point_distance", 0.35)
        self.xy_tolerance = rospy.get_param("/move_base/DWAPlannerROS/xy_goal_tolerance", 0.1)
        self.yaw_tolerance = rospy.get_param("/move_base/DWAPlannerROS/yaw_goal_tolerance", 10*pi/180)

        # Topic Publishers
        self.checkpoint_pub = rospy.Publisher("/idmind_controller/checkpoint", PoseStamped, queue_size=10)
        self.vel_pub = rospy.Publisher("/cmd_vel_idmind", Twist, queue_size=10)

        # Topic Subscribers
        self.close_obstacles = False
        self.obstacles = {"front": 10., "back": 10., "left": 10., "right": 10.}
        rospy.Subscriber("/idmind_controller/closest_obstacles", CloserObstaclesMsg, self.handle_lasers)

        # Services required
        self.plan_lock = Lock()
        self.new_plan = False
        self.plan = []
        while not rospy.is_shutdown():
            try:
                rospy.wait_for_service("/move_base/NavfnROS/make_plan", 1)
                break
            except rospy.ROSException:
                self.log("Move_Base not available. Waiting 5 seconds.", 5)
                rospy.sleep(5)
        self.move_base_plan = rospy.ServiceProxy("/move_base/NavfnROS/make_plan", GetPlan)

        while not rospy.is_shutdown():
            try:
                rospy.wait_for_service("/idmind_recovery/obstacle_handler", 1)
                break
            except rospy.ROSException:
                self.log("idmind_recovery service not available. Waiting 5 seconds.", 5)
                rospy.sleep(5)
        self.call_obstacle_handler = rospy.ServiceProxy("/idmind_recovery/obstacle_handler", Trigger)
        self.request_lasers = rospy.ServiceProxy("/idmind_controller/list_obstacles", Obstacles)

        # Services provided
        rospy.Service("/idmind_controller/ready", Trigger, self.report_ready)

        # Replanning Timer
        self.replan_timer = rospy.Timer(rospy.Duration(5), self.replan_callback)
        self.action_server.start()
        self.ready = True
        self.log("Node Initialized", 5)

    #################################
    #  Callbacks and other updates  #
    #################################
    def report_ready(self, _req):
        """ Simple Service callback to show node is ready """
        return TriggerResponse(self.ready, "IDMind Controller is " + ("ready" if self.ready else "not ready"))

    def handle_lasers(self, msg):
        """
        Callback for /idmind_controller/closest_obstacles"
        :param msg:
        :return:
        """
        self.obstacles["front"] = msg.front
        self.obstacles["back"] = msg.back
        self.obstacles["left"] = msg.left
        self.obstacles["right"] = msg.right
        if any(d < 0.1 for d in self.obstacles.itervalues()):
            self.close_obstacles = True
        else:
            self.close_obstacles = False

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

    def get_pose(self, frame_id, origin_frame="map"):
        """
        Returns pose of a frame in the global map
        :param frame_id:
        :param origin_frame:
        :return PoseStamped pose:
        """
        try:
            transf = self.tf_buffer.lookup_transform(origin_frame, frame_id, rospy.Time(0))
        except (LookupException, ConnectivityException, ExtrapolationException, InvalidArgumentException) as err:
            self.log("get_pose() exception: {}".format(err), 3)
            return PoseStamped()

        pose = PoseStamped()
        pose.header = transf.header
        pose.header.frame_id = origin_frame
        pose.pose.position = transf.transform.translation
        pose.pose.orientation = transf.transform.rotation
        return pose

    def replan_callback(self, _ev):
        if self.action_server.is_active():
            current_pose = self.get_pose("base_link", "map")
            goal = self.action_server.current_goal.get_goal().target_pose
            self.update_plan(current_pose, goal)

    def load_footprint(self):
        """
        This method will search for the footprint in ros params:
            1. /move_base/local_costmap/footprint
            2. /move_base/global_costmap/footprint
            3. /move_base/local_costmap/robot_radius
            4. Use default value of 0.5m
        Stores it under self.footprint
        :return:
        """

        try:
            # Search for footprint in params.
            f = ""
            for idx in range(0, 5):
                if rospy.has_param("/move_base/local_costmap/footprint"):
                    f = rospy.get_param("/move_base/local_costmap/footprint")[2:-2].split("],[")
                    break
                elif rospy.has_param("/move_base/global_costmap/footprint"):
                    f = rospy.get_param("/move_base/global_costmap/footprint")[2:-2].split("],[")
                    break
                elif rospy.has_param("/move_base/local_costmap/robot_radius"):
                    rad = rospy.get_param("/move_base/local_costmap/robot_radius")
                    f = [[rad, rad], [rad, -rad], [-rad, -rad], [-rad, rad]]
                    break
                rospy.sleep(1)

            # Transform the string in list or use default
            if len(f) > 0:
                fp = []
                for val in f:
                    a = val.split(',')
                    fp.append([float(a[0]), float(a[1])])
            else:
                rad = 0.5
                fp = [[rad, rad], [rad, -rad], [-rad, -rad], [-rad, rad]]
            self.footprint = list(fp)
            return
        except AttributeError:
            # In some cases, footprint comes directly as a list instead of string
            fp = rospy.get_param("/move_base/local_costmap/footprint")

        self.footprint = list(fp)

    ########################
    #  Navigation methods  #
    ########################
    def update_plan(self, current_pose, goal):
        """
        This function receives a PoseStamped goal and return a Path plan.
        The planner by default is move_base.
        TODO: implement a new planner?
        :param current_pose:
        :param goal:
        :return plan:
        """
        self.log("Request a plan from navfn", 5)
        # start and goal for the request are PoseStamped
        req = GetPlanRequest()
        # Start point
        req.start.header.frame_id = "map"  # Just to make sure...
        req.start.pose.position.x = current_pose.pose.position.x
        req.start.pose.position.y = current_pose.pose.position.y
        req.start.pose.orientation.x = current_pose.pose.orientation.x
        req.start.pose.orientation.y = current_pose.pose.orientation.y
        req.start.pose.orientation.z = current_pose.pose.orientation.z
        req.start.pose.orientation.w = current_pose.pose.orientation.w
        # Goal
        req.goal.header.frame_id = "map"
        req.goal.pose.position.x = goal.pose.position.x
        req.goal.pose.position.y = goal.pose.position.y
        req.goal.pose.orientation.x = goal.pose.orientation.x
        req.goal.pose.orientation.y = goal.pose.orientation.y
        req.goal.pose.orientation.z = goal.pose.orientation.z
        req.goal.pose.orientation.w = goal.pose.orientation.w
        # Tolerance
        req.tolerance = self.xy_tolerance
        temp_plan = self.move_base_plan(req).plan.poses
        for idx in range(0, len(temp_plan)-1):
            curr_pose = temp_plan[idx].pose.position
            nxt_pose = temp_plan[idx+1].pose.position
            yaw = arctan2(nxt_pose.y - curr_pose.y, nxt_pose.x - curr_pose.x)
            q = trf.quaternion_from_euler(0, 0, yaw)
            temp_plan[idx].pose.orientation = Quaternion(*q)
        self.plan_lock.acquire()
        self.plan = list(temp_plan)
        self.new_plan = True
        self.plan_lock.release()

    @staticmethod
    def clear_checkpoint(dx, dy, last_vel):
        """
        Checks if a checkpoint is close enough to be cleared or not.
        Step 1: check only distance from current pose
        TODO: check distance from estimated next pose (take velocity in account)
        :param dx:
        :param dy:
        :param last_vel:
        :return True/False:
        """
        v = [last_vel.linear.x, last_vel.linear.y]
        clear_d = 0.25  # if any(d < 0.2 for d in self.obstacles.itervalues()) else 2*self.clearance_dist
        return False if norm([dx, dy]) > min(clear_d + 0.9 * norm([v]), 1.2) else True

    def handle_obstacles(self):
        """
        This method should promote obstacle avoidance and issue a replanning after reaching safety.
        :return:
        """
        # Initiate variables and stop the robot
        self.log("Handling Obstacles", 5)
        self.vel_pub.publish(Twist())
        rospy.sleep(rospy.Duration(1))
        self.replan_timer.shutdown()

        res = False
        while not res:
            res = self.call_obstacle_handler(TriggerRequest()).success
            if not res:
                self.log("Obstacle handler failed. Waiting 5 secs and retrying.", 3, alert="warn")
                rospy.sleep(5)

        current_pose = self.get_pose("base_link", "map")
        goal = self.action_server.current_goal.get_goal().target_pose
        self.update_plan(current_pose, goal)
        self.replan_timer = rospy.Timer(rospy.Duration(5), self.replan_callback)
        self.log("Obstacles avoided", 5)
        return True

    def handle_obstacles_simple(self):
        """
        This method should promote obstacle avoidance and issue a replanning after reaching safety.
        TODO: improve handle lateral obstacles
        :return:
        """
        r = rospy.Rate(self.control_freq)
        self.log("Handling Obstacles", 5)
        last_vel = Twist()
        while not rospy.is_shutdown() and any(d < 0.2 for d in self.obstacles.itervalues()):
            if self.new_plan:
                break
            new_vel = Twist()
            # Forces caused by obstacles - max() is to handle collisions
            fxb = .2/max(0.01, self.obstacles["back"]) if self.obstacles["back"] < 0.2 else 0
            fxf = .2/max(0.01, self.obstacles["front"]) if self.obstacles["front"] < 0.2 else 0
            fyr = .2/max(0.01, self.obstacles["right"]) if self.obstacles["right"] < 0.2 else 0
            fyl = .2/max(0.01, self.obstacles["left"]) if self.obstacles["left"] < 0.2 else 0

            if fxb > 0 and fxf > 0:
                new_vel.angular.z = fxb + fxf
            else:
                new_vel.linear.x = fxb - fxf

            if self.kinematics == "2wd":
                new_vel.linear.y = 0.
                new_vel.angular.z = new_vel.angular.z + fyr - fyl
            else:
                new_vel.linear.y = fyr - fyl

            new_vel = self.limit_velocities(new_vel, last_vel, latch_goal=False)
            self.vel_pub.publish(new_vel)
            last_vel = new_vel
            r.sleep()

        for i in range(0, 5):
            self.vel_pub.publish(Twist())
            r.sleep()
        current_pose = self.get_pose("base_link", "map")
        goal = self.action_server.current_goal.get_goal().target_pose
        self.update_plan(current_pose, goal)
        self.log("Obstacles avoided", 5)

    def pid_controller(self, err_x, err_y, err_th, latch_goal):
        """
        Implementation of a PID controller for trajectory following.
        Special cases for |err_th| larger than pi/4 (only turns)
        TODO: Special case when goal is short distance behind robot
        :param err_x:
        :param err_y:
        :param err_th:
        :param latch_goal:
        :return:
        """

        kp = 2.5
        ki = 1.0
        kd = 0.7
        new_vel = Twist()

        self.log("EX: {} | EY: {} | ETH: {}".format(err_x[-1], err_y[-1], err_th[-1]), 8)
        # Angular velocity
        vel_th_p = kp * err_th[-1]
        vel_th_i = ki * sum(err_th) / (len(err_th) * self.control_freq)
        vel_th_d = kd * self.control_freq * (err_th[-2] - err_th[-1]) if len(err_th) >= 2 else 0
        new_vel.angular.z = vel_th_p + vel_th_i + vel_th_d

        # If the heading error is too large, focus on rotation (is it necessary?)
        if abs(err_th[-1]) > pi/4:
            self.log("Orienting to path", 8)
            return new_vel
        # If the goal is latched, the robot rotates in place
        if latch_goal:
            new_vel.linear.x = 0
            new_vel.linear.y = 0
            return new_vel

        # p_gain * pos_error - The abs() is a simple solution for when dx < 0
        vel_x_p = kp * abs(err_x[-1])
        # Derivative gain (difference * freq)
        vel_x_d = kd * self.control_freq * (err_x[-2] - err_x[-1]) if len(err_x) >= 2 else 0
        # Integral gain (mean of error / freq)
        vel_x_i = ki * sum(err_x) / (len(err_x) * self.control_freq)
        new_vel.linear.x = vel_x_p + vel_x_i + vel_x_d

        if self.kinematics == "2wd":
            new_vel.linear.y = 0
        elif self.kinematics == "omni":
            # p_gain * pos_error
            vel_y_p = kp * abs(err_y[-1])
            # Derivative gain (difference * freq)
            vel_y_d = kd * self.control_freq * (err_y[-2] - err_y[-1]) if len(err_y) >= 2 else 0
            # Integral gain (mean of error / freq)
            vel_y_i = ki * sum(err_y) / (len(err_y) * self.control_freq)
            new_vel.linear.y = vel_y_p + vel_y_i + vel_y_d

        self.log("VX: {} | VY: {} | VTH: {}".format(new_vel.linear.x, new_vel.linear.y, new_vel.angular.z), 8)
        return new_vel

    @staticmethod
    def dwa_controller():
        new_vel = Twist()
        return new_vel

    def limit_velocities(self, new_vel, last_vel, latch_goal):

        # Start by applying smoother, based on maximum accelerations
        dvx = new_vel.linear.x - last_vel.linear.x
        aux_vel = new_vel.linear.x
        self.log("DVX: {}".format(dvx), 8)
        if dvx > self.max_linear_acc / self.control_freq:
            new_vel.linear.x = last_vel.linear.x + self.max_linear_acc / self.control_freq
            self.log("X velocity {} reduced to {}".format(aux_vel, new_vel.linear.x), 8)
        if dvx < - self.max_linear_acc / self.control_freq:
            new_vel.linear.x = last_vel.linear.x - self.max_linear_acc / self.control_freq
            self.log("X velocity {} increased to {}".format(aux_vel, new_vel.linear.x), 8)
        if self.kinematics == "omni":
            dvy = last_vel.linear.y - new_vel.linear.y
            if dvy > self.max_linear_acc / self.control_freq:
                new_vel.linear.y = last_vel.linear.y + self.max_linear_acc / self.control_freq
                self.log("Y velocity reduced to {}".format(new_vel.linear.x), 8)
            if dvy < - self.max_linear_acc / self.control_freq:
                new_vel.linear.y = last_vel.linear.y - self.max_linear_acc / self.control_freq
                self.log("Y velocity increased to {}".format(new_vel.linear.x), 8)

        err_z = new_vel.angular.z - last_vel.angular.z
        if last_vel.angular.z < 0:
            if err_z > self.max_rot_dacc/self.control_freq:
                new_vel.angular.z = min(last_vel.angular.z + self.max_rot_dacc/self.control_freq, 0)
            elif err_z < - self.max_rot_acc/self.control_freq:
                new_vel.angular.z = max(last_vel.angular.z - self.max_rot_acc/self.control_freq, -self.max_rot_vel)
        elif last_vel.angular.z > 0:
            if err_z < - self.max_rot_dacc/self.control_freq:
                new_vel.angular.z = max(last_vel.angular.z - self.max_rot_dacc/self.control_freq, 0)
            elif err_z > self.max_rot_acc/self.control_freq:
                new_vel.angular.z = min(last_vel.angular.z + self.max_rot_acc/self.control_freq, self.max_rot_vel)

        max_l_vel = self.max_linear_vel/2 if any(d < 0.3 for d in self.obstacles.itervalues()) else self.max_linear_vel
        max_r_vel = self.max_rot_vel/2 if any(d < 0.3 for d in self.obstacles.itervalues()) else self.max_rot_vel

        # Apply limits, based on maximum velocities
        new_vel.linear.x = max(min(new_vel.linear.x, max_l_vel), -max_l_vel)
        new_vel.linear.y = max(min(new_vel.linear.y, max_l_vel), -max_l_vel)
        new_vel.angular.z = max(min(new_vel.angular.z, max_r_vel), -max_r_vel)

        # Apply limits, based on minimum velocities (in simulation, navigation is heavilly affected)
        if latch_goal:
            new_vel.linear.x = 0
            new_vel.linear.y = 0
        else:
            if abs(new_vel.angular.z) < self.min_rot_vel:
                if new_vel.linear.y == 0:
                    new_vel.linear.x = sign(new_vel.linear.x) * self.min_linear_vel if self.min_linear_vel > abs(
                        new_vel.linear.x) > 0.01 else new_vel.linear.x
                if new_vel.linear.x == 0:
                    new_vel.linear.y = sign(new_vel.linear.y) * self.min_linear_vel if self.min_linear_vel > abs(
                        new_vel.linear.y) > 0.01 else new_vel.linear.y

        if new_vel.linear.x == 0 and new_vel.linear.y == 0:
            vz = new_vel.angular.z
            min_rot = self.min_rot_vel if not self.simulation else 0.01
            new_vel.angular.z = sign(vz) * min_rot if min_rot > abs(vz) > 0 else vz

        self.log("VXC: {} | VYC: {} | VTHC: {}".format(new_vel.linear.x, new_vel.linear.y, new_vel.angular.z), 8)
        return new_vel

    @staticmethod
    def update_history(current_pose, checkpoint, err_x, err_y, err_th, latch_goal=False):
        err_x.append(checkpoint.pose.position.x - current_pose.pose.position.x)
        err_x = err_x[-10:] if len(err_x) > 10 else err_x
        err_y.append(checkpoint.pose.position.y - current_pose.pose.position.y)
        err_y = err_y[-10:] if len(err_y) > 10 else err_y
        thi = trf.euler_from_quaternion([current_pose.pose.orientation.x, current_pose.pose.orientation.y,
                                         current_pose.pose.orientation.z, current_pose.pose.orientation.w])
        thf = trf.euler_from_quaternion([checkpoint.pose.orientation.x, checkpoint.pose.orientation.y,
                                         checkpoint.pose.orientation.z, checkpoint.pose.orientation.w])
        # dth can be in relation to next point (latch_goal is false) or to last pose (latch_goal is true)
        dth = thf[2] - thi[2] if latch_goal else arctan2(err_y[-1], err_x[-1]) - thi[2]
        dth = dth - sign(dth) * 2 * pi if abs(dth) > pi else dth
        err_th.append(dth)
        err_th = err_th[-10:] if len(err_th) > 10 else err_th

        return [err_x, err_y, err_th]

    def obstacle_avoidance(self, checkpoint, safe_distance=0.2):
        """
        This function will check the surroundings of the checkpoint and move it if close to obstacles.
        Special cases: corridors and doors (obstacles in both sides but not in front)
        Returns the new checkpoint
        :param checkpoint:
        :param safe_distance:
        :return checkpoint:
        """

        # Check for obstacles in the vicinity. If so, call handle_obstacles()
        if self.close_obstacles:
            self.log("Obstacle handler called", 5)
            self.handle_obstacles()
            return checkpoint
        # Else, check if the checkpoint has obstacles around and apply forces
        else:
            # Publish checkpoint as a frame (to simply computation for obstacle limits)
            transf_msg = TransformStamped()
            transf_msg.header.stamp = rospy.Time.now()
            transf_msg.header.frame_id = "map"
            transf_msg.child_frame_id = "base_link_next"

            transf_msg.transform.translation.x = checkpoint.pose.position.x
            transf_msg.transform.translation.y = checkpoint.pose.position.y
            q = checkpoint.pose.orientation
            transf_msg.transform.rotation = Quaternion(x=q.x, y=q.y, z=q.z, w=q.w)
            self.odom_broadcast.sendTransform(transf_msg)

            # Requests lasers - [xi, yi] in base_link frame, transform to next
            lasers = self.request_lasers(ObstaclesRequest()).obstacles
            left_obs = 1e6
            right_obs = -1e6
            for idx in range(0, len(lasers)/2):
                if norm([lasers[2*idx], lasers[2*idx+1]]) < 1.5:
                    obs_pose = PoseStamped()
                    obs_pose.header.frame_id = "base_link"
                    obs_pose.pose.position.x = lasers[2 * idx]
                    obs_pose.pose.position.y = lasers[2 * idx + 1]
                    obs_pose.pose.orientation.w = 1.
                    new_obs_pose = self.tf_buffer.transform(obs_pose, "base_link_next", timeout=rospy.Duration(1))

                    if self.edges[0] + safe_distance > new_obs_pose.pose.position.x > self.edges[2] - safe_distance:
                        d = new_obs_pose.pose.position.y
                        if d > 0.:  # Obstacle at the left
                            left_obs = d - self.edges[3] if d - self.edges[3] < left_obs else left_obs
                        else:
                            right_obs = d - self.edges[1] if d - self.edges[1] > right_obs else right_obs

            # self.log("Left: {} | Right: {}".format(round(left_obs, 3), round(right_obs, 3)), 5)

            if left_obs < 0.3 and right_obs > -0.3:
                self.log("Corridor", 5)
                new_checkpoint = PoseStamped()
                new_checkpoint.header.frame_id = "base_link_next"
                new_checkpoint.pose.position.y = (left_obs + right_obs)/2
                new_checkpoint.pose.orientation.w = 1
                checkpoint = self.tf_buffer.transform(new_checkpoint, "map", timeout=rospy.Duration(1))
            elif left_obs < 0.2:
                self.log("Obstacle on the left", 5)
            elif right_obs > -0.2:
                self.log("Obstacle on the right", 5)

            # Transform the adapted checkpoint to map frame
            return checkpoint

    ##################
    #   Controller   #
    ##################
    def start(self, mb_goal):
        """
        Action server callback
        :param mb_goal: MoveBaseGoal
        :return:
        """
        self.log("Accepted new goal", 5)
        result = MoveBaseResult()   # This is an empty message
        r = rospy.Rate(self.control_freq)
        # Get plan from current pose to goal
        cur_pose = self.get_pose("base_link", "map")
        # goal = mb_goal.goal.target_pose
        goal = mb_goal.target_pose
        self.update_plan(cur_pose, goal)

        self.log("Starting the navigation", 5)
        last_vel = Twist()
        latch_goal = False
        success = False
        cancel = False
        err_x = []
        err_y = []
        err_th = []

        while len(self.plan) > 0:
            try:
                if cancel:
                    break

                # Load new checkpoint from plan
                checkpoint = self.plan.pop(0)
                checkpoint_reached = False
                self.new_plan = False
                self.checkpoint_pub.publish(checkpoint)

                # Stay in loop until reaching checkpoint or a new plan is issued
                while not rospy.is_shutdown() and not checkpoint_reached and not self.new_plan:
                    # Update current_pose (and other variables if needed) and post feedback
                    cur_pose = self.get_pose("base_link", "map")
                    self.action_server.publish_feedback(MoveBaseFeedback(base_position=cur_pose))

                    # Check if goal has been canceled by user
                    if self.action_server.is_preempt_requested():
                        self.log("Received request to cancel current goal", 7)
                        cancel = True
                        success = False
                        break

                    checkpoint = self.obstacle_avoidance(checkpoint)

                    # Compute error in pose and heading
                    [err_x, err_y, err_th] = self.update_history(cur_pose, checkpoint, err_x, err_y, err_th, latch_goal)

                    # If we have arrived close enough to the goal, latch the robot (stop translation)
                    if len(self.plan) == 0 and norm([err_x[-1], err_y[-1]]) < self.xy_tolerance and not latch_goal:
                        self.log("Goal latched", 5)
                        latch_goal = True
                        continue

                    # If we have already latched to goal and are within yaw_tolerance, signal as over
                    if len(self.plan) == 0 and latch_goal and abs(err_th[-1]) < self.yaw_tolerance:
                        self.log("Goal reached", 5)
                        success = True
                        checkpoint_reached = True
                        self.vel_pub.publish(Twist())
                        last_vel = Twist()
                        continue

                    # If we are going through checkpoints, check if we can clear the checkpoint
                    # Due to the high density of points, several need to be cleared at each iteration
                    while len(self.plan) > 0 and self.clear_checkpoint(err_x[-1], err_y[-1], last_vel):
                        checkpoint = self.plan.pop(0)
                        self.new_plan = False
                        self.checkpoint_pub.publish(checkpoint)
                        [err_x, err_y, err_th] = self.update_history(cur_pose, checkpoint, err_x, err_y, err_th,
                                                                     latch_goal)

                    new_vel = self.pid_controller(err_x, err_y, err_th, latch_goal)
                    new_vel = self.limit_velocities(new_vel, last_vel, latch_goal)

                    self.vel_pub.publish(new_vel)
                    last_vel = new_vel
                    r.sleep()
            except (KeyboardInterrupt, rospy.ROSInterruptException, rospy.ROSException) as err:
                success = False
                rospy.logwarn(rospy.get_name() + ": Shutting down in start(): {}".format(err))
                self.vel_pub.publish(Twist())
                self.plan = []
                break
            r.sleep()

        if success:
            rospy.logerr("Goal set as succeeded")
            self.action_server.set_succeeded(result=result)
        else:
            rospy.logerr("Goal set as aborted")
            self.action_server.set_aborted(result=result)


if __name__ == "__main__":
    rospy.init_node("idmind_controller")
    controller = IDMindController()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logwarn(rospy.get_name() + ": Shutting down in main")
        controller.vel_pub.publish(Twist())
