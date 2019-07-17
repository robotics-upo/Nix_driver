#!/usr/bin/env python

import rospy
import rosnode
from numpy.linalg import norm
from idmind_robot.msg import Log
from sensor_msgs.msg import LaserScan
from numpy import sin, cos, arctan2
from tf2_geometry_msgs import PoseStamped
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Twist
# from tf_conversions import transformations as trf
from std_srvs.srv import Trigger, TriggerResponse
from idmind_controller.msg import CloserObstaclesMsg
from idmind_controller.srv import Obstacles, ObstaclesResponse

VERBOSE = 8
LOGS = 8


class IDMindRecovery:

    def __init__(self):
        self.control_freq = rospy.get_param("/move_base/controller_frequency", default=10.)
        self.closest_obstacles = {"front": 10., "back": 10., "left": 10., "right": 10.}

        # Logging
        self.logging = rospy.Publisher("/idmind_logging", Log, queue_size=10)

        #####################
        #  Load ROS Params  #
        #####################
        self.kinematics = rospy.get_param("/bot/kinematics", "2wd")
        self.max_linear_vel = rospy.get_param("/move_base/DWAPlannerROS/max_vel_x", 0.7)
        self.max_rot_vel = rospy.get_param("/move_base/DWAPlannerROS/max_rot_vel", 0.7)
        self.min_linear_vel = rospy.get_param("/move_base/DWAPlannerROS/min_trans_vel", 0.2)
        self.min_rot_vel = rospy.get_param("/move_base/DWAPlannerROS/min_rot_vel", 0.4)

        ########################
        #       Sensors        #
        ########################
        ok_flag = False
        while not ok_flag:
            try:
                self.log("Loading footprint", 7)
                self.footprint = []
                self.load_footprint()

                # Laser Scans
                self.log("Loading sensors", 7)
                # Register all declared sensors for move_base
                self.obstacles = []
                self.sensors = rospy.get_param("/move_base/local_costmap/obstacle_layer/observation_sources").split()
                self.lasers = []
                for sensor in self.sensors:
                    sensor_type = rospy.get_param("/move_base/local_costmap/obstacle_layer/"+sensor+"/data_type")
                    if sensor_type == "LaserScan":
                        self.log("Registering new laser", 5)
                        self.lasers.append(rospy.get_param("/move_base/local_costmap/obstacle_layer/"+sensor))
                    else:
                        self.log("Unknown Sensor Type - " + sensor_type, 5, alert="warn")

                self.log("Starting laser handlers", 7)
                self.last_scan = [LaserScan()] * len(self.lasers)
                # Start rospy Subscribers of sensors
                for laser in self.lasers:
                    no_message = True
                    while no_message:
                        try:
                            rospy.wait_for_message(laser["topic"], LaserScan, timeout=5)
                            rospy.Subscriber(laser["topic"], LaserScan, self.update_laser)
                            no_message = False
                        except rospy.ROSException:
                            rosnode.kill_nodes(laser["sensor_node"])
                            rospy.sleep(5)
                ok_flag = True
            except KeyError as key_err:
                self.log("Failed to load sensor information. Retrying in 5s. Log: {}".format(key_err), 3, alert="warn")
                rospy.sleep(5)

        ################
        # Localization #
        ################
        self.tf_buffer = Buffer()
        self.transform_listener = TransformListener(self.tf_buffer)

        ###############
        #  Auxiliary  #
        ###############
        # Compute edges of robot - front, right, back, left
        self.edges = [0, 0, 0, 0]
        for p in self.footprint:
            self.edges[0] = p[0] if p[0] > self.edges[0] else self.edges[0]
            self.edges[2] = p[0] if p[0] < self.edges[2] else self.edges[2]
            self.edges[1] = p[1] if p[1] < self.edges[1] else self.edges[1]
            self.edges[3] = p[1] if p[1] > self.edges[3] else self.edges[3]

        ################
        # ROS Services #
        ################
        rospy.Service("/idmind_controller/list_obstacles", Obstacles, self.return_obstacles)
        rospy.Service("/idmind_recovery/obstacle_handler", Trigger, self.handle_obstacles)
        rospy.Service("/idmind_recovery/test_rotation", Trigger, self.test_service)

        ################
        #  ROS Topics  #
        ################
        self.laser_pub = rospy.Publisher("/idmind_controller/closest_obstacles", CloserObstaclesMsg, queue_size=10)
        self.vel_pub = rospy.Publisher("/cmd_vel_idmind", Twist, queue_size=10)

    #################
    #   Auxiliary   #
    #################

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

    def update_laser(self, msg):
        """
        Callback when a new laser scan is published
        :param msg:
        :return:
        """
        idx = 0
        for laser in self.lasers:
            if laser["sensor_frame"] == msg.header.frame_id:
                break
            else:
                idx = idx + 1
        if idx > len(self.lasers):
            self.log("Received invalid scan reading in "+msg.header.frame_id+" frame.", 7, alert="warn")
        else:
            self.last_scan[idx] = msg

    def get_pose(self, frame_id, parent_frame="map"):
        """
        Returns frame_id pose in parent_frame (ex. return laser poase in map coordinates
        :param frame_id:
        :param parent_frame:
        :return f_pose:
        """

        while not self.tf_buffer.can_transform(parent_frame, frame_id, rospy.Time(0)):
            return PoseStamped()
        try:
            transf = self.tf_buffer.lookup_transform(parent_frame, frame_id, rospy.Time(0))
        except Exception as err:
            self.log("Laser Transforms Lookup Exception - "+str(err), 8, alert="warn")
            return

        f_pose = PoseStamped()
        f_pose.header = transf.header
        f_pose.pose.position = transf.transform.translation
        f_pose.pose.orientation = transf.transform.rotation
        return f_pose

    def point_in_hull(self, p, safe_distance=0.25, direction=None):
        """
        This function returns True if the point p is in the convex hull of points in hull, False otherwise
        :param p:
        :param safe_distance:
        :param direction:
        :return in_hull:
        """
        # Check for safety in general
        if direction is None:
            if (self.edges[2] - safe_distance < p[0] < self.edges[0] + safe_distance) and \
                    (self.edges[1] - safe_distance < p[1] < self.edges[3] + safe_distance):
                return True
            else:
                return False
        else:
            # If direction is given, check only points in that direction
            if direction == "front":
                if self.edges[1] < p[1] < self.edges[3] and self.edges[2] < p[0] < self.edges[0] + safe_distance:
                    return True
                else:
                    return False
            elif direction == "back":
                if self.edges[1] < p[1] < self.edges[3] and self.edges[2] - safe_distance < p[0] < self.edges[0]:
                    return True
                else:
                    return False
            elif direction == "right":
                if self.edges[0] > p[0] > self.edges[2] and self.edges[1] - safe_distance < p[1] < self.edges[3]:
                    return True
                else:
                    return False
            elif direction == "left":
                if self.edges[0] > p[0] > self.edges[2] and self.edges[1] < p[1] < self.edges[3] + safe_distance:
                    return True
                else:
                    return False
        return None

    ########################
    #  Obstacle Detection  #
    ########################
    def update_obstacles(self):
        """
        This function will update the obstacle list relative to base_link
        :return:
        """
        obstacle_list = []
        laser_scans = self.last_scan
        beam_pos = PoseStamped()
        for msg in laser_scans:
            if not self.tf_buffer.can_transform(msg.header.frame_id, "base_link", rospy.Time.now()):
                self.log("Invalid laser frame (normal at start)", 7, alert="warn")
                break

            curr_angle = msg.angle_min
            for beam in msg.ranges:
                if msg.range_min < beam < msg.range_max:
                    beam_pos.header.frame_id = msg.header.frame_id
                    beam_pos.pose.position.x = beam * cos(curr_angle)
                    beam_pos.pose.position.y = beam * sin(curr_angle)
                    base_pos = self.tf_buffer.transform(beam_pos, "base_link")
                    obstacle_list.append([base_pos.pose.position.x, base_pos.pose.position.y])
                curr_angle = curr_angle + msg.angle_increment

        self.obstacles = obstacle_list

    def return_obstacles(self, _req):
        """
        Callback to list_obstacles Service. Returns a list of [x1,y1,...xn,yn] positions of obstacles in base_link frame
        :param _req:
        :return res:
        """
        try:
            obstacle_list = list(self.obstacles)
            res = ObstaclesResponse()
            for it in obstacle_list:
                res.obstacles.extend(it)
            return res
        except IndexError as i_err:
            self.log("Error in service: {}".format(i_err), 2, alert="error")
            return ObstaclesResponse()

    def obstacles_pub(self):
        """
        Function that publishes the closest obstacles to the robot on four directions
        :return:
        """
        fdist = bdist = rdist = ldist = 1e6
        obstacle_list = list(self.obstacles)

        for obs in obstacle_list:
            if obs[0] > 0. and self.edges[1] < obs[1] < self.edges[3]:
                fdist = obs[0] - self.edges[0] if obs[0] - self.edges[0] < fdist else fdist

            if obs[0] < 0. and self.edges[1] < obs[1] < self.edges[3]:
                bdist = abs(obs[0] - self.edges[2]) if abs(obs[0] - self.edges[2]) < bdist else bdist

            if obs[1] > 0. and self.edges[2] < obs[0] < self.edges[0]:
                ldist = obs[1] - self.edges[3] if obs[1] - self.edges[3] < ldist else ldist

            if obs[1] < 0. and self.edges[2] < obs[0] < self.edges[0]:
                rdist = abs(obs[1] - self.edges[1]) if abs(obs[1] - self.edges[1]) < rdist else rdist

        msg = CloserObstaclesMsg()
        msg.front = fdist
        msg.back = bdist
        msg.right = rdist
        msg.left = ldist
        self.closest_obstacles["front"] = msg.front
        self.closest_obstacles["back"] = msg.back
        self.closest_obstacles["left"] = msg.left
        self.closest_obstacles["right"] = msg.right
        self.laser_pub.publish(msg)

    def rotate_safely(self, twist=Twist(), safe_distance=0.05):
        """
        Evaluates if a rotation at velocity (minimum by default) is safe
        :param twist: velocity to be used for turning (0 for default, other value in m/s)
        :param safe_distance: distance to obstacle to be considered safe (0.05m by default)
        :return safe: True/False
        """
        rot_vel = self.min_rot_vel if twist.angular.z == 0 else twist.angular.z
        dth = rot_vel / self.control_freq
        obstacle_list = [obs for obs in self.obstacles if norm(obs) < 1.]
        for obs in obstacle_list:
            d = norm(obs)
            th = arctan2(obs[1], obs[0])
            obstacle_nxt = [d*cos(th + dth), d*sin(th + dth)]
            if self.point_in_hull(obstacle_nxt, safe_distance=safe_distance):
                return False
        return True

    def translate_safely(self, twist=Twist(), safe_distance=0.05):
        """
        Evaluates if a translation at velocity (minimum by default) is safe
        :param twist: velocity to be used for translation(0 for default, other value in m/s)
        :param safe_distance: distance to obstacle to be considered safe (0.05m by default)
        :return safe: True/False
        """
        vx = self.min_linear_vel if twist.linear.x == 0 else twist.linear.x
        vy = twist.linear.y if self.kinematics == "omni" else 0.

        dx = vx / self.control_freq
        dy = vy / self.control_freq

        obstacle_list = [obs for obs in self.obstacles if norm(obs) < 1.]
        for obs in obstacle_list:
            obstacle_nxt = [obs[0] - dx, obs[1] - dy]
            if vy == 0:
                if vx > 0:
                    if self.point_in_hull(obstacle_nxt, safe_distance=safe_distance, direction="front"):
                        self.log("Unable to move forward", 5)
                        return False
                else:
                    if self.point_in_hull(obstacle_nxt, safe_distance=safe_distance, direction="back"):
                        self.log("Unable to move backwards", 5)
                        return False
            elif vx == 0:
                if vy > 0:
                    if self.point_in_hull(obstacle_nxt, safe_distance=safe_distance, direction="left"):
                        self.log("Unable to move left", 5)
                        return False
                else:
                    if self.point_in_hull(obstacle_nxt, safe_distance=safe_distance, direction="right"):
                        self.log("Unable to move right", 5)
                        return False
            else:
                if self.point_in_hull(obstacle_nxt, safe_distance=safe_distance):
                    self.log("Unable to move", 5)
                    return False
        return True

    def move_away_from_wall(self, wall_side, safe_dist=0.25):
        """
        This method should be called when the robot is close to a wall and needs to get away.
        It will alternatively rotate in place and move_forward, slowly and in close distances to unblock itself.
        :param wall_side: "left" or "right"
        :param safe_dist:
        :return:
        """
        self.log("Starting move_away_from_wall", 3)
        r = rospy.Rate(self.control_freq)
        try:
            while not rospy.is_shutdown() and self.closest_obstacles[wall_side] < safe_dist:
                # Rotate away from the wall
                rot_iter = False
                rot_vel = Twist()
                rot_vel.angular.z = (1 if wall_side == "right" else -1) * 2 * self.min_rot_vel
                while not rospy.is_shutdown() and self.rotate_safely(rot_vel) and self.closest_obstacles[wall_side] < safe_dist:
                    rot_iter = True
                    self.log("Rotate in place", 8)
                    self.vel_pub.publish(rot_vel)
                    r.sleep()

                # Move away from the wall
                tr_iter = False
                dt = 0
                tr_vel = Twist()
                tr_vel.linear.x = self.min_linear_vel
                last_pose = self.get_pose("base_link", "odom").pose.position
                while not rospy.is_shutdown() and self.translate_safely(tr_vel) and self.closest_obstacles[wall_side] < safe_dist and dt < 0.1:
                    curr_pose = self.get_pose("base_link", "odom").pose.position
                    dx = curr_pose.x - last_pose.x
                    dy = curr_pose.y - last_pose.y
                    dt = dt + norm([dx, dy])
                    tr_iter = True
                    self.log("Move forward", 8)
                    self.vel_pub.publish(tr_vel)
                    last_pose = curr_pose
                    r.sleep()

                # If no movement is possible, the robot might be stuck
                if not rot_iter and not tr_iter:
                    self.log("Robot unable to move", 5)
                    self.vel_pub.publish(Twist())
                    return False
                r.sleep()

            self.vel_pub.publish(Twist())
            self.log("Robot is away from wall", 3)
            return True
        except (KeyboardInterrupt, rospy.ROSException):
            self.log("move_away_from_wall interupted by exception", 2)
            self.vel_pub.publish(Twist())
            return False

    def move_in_corridor(self, safe_dist=0.10):
        self.log("Starting move_in_corridor", 5)
        r = rospy.Rate(self.control_freq)
        obs_flag = self.closest_obstacles["right"] < safe_dist and self.closest_obstacles["left"] < safe_dist
        while not rospy.is_shutdown() and obs_flag:
            obs_flag = self.closest_obstacles["right"] < safe_dist and self.closest_obstacles["left"] < safe_dist
            r.sleep()

    def test_service(self, _req):
        return TriggerResponse(self.move_away_from_wall("right"), "")

    #######################
    #  Obstacle handling  #
    #######################
    def handle_obstacles(self, _req):
        """
        This method should promote obstacle avoidance and issue a replanning after reaching safety.
        TODO: improve handle lateral obstacles
        :return:
        """
        # Initiate variables and stop the robot
        self.log("Handling Obstacles", 5)
        self.vel_pub.publish(Twist())
        rospy.sleep(rospy.Duration(1))

        r = rospy.Rate(self.control_freq)

        # Start the main recovery loop
        fwd = Twist()
        fwd.linear.x = self.min_linear_vel
        while not rospy.is_shutdown() and any(d < 0.2 for d in self.closest_obstacles.itervalues()):
            # Request laser readings in relation to base_link
            points = list(self.obstacles)

            # print "===== Safe to rotate? ====="
            # print "===== Safe to move forward? ====="
            if self.translate_safely():
                self.vel_pub.publish(fwd)
                r.sleep()
            else:
                # print "===== Reaction Forces ====="
                fr = [0., 0.]     # [fx, fy]
                for p in points:
                    if self.point_in_hull(p):
                        fr[0] = fr[0]-p[0]/(norm(p)**2)
                        fr[1] = fr[1]-p[1]/(norm(p)**2)

                new_vel = Twist()
                new_vel.linear.x = max(min(fr[0] / self.control_freq, self.max_linear_vel/2), - self.max_linear_vel/2)
                fy = fr[1] / self.control_freq
                if self.kinematics == "2wd":
                    new_vel.angular.z = max(min(fy, self.max_rot_vel/2), - self.max_rot_vel/2)
                else:
                    new_vel.linear.y = max(min(fy, self.max_linear_vel/2), - self.max_linear_vel/2)

                self.vel_pub.publish(new_vel)
                r.sleep()

        # Stop the robot and request a new plan
        self.vel_pub.publish(Twist())
        rospy.sleep(rospy.Duration(1))
        self.log("Obstacles avoided", 5)
        return TriggerResponse(True, "Obstacles avoided")

    ###############
    #  Main Loop  #
    ###############
    def start(self):
        r = rospy.Rate(self.control_freq)
        while not rospy.is_shutdown():
            try:
                self.update_obstacles()
                self.obstacles_pub()
                r.sleep()
            except KeyboardInterrupt:
                break
            except Exception as err:
                self.log("Exception in start():" + str(err), 4, alert="error")

        self.log("Shutting down", 3, alert="warn")


if __name__ == "__main__":
    rospy.init_node("idmind_recovery")
    rec = IDMindRecovery()
    rec.start()
