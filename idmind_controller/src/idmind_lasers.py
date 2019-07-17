#!/usr/bin/env python

import json
import rospy
import rosnode
from numpy import sin, cos
from idmind_robot.msg import Log
from sensor_msgs.msg import LaserScan
from tf2_geometry_msgs import PoseStamped
from std_srvs.srv import Trigger, TriggerResponse
from idmind_controller.msg import CloserObstaclesMsg
from tf2_ros import Buffer, TransformListener
from idmind_controller.srv import Obstacles, ObstaclesResponse

VERBOSE = 5
LOGS = 5


class IDMindLasers:
    """
    Class responsible for processing laser data and making available:
        []ObstacleList Service - Return a list of obstacle coordinates in base_link frame
        [x]/idmind_navigation/closest_obstacles Topic - Publishes the shortest distance to obstacles on four directions
    """

    def __init__(self):
        """
        Constructor of the IDMindLasers class.
        Reads sensor and robot information from move_base parameters.
        Initializes ROS subscribers and publishers
        """
        self.control_freq = rospy.get_param("/move_base/controller_frequency", default=10.)

        ########################
        #       Sensors        #
        ########################
        ok_flag = False
        while not ok_flag:
            try:
                # Load footprint (Its a string with a list of points)
                if rospy.has_param("/move_base/local_costmap/footprint") and \
                        len(rospy.get_param("/move_base/local_costmap/footprint")) > 2:
                    try:
                        f = rospy.get_param("/move_base/local_costmap/footprint")[2:-2].split("],[")
                        self.footprint = []
                        for val in f:
                            a = val.split(',')
                            self.footprint.append([float(a[0]), float(a[1])])
                    except AttributeError:
                        self.footprint = rospy.get_param("/move_base/local_costmap/footprint")
                else:
                    rad = rospy.get_param("/move_base/local_costmap/robot_radius")
                    self.footprint = [[rad, rad], [rad, -rad], [-rad, -rad], [-rad, rad]]
                self.obstacles = []
                # Laser Scans
                # Register all declared sensors for move_base
                self.sensors = rospy.get_param("/move_base/local_costmap/obstacle_layer/observation_sources").split()
                self.lasers = []
                for sensor in self.sensors:
                    sensor_type = rospy.get_param("/move_base/local_costmap/obstacle_layer/"+sensor+"/data_type")
                    if sensor_type == "LaserScan":
                        rospy.loginfo(rospy.get_name() + ": Registering new laser")
                        self.lasers.append(rospy.get_param("/move_base/local_costmap/obstacle_layer/"+sensor))
                    else:
                        rospy.logwarn(rospy.get_name()+": Unknown Sensor Type - " + sensor_type)
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
                msg = "{}: Fail to load sensor information. Retry in 5s. Log: {}".format(rospy.get_name(), key_err)
                self.log(msg, 2, alert="error")
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
        self.logging = rospy.Publisher("/idmind_logging", Log, queue_size=10)
        rospy.Service("/idmind_controller/list_obstacles", Obstacles, self.return_laser_readings)
        rospy.Service("/idmind_controller/request_footprint", Trigger, self.return_footprint)
        self.laser_pub = rospy.Publisher("/idmind_controller/closest_obstacles", CloserObstaclesMsg, queue_size=10)

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
            self.log("Laser Transforms Lookup Exception - "+str(err), 7, alert="warn")
            return

        f_pose = PoseStamped()
        f_pose.header = transf.header
        f_pose.pose.position = transf.transform.translation
        f_pose.pose.orientation = transf.transform.rotation
        return f_pose

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

    def return_laser_readings(self, _req):
        """
        Callback to list_laser_readings Service.
        Returns a list of [x1,y1,...xn,yn] positions of obstacles in base_link frame
        :param _req:
        :return res:
        """
        try:
            obstacle_list = list(self.obstacles)
            res = ObstaclesResponse()
            for it in obstacle_list:
                res.obstacles.extend(it)
            return res
        except Exception as exc:
            rospy.logerr("{}: Error in service: {}".format(rospy.get_name(), exc))

    def return_footprint(self, _req):
        msg = json.dumps(self.footprint)
        return TriggerResponse(True, msg)

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
        self.laser_pub.publish(msg)

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
                self.log("Exception in start():" + str(err), 3, alert="error")

        rospy.logwarn(rospy.get_name()+": Shutting down")


if __name__ == "__main__":
    rospy.init_node("idmind_lasers")
    lasers = IDMindLasers()
    lasers.start()
