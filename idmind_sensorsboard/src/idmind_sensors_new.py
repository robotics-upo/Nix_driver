#!/usr/bin/env python

import rospy
from idmind_robot.msg import Log
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, TriggerResponse
from idmind_sensorsboard.msg import SystemVoltages
from sensors_drivers import Sensors

VERBOSE = 5
LOGS = 5


class SensorBoard:
    """ Class responsible for communication with sensorsboard """

    def __init__(self, address="/dev/idmind-sensorsboard", baudrate=115200, timeout=.5):
        #####################
        #  BOARD VARIABLES  #
        #####################
        self.logging = rospy.Publisher("/idmind_logging", Log, queue_size=10)
        self.sensors = Sensors(address=address, baudrate=baudrate, timeout=timeout)

        #################
        #  ROBOT STATE  #
        #################
        self.log("Setting Sensor Board Variables", 7)        
        # Lights - On/Off
        self.switch_lights = False        

        ################
        #  ROS TOPICS  #
        ################
        self.log("Setting ROS Publishers", 7)
        self.pub_volt = rospy.Publisher("/idmind_sensors/voltages", SystemVoltages, queue_size=10)
        self.pub_lights = rospy.Publisher("/idmind_sensors/lights", Bool, queue_size=10)

        ##################
        #  ROS SERVICES  #
        ##################
        self.log("Setting ROS Services", 7)
        rospy.Service("/idmind_sensors/switch_lights", Trigger, self.handle_switch_lights)

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

    def handle_switch_lights(self, _req):
        """ Service callback to switch the lights on or off """
        self.switch_lights = True
        msg = "Lights are turning {}".format("off" if self.sensors.lights_enabled else "on")
        self.log(msg, 5)        
        return TriggerResponse(True, msg)

    def publish_data(self):
        """
        Callable function that gets voltages and relays from board and publishes them and lights status
        :return:
        """

        # Voltages
        try:
            if not self.sensors.get_voltages():
                raise IOError("Failure to get voltages")
            else:
                v_msg = SystemVoltages()
                v_msg.header.stamp = rospy.Time.now()
                v_msg.motor_voltage = self.sensors.voltages["motor_voltage"]
                v_msg.electronic_voltage = self.sensors.voltages["electronic_voltage"]
                v_msg.motor_current = self.sensors.voltages["motor_current"]
                v_msg.electronic_current = self.sensors.voltages["electronic_current"]
                self.pub_volt.publish(v_msg)
        except IOError as io_err:
            self.log("Exception getting voltages", 3)

        # Light status
        try:
            self.pub_lights.publish(self.sensors.lights_enabled)
        except IOError as io_err:
            raise io_err

    def apply_control(self):
        """
        Callable function that controls the lights and relays
        :return:
        """
        if self.switch_lights:
            self.sensors.nix_lights(not self.sensors.lights_enabled)
            self.switch_lights = False
        return

    def start(self):
        """
        Main loop - Starts by updating and publishing status and then applies controls
        :return:
        """
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            try:
                self.publish_data()
                self.apply_control()
                r.sleep()
            except KeyboardInterrupt:
                rospy.logwarn("{}: Shutting down by user".format(rospy.get_name()))
                break


if __name__ == "__main__":
    rospy.init_node("idmind_sensors")
    s = SensorBoard("/dev/idmind_sensorsboard", baudrate=115200, timeout=0.5)
    s.start()
