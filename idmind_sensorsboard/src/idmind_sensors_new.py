#!/usr/bin/env python

import time
import rospy
from idmind_robot.msg import Log
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, TriggerResponse
from idmind_sensorsboard.msg import SystemVoltages
from idmind_serial2.idmind_serialport import IDMindSerial
import subprocess
import serial.tools.list_ports

VERBOSE = 3
LOGS = 3

SET_POWER = 0x43
SET_LIGHTS = 0x40
GET_VOLTAGES = 0x51
GET_FIRMWARE = 0x20

ELECTRONIC_BIT = 4
CABLE_BIT = 2
MOTOR_BIT = 1

MIN_VOL = 14.


class SensorBoard:
    """ Class responsible for communication with sensorsboard """

    def __init__(self, address="/dev/idmind-sensorsboard", baudrate=57600, timeout=.5):
        #####################
        #  BOARD VARIABLES  #
        #####################
        self.logging = rospy.Publisher("/idmind_logging", Log, queue_size=10)
        # Connect to sensorsboard
        connected = False
        # Search in all available ports until connection is achieved
        while not connected:
            try:
                self.search_connect()
                connected = True
            except KeyboardInterrupt:
                self.log("\tInterrupted by user", 5)
                raise KeyboardInterrupt
            except Exception as err:
                self.log("\tException occurred:\n\t\t{}".format(err), 5)

        #################
        #  ROBOT STATE  #
        #################
        self.log("Setting Sensor Board Variables", 7)
        self.firmware = ""
        # Lights - On/Off
        self.switch_lights = False
        self.lights = False

        # About the relays:
        # electronic_relay - determines if electronic is connected to battery (true) or not. Always connected to cable.
        # motor_relay - determines if motors are connected to battery (true) or not
        # cable_relay - determines if motors are connected to cable (true) or not
        self.switch_relays = False
        self.relays = {"electronic_relay": False, "motor_relay": False, "cable_relay": False}       # Connections
        self.set_relays = {"electronic_relay": False, "motor_relay": False, "cable_relay": False}   # Connections
        self.voltages = {"motor_battery": 0., "electronic_battery": 0., "cable": 0.}                 # Voltages

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
        rospy.Service("/idmind_sensors/switch_motor_relay", Trigger, self.handle_switch_motor_relay)
        rospy.Service("/idmind_sensors/switch_cable_relay", Trigger, self.handle_switch_cable_relay)
        rospy.Service("/idmind_sensors/switch_electronic_relay", Trigger, self.handle_switch_electronic_relay)
        rospy.Service("/idmind_sensors/set_low_latency", Trigger, self.handle_low_latency)

    #########################
    #  AUXILIARY FUNCTIONS  #
    #########################
    def search_connect(self):
        self.log("Searching for SensorBoard", 5)
        for addr in [comport.device for comport in serial.tools.list_ports.comports()]:
            # If the lsof call returns an output, then the port is already in use!
            try:
                subprocess.check_output(['lsof', '+wt', addr])
                continue
            except subprocess.CalledProcessError:
                # Port is not being used, try to connect
                try:
                    self.log("Connecting to SensorsBoard on {}".format(addr), 4)
                    self.ser = IDMindSerial(addr=addr, baudrate=57600, timeout=0.5)
                    res = self.get_firmware()
                    if res:
                        self.log("Firmware: {}".format(res), 5)
                        if "Board1 fw 1.00 2019/01/15" in res:
                            self.log("\tSensorBoard found in {}".format(addr), 5)
                            time.sleep(0.5)
                            return
                        else:
                            self.log("\tWrong firmware reply from {}".format(addr), 5)
                            time.sleep(0.5)
                            continue
                    else:
                        self.log("\tNo response from firmware request", 5)
                        continue
                except Exception as err:
                    print("Exception detecting SensorBoard: {}".format(err))
        raise Exception("Sensor Board not found")

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
        self.lights = not self.lights
        msg = "Lights are {}".format("on" if self.lights else "off")
        self.log(msg, 5)
        self.switch_lights = True
        return TriggerResponse(True, msg)

    def handle_switch_motor_relay(self, _req):
        """ Service Callback to switch motor battery relay on or off.
            Always check if relay changed.
        """
        self.set_relays["motor_relay"] = not self.relays["motor_relay"]
        msg = "Motor relay will be {}".format("on" if self.set_relays["motor_relay"] else "off")
        self.log(msg, 5)
        self.switch_relays = True
        return TriggerResponse(True, msg)

    def handle_switch_electronic_relay(self, _req):
        """ Service Callback to switch electronic battery relay on or off
            If relay is off, it can always be turned on.
            If relay is on, it can only be turned off if cable is connected.
            Always check if relay changed.
        """
        if not self.relays["electronic_relay"]:
            self.set_relays["electronic_relay"] = not self.relays["electronic_relay"]
            msg = "Electronic relay will be {}".format("on" if self.set_relays["electronic_relay"] else "off")
            self.log(msg, 5)
            self.switch_relays = True
            return TriggerResponse(True, msg)
        else:
            if self.voltages["cable"] > 16.:
                self.set_relays["electronic_relay"] = not self.relays["electronic_relay"]
                msg = "Electronic relay will be {}".format("on" if self.set_relays["electronic_relay"] else "off")
                self.log(msg, 5)
                self.switch_relays = True
                return TriggerResponse(True, msg)
            else:
                msg = "Refused to switch electronic relay: cable is not connected."
                self.log(msg, 5)
                return TriggerResponse(False, msg)

    def handle_switch_cable_relay(self, _req):
        """ Service callback to switch motors cable relay on or off
            Always check if relay changed.
        """
        if self.voltages["cable"] > 16.:
            self.set_relays["cable_relay"] = not self.relays["cable_relay"]
            msg = "Cable relay will be {}".format("on" if self.set_relays["cable_relay"] else "off")
            self.log(msg, 5)
            self.switch_relays = True
            return TriggerResponse(True, msg)
        else:
            msg = "Refused to switch cable relay: cable is not connected"
            self.log(msg, 5)
            self.switch_relays = True
            return TriggerResponse(False, msg)
    def handle_low_latency(self, _req):
        """ Service to set the FTDI time 1 ms
            
        """
        self.set_low_latency()
        return TriggerResponse(True, "Finished")
    ###################################################
    #           Raposa Sensor GET Functions           #
    ###################################################
    def get_voltages(self):
        """##################################################
        #        Get_RaposaNG_Voltages                      #
        #    [0X51]                                         #
        #    receive:                                       #
        #    [0x51][Motor_Battery_Voltage]                  #
        #    [Electronic_Battery_Voltage]                   #
        #    [Cable_Voltage][Relay Status]                  #
        #    [Sent_number][Chsum_H][Chsum_L]                #
        ##################################################"""
        try:
            bt = self.ser.send_command([GET_VOLTAGES])
            if bt != 1:
                return False
        except IOError as e:
            self.log("Exception asking voltages to sensor board: {}".format(e), 3)
            raise IOError("Exception asking voltages to sensor board: {}".format(e))

        try:
            byte_nr = 8
            res = self.ser.read_command(byte_nr)
            if len(res) < byte_nr:
                self.log("Wrong message: {}".format(res), 3)
                return False

            chk = self.ser.to_num(res[-2], res[-1])
            bytesum = reduce(lambda x, y: x + y, [ord(el) for el in res[:-2]])

            if ord(res[0]) != GET_VOLTAGES or chk != (bytesum & 0xffff):
                return False
            else:
                self.voltages["motor_battery"] = ord(res[1]) * 0.1
                self.voltages["electronic_battery"] = ord(res[2]) * 0.1
                self.voltages["cable"] = ord(res[3]) * 0.1
                self.relays["motor_relay"] = (ord(res[4]) & MOTOR_BIT) == 1
                self.relays["cable_relay"] = ((ord(res[4]) & CABLE_BIT) >> 1) == 1
                self.relays["electronic_relay"] = ((ord(res[4]) & ELECTRONIC_BIT) >> 2) == 1
                return True
        except Exception as e:
            self.log("Exception reading voltages from sensor board: {}".format(e), 3)
            raise IOError("Exception reading voltages from sensor board: {}".format(e))

    def get_firmware(self):
        """################################################
        #      GET_firmware                               #
        #        [0X20]                                   #
        #        receive:                                 #
        #        [0x20][FirmWare-> 25 Bytes]              #
        #              [Sent_number][Chsum_H][Chsum_L]    #
        #      where FirmWare:                            #
        #      "Board1 fw 1.00 2017/07/11"                #
        ################################################"""
        try:
            msg = [GET_FIRMWARE]
            bt = self.ser.send_command(msg)
            if bt != 1:
                self.log("Failure to write to motor board for firmware", 3)
                return False
        except IOError as e:
            self.log("Exception asking firmware to motor board: {}".format(e), 3)
            raise IOError("Exception asking firmware to motor board: {}".format(e))

        try:
            byte_nr = 29
            res = self.ser.read_command(byte_nr)
            if len(res) < byte_nr:
                self.log("Wrong message: {}".format(res), 3)
                return False

            chk = self.ser.to_num(res[-2], res[-1])
            bytesum = reduce(lambda x, y: x + y, [ord(el) for el in res[:-2]])
            if ord(res[0]) != msg[0] or chk != (bytesum & 0xffff):
                self.log("Wrong message: {}".format(res), 3)
                return False
            else:
                self.firmware = res[1:-3]
                return True
        except Exception as e:
            self.log("Exception reading firmware from motor board: {}".format(e), 3)
            raise IOError("Exception reading firmware from motor board: {}".format(e))

    ###################################################
    #           Raposa Motor SET Functions            #
    ###################################################
    def set_power_control(self):
        """ ###############################################
        #    Set_RaposaNG_Power_Control                   #
        #    [0X43][Relay Status]                         #
        #    receive:                                     #
        #    [0x53][Sent_number][Chsum_H][Chsum_L]        #
        #    where [Relays Status] is                     #
        #    [1 0 0 0 0 electronic cable motor]           #
        ################################################"""
        try:
            relays_byte = 128 + (4 if self.set_relays["electronic_relay"] else 0) + (
                2 if self.set_relays["cable_relay"] else 0) + (1 if self.set_relays["motor_relay"] else 0)
            msg = [SET_POWER, relays_byte]
            bt = self.ser.send_command(msg)
            if bt != len(msg):
                return False
        except Exception as err:
            self.log("Exception setting relays to sensors board: {}".format(err), 3)
            raise IOError("Exception setting relays to sensors board: {}".format(err))

        try:
            byte_nr = 4
            res = self.ser.read_command(byte_nr)
            if len(res) < byte_nr:
                self.log("Wrong message: {}".format(res), 3)
                return False

            chk = self.ser.to_num(res[-2], res[-1])
            bytesum = reduce(lambda x, y: x + y, [ord(el) for el in res[:-2]])

            if ord(res[0]) != msg[0] or chk != (bytesum & 0xffff):
                return False
            else:
                return True
        except Exception as err:
            self.log("Exception setting relays reply from sensors board: {}".format(err), 3)
            raise IOError("Exception setting relays reply from sensors board: {}".format(err))
    
    def set_low_latency(self):
        try:
            subprocess.check_call(['setserial','/dev/idmind-sensorsboard','low_latency'])
        except subprocess.CalledProcessError as err:
            if VERBOSE > 2:
                print("\t Unable to set low lattency: {}".format(err.returncode))

    def set_lights_control(self):
        """################################################
        #    Set_RaposaNG_Ligths                          #
        #    [0X40][Lights_Enable]                        #
        #    receive:                                     #
        #    [0x40][Sent_number][Chsum_H][Chsum_L]        #
        ################################################"""
        try:
            msg = [SET_LIGHTS, (0x01 if self.lights else 0x00)]
            bt = self.ser.send_command(msg)
            if bt != len(msg):
                return False
        except Exception as err:
            self.log("Exception setting lights to sensors board: {}".format(err), 3)
            raise IOError("Exception setting lights to sensors board: {}".format(err))

        try:
            byte_nr = 4
            res = self.ser.read_command(byte_nr)
            if len(res) < byte_nr:
                self.log("Wrong message: {}".format(res), 3)
                return False

            chk = self.ser.to_num(res[-2], res[-1])
            bytesum = reduce(lambda x, y: x + y, [ord(el) for el in res[:-2]])

            if ord(res[0]) != msg[0] or chk != (bytesum & 0xffff):
                return False
            else:
                return True
        except Exception as err:
            self.log("Exception setting lights reply from sensors board: {}".format(err), 3)
            raise IOError("Exception setting lights reply from sensors board: {}".format(err))

    def publish_data(self):
        """
        Callable function that gets voltages and relays from board and publishes them and lights status
        :return:
        """

        # Voltages and relays
        try:
            if not self.get_voltages():
                raise IOError("Failure to get voltages")
            else:

                v_msg = SystemVoltages()
                v_msg.header.stamp = rospy.Time.now()
                v_msg.motor_battery = self.voltages["motor_battery"]
                v_msg.electronic_battery = self.voltages["electronic_battery"]
                v_msg.cable = self.voltages["cable"]
                v_msg.electronic_relay = self.relays["electronic_relay"]
                v_msg.motor_relay = self.relays["motor_relay"]
                v_msg.cable_relay = self.relays["cable_relay"]
                self.pub_volt.publish(v_msg)
                plugged = v_msg.cable_relay or v_msg.cable > 16
                if (v_msg.motor_battery < MIN_VOL or v_msg.electronic_battery < MIN_VOL) and not plugged:
                    low_bat = "Motor" if v_msg.motor_battery < v_msg.electronic_battery else "Electronic"
                    # self.log("{} battery is low! Charge Them!".format(low_bat), 1, alert="error")

        except IOError as io_err:
            self.log("Exception getting voltages", 3)

        # Light status
        try:
            self.pub_lights.publish(self.lights)
        except IOError as io_err:
            raise io_err

    def apply_control(self):
        """
        Callable function that controls the lights and relays
        :return:
        """
        if self.switch_lights:
            self.set_lights_control()
            self.switch_lights = False
        if self.switch_relays:
            self.set_power_control()
            self.switch_relays = False
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
    s = SensorBoard("/dev/idmind-sensorsboard", baudrate=57600, timeout=0.5)
    s.start()
