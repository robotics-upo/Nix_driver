#!/usr/bin/env python

from numpy import pi, sign

import rospy
from idmind_robot.msg import Log
from idmind_motorsboard.msg import WheelsMB
from idmind_serial2.idmind_serialport import IDMindSerial
from std_msgs.msg import Int32

VERBOSE = 3
LOGS = 3

GET_ENC = 0x4A
GET_ARM = 0x48
GET_FIRMWARE = 0x20
SET_VEL = 0x56
SET_ARM = 0x42
SET_ARM_MAX = 690
SET_ARM_MIN = 444


class MotorBoard:
    """ Class responsible for communication with motorsboard"""

    def __init__(self, address="/dev/idmind-motorsboard", baudrate=57600, timeout=.5):
        #####################
        #  BOARD VARIABLES  #
        #####################
        self.logging = rospy.Publisher("/idmind_logging", Log, queue_size=10)
        # Connect to motorsboard
        try:
            self.ser = IDMindSerial(address, baudrate=baudrate, timeout=timeout)
        except Exception as e:
            self.log("Exception connecting to Motors Board: {}".format(e), 1, alert="err")
            raise e

        # Check for robot type
        try:
            self.kinematics = rospy.get_param("/motors/kinematics", "2wd")
        except KeyError:
            self.log("No kinematics information was found", 1, alert="err")
            raise KeyError("No kinematics information was found")

        # Check for wheel_radius
        try:
            self.wheel_radius = rospy.get_param("/motors/wheel_radius")
        except KeyError:
            self.log("No wheel radius information was found", 1, alert="err")
            raise KeyError("No wheel radius information was found")

        # Load ticks/turn (params stored under /idmind_motors/ticks and differ greatly for each robot)
        try:
            self.ticks_turn = rospy.get_param("/motors/ticks")
        except KeyError:
            self.log("No tick information was found, using defaults", 1, alert="warn")
            self.ticks_turn = {"front_left": 120000, "front_right": 120000}

        # Load velocity information
        try:
            self.max_linear_speed = rospy.get_param("/motors/max_vel")
            self.min_linear_speed = rospy.get_param("/motors/min_vel")
            self.max_ref = rospy.get_param("/motors/max_ref")
        except KeyError:
            self.log("No velocity information was found, using defaults", 1, alert="warn")
            self.max_linear_speed = 1.2
            self.min_linear_speed = 0.2
            self.max_ref = 1100

        # Load time tolerance for velocities to be passed to motors
        self.time_tolerance = rospy.get_param("/motors/time_tolerance", default=0.5)

        #################
        #  ROBOT STATE  #
        #################
        self.firmware = ""
        ################
        #  ROS TOPICS  #
        ################
        self.ticks = {"front_left": 0., "front_right": 0., "back_left": 0., "back_right": 0.}
        self.pub_dist = rospy.Publisher("/idmind_motors/wheel_odom", WheelsMB, queue_size=10)
        self.ticks_pub = rospy.Publisher("/idmind_motors/ticks", WheelsMB, queue_size=10)

        self.arm_position = Int32()
        self.pub_arm = rospy.Publisher("/idmind_motors/arm", Int32, queue_size=10)

        self.set_velocities = WheelsMB()
        self.set_velocities.header.frame_id = "/odom"
        self.set_velocities.header.stamp = rospy.Time.now()
        rospy.Subscriber("/idmind_motors/set_velocities", WheelsMB, self.handle_velocities)

        self.arm_ts = rospy.Time.now()
        self.set_arm = Int32()
        rospy.Subscriber("/idmind_motors/set_arm", Int32, self.handle_arm)

        ##################
        #  ROS SERVICES  #
        ##################

    #########################
    #  AUXILIARY FUNCTIONS  #
    #########################

    def log(self, msg, msg_level, log_level=-1, alert="info"):
        """
        Log function that publish in screen and in topic
        :param msg: Message to be published
        :param msg_level: Message level (1-10, where 1 is most important)
        :param log_level: Message level for logging (1-10, optional, -1 uses the same as msg_level)
        :param alert: Alert level of message - "info", "warn" or "error"
        :return:
        """
        if VERBOSE >= msg_level:
            if alert == "info":
                rospy.loginfo("{}: {}".format(rospy.get_name(), msg))
            elif alert == "warn":
                rospy.logwarn("{}: {}".format(rospy.get_name(), msg))
            elif alert == "error":
                rospy.logerr("{}: {}".format(rospy.get_name(), msg))
        if LOGS >= (log_level if log_level != -1 else msg_level):
            self.logging.publish(rospy.Time.now().to_sec(), rospy.get_name(), msg)

    def handle_velocities(self, msg):
        """
            ROS Topic callback to set motor velocities.
            :param msg:
            :return:
        """
        self.set_velocities = msg
    def handle_arm(self, msg):
        """
            ROS Topic callback to set arm position
            :param msg:
            :return:
        """
        self.arm_ts = rospy.Time.now()
        self.set_arm = msg

    ###################################################
    #           Raposa Motor GET Functions           #
    ###################################################
    def get_velocity_motor_ticks(self):
        """################################################
        #      GET_Velocity_Motor_Ticks                   #
        #        [0X4A]                                   #
        #        receive:                                 #
        #        [0x4A][Rt_Tick_H][Rt_Tick_L]             #
        #              [Lf_Tick_H][Lf_Tick_L]             #
        #              [Sent_number][Chsum_H][Chsum_L]    #
        ################################################"""
        try:
            bt = self.ser.send_command([GET_ENC])
            if bt != 1:
                return False
        except IOError as e:
            self.log("Exception asking velocities to motor board: {}".format(e), 3)
            raise IOError("Exception asking velocities to motor board: {}".format(e))

        try:
            byte_nr = 8
            res = self.ser.read_command(byte_nr)
            if len(res) < byte_nr:
                self.log("Wrong message: {}".format(res), 3)
                return False

            chk = self.ser.to_num(res[-2], res[-1])
            bytesum = reduce(lambda x, y: x + y, [ord(el) for el in res[:-2]])

            if ord(res[0]) != GET_ENC or chk != (bytesum & 0xffff):
                return False
            else:
                self.ticks["front_right"] = self.ser.to_num(res[1], res[2])
                self.ticks["front_left"] = self.ser.to_num(res[3], res[4])
                if self.kinematics == "omni":
                    self.ticks["back_right"] = self.ser.to_num(res[5], res[6])
                    self.ticks["back_left"] = self.ser.to_num(res[7], res[8])
                return True
        except Exception as e:
            self.log("Exception reading velocities from motor board: {}".format(e), 3)
            raise IOError("Exception reading velocities from motor board: {}".format(e))

    def get_arm_position(self):
        """################################################
        #      GET_arm_position                           #
        #        [0X48]                                   #
        #        receive:                                 #
        #        [0x48][POS_H][POS_L]                     #
        #              [Sent_number][Chsum_H][Chsum_L]    #
        ################################################"""
        try:
            msg = [GET_ARM]
            bt = self.ser.send_command(msg)
            if bt != 1:
                self.log("Failure to write to motor board for arm position", 3)
                return False
        except IOError as e:
            self.log("Exception asking arm to motor board: {}".format(e), 3)
            raise IOError("Exception asking arm to motor board: {}".format(e))

        try:
            byte_nr = 6
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
                self.arm_position = self.ser.to_num(res[1], res[2])
                return True
        except Exception as e:
            self.log("Exception reading arm from motor board: {}".format(e), 3)
            raise IOError("Exception reading arm from motor board: {}".format(e))

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
    def set_motor_velocities_control(self):
        """ ###############################################
        #           SET_Motor_Velocities_Control          #
        #               [0X56][Rt_H][Rt_L]                #
        #               [Lft_H][Lft_L]                    #
        #           receive:                              #
        #           [0x56][Sent_number][Chsum_H][Chsum_L] #
        ############################################### """

        dt = (rospy.Time.now() - self.set_velocities.header.stamp).to_sec()
        if dt > self.time_tolerance:
            self.log("Time tolerance for velocities exceeded: {}".format(dt), 5)
            msg = [SET_VEL, 0, 0, 0, 0] if self.kinematics == "2wd" else [SET_VEL, 0, 0, 0, 0, 0, 0, 0, 0]
        else:
            # Convert from wheel linear velocity to board reference (provided by C.Marques)
            # Convert from left and right speed velocities to [-1100, 1100]
            msg = [SET_VEL]
            ratio = self.max_ref / self.max_linear_speed
            fr_vel = self.set_velocities.front_right
            fl_vel = self.set_velocities.front_left
            fr_ref = sign(fr_vel) * max(self.min_linear_speed, abs(fr_vel)) * ratio
            fl_ref = sign(fl_vel) * max(self.min_linear_speed, abs(fl_vel)) * ratio
            refs = [fr_ref, fl_ref]

            if self.kinematics == "omni":
                br_vel = self.set_velocities.back_right
                bl_vel = self.set_velocities.back_left
                br_ref = sign(br_vel) * max(self.min_linear_speed, abs(br_vel)) * ratio
                bl_ref = sign(bl_vel) * max(self.min_linear_speed, abs(bl_vel)) * ratio
                refs.extend(br_ref)
                refs.extend(bl_ref)

            # Limit wheel velocities proportionally
            try:
                r = min(1, self.max_ref/max([abs(a) for a in refs]))
            except ZeroDivisionError:
                r = 1
            except RuntimeWarning:##TODO saber porque esto
                r = 1
            for w in refs:
                msg.extend(self.ser.to_bytes(w * r))

        try:
            bt = self.ser.send_command(msg)
            if bt != len(msg):
                return False
        except Exception as err:
            self.log("Exception setting velocities to motor board: {}".format(err), 3)
            raise IOError("Exception setting velocities to motor board: {}".format(err))

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
            self.log("Exception getting velocities reply from motor board: {}".format(err), 3)
            raise IOError("Exception getting velocities reply from  motor board: {}".format(err))

    def set_arm_position(self):
        """ ###############################################
        #           SET_Motor_Velocities_Control          #
        #               [0X42][Arm_H][Arm_L]              #
        #           receive:                              #
        #           [0x42][Sent_number][Chsum_H][Chsum_L] #
        ############################################### """

        dt = (rospy.Time.now() - self.arm_ts).to_sec()
        if dt > self.time_tolerance:
            self.log("Time tolerance for arm position exceeded: {}".format(dt), 5)
            msg = [SET_ARM]
            msg.extend(self.ser.to_bytes(self.arm_position))
        else:
            msg = [SET_ARM]
            msg.extend(self.ser.to_bytes(self.set_arm.data))

        try:
            bt = self.ser.send_command(msg)
            if bt != len(msg):
                return False
        except Exception as err:
            self.log("Exception setting arm position to motor board: {}".format(err), 3)
            raise IOError("Exception setting arm position to motor board: {}".format(err))

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
            self.log("Exception setting arm reply from motor board: {}".format(err), 3)
            raise IOError("Exception setting arm reply from  motor board: {}".format(err))

    def publish_data(self):
        """
        Callable function that gets arm position and ticks from board and publishes them
        :return:
        """
        # Update and publish arm position
        # try:
        #    if not self.get_arm_position():
        #        raise IOError("Failure to get arm position")
        #    else:
        #        arm_msg = Int32(self.arm_position)
        #        self.pub_arm.publish(arm_msg)
        # except IOError as io_err:
        #    raise io_err

        # Update and publish motor distances
        
        try:
            if not self.get_velocity_motor_ticks():
                raise IOError("Failure to get motor ticks")
            else:
                dist_msg = WheelsMB()
                dist_msg.header.frame_id = "/odom"
                dist_msg.header.stamp = rospy.Time.now()
                dist_msg.kinematics = self.kinematics
                #alpha = 1.0  # WTF is alpha?? TODO
                #dist=alpha*2*pi*self.r
                per = 2 * pi * self.wheel_radius
                dist_msg.front_left = per * self.ticks["front_left"]/self.ticks_turn["front_left"]
                dist_msg.front_right = per * self.ticks["front_right"]/self.ticks_turn["front_right"]
                if self.kinematics == "omni":
                   dist_msg.back_left = per * self.ticks["back_left"]/self.ticks_turn["back_left"]
                   dist_msg.back_right = per * self.ticks["back_right"]/self.ticks_turn["back_right"]

                self.pub_dist.publish(dist_msg)
                
                ticks_msg = WheelsMB()
                ticks_msg.front_left = self.ticks["front_left"]
                ticks_msg.front_right = self.ticks["front_right"]
                self.ticks_pub.publish(ticks_msg)

        except IOError as io_err:
            raise io_err

    def apply_control(self):
        """
        Callable function that controls the arm position and motor velocities
        :return:
        """
        try:
            if not self.set_arm_position():
                raise IOError("Failure to set arm position")
        except IOError as io_err:
            raise io_err

        # Set motor velocities
        try:
            if not self.set_motor_velocities_control():
                raise IOError("Failure to set motor velocities")
        except IOError as io_err:
            raise io_err

    def start(self):
        """
        Main loop - Starts by updating and publishing status and then applies controls
        :return:
        """
        error_counter = 0
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            try:
                self.publish_data()
                self.apply_control()
                r.sleep()                
            except IOError as io_err:
                error_counter = error_counter + 1
                self.log("{}: Failed communication - {}".format(rospy.get_name(), io_err), 5, alert="warn")
                if error_counter > 5:
                    raise io_err
            except KeyboardInterrupt:
                rospy.logwarn("{}: Shutting down by user".format(rospy.get_name()))
                break
        self.ser.close()


if __name__ == "__main__":
    rospy.init_node("idmind_motors")
    m = MotorBoard("/dev/idmind-motorsboard", baudrate=57600, timeout=0.5)
    m.start()
