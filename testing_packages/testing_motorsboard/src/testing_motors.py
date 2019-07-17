#!/usr/bin/env python
import rospy
from numpy import sign, pi
from std_msgs.msg import Int32, String, Header
from testing_motorsboard.msg import MotorsState
from testing_motorsboard.srv import SetArmPosition, SetArmPositionResponse, SetMotorVelocityResponse
from testing_motorsboard.srv import SetMotorVelocity
from testing_serial.testing_serial import TestingSerial

from idmind_serial2.idmind_serialport import IDMindSerial

SET_ARM_POSITION = 0x42
GET_ARM_POSITION = 0x48
GET_ARM_POSITION_SIZE = 6
SET_ARM_POSITION_SIZE = 4
ARM_POSITION_MAX = 690
ARM_POSITION_MIN = 444

SET_MOTOR_VELOCITY = 0X56
GET_MOTOR_VELOCITY = 0X4A
GET_MOTOR_VELOCITY_SIZE = 8
SET_MOTOR_VELOCITY_SIZE = 4

GET_FIRMWARE_VERSION = 0X20
GET_FIRMWARE_VERSION_SIZE = 29

# SERIAL_PORT_ADDRESS = '/dev/ttyUSB0'
SERIAL_PORT_ADDRESS = '/dev/idmind-motorsboard'
BAUDRATE = 57600
TIMEOUT = 0.5

MAX_REFERENCE_VELOCITY = 1100
MAX_LINEAR_VELOCITY = 0.7
MIN_LINEAR_VELOCITY = 0.06

serial_port = TestingSerial(address=SERIAL_PORT_ADDRESS, baudrate=BAUDRATE, timeout=TIMEOUT)
ratio = 0


# def write_serial_message(msg, res_byte_nr):
#     if not ser.is_open:
#         print "Error: serial port not open"
#         return None
#
#     # print "Writing byte array to serial port"
#     # print str(msg)[1:-1]
#
#     ser.reset_input_buffer()
#     ser.reset_output_buffer()
#
#     bytes_written = ser.write(msg)
#     res = ser.read(res_byte_nr)
#
#     # print "Serial port response: %s" % [ord(s) for s in res][1:-1]
#
#     if bytes_written != len(msg):
#         raise IOError("Error writing serial message")
#
#     if len(res) != res_byte_nr:
#         print "response size is wrong: %s" % len(res)
#         raise IOError("Format error in serial response")
#     # for i in range(len(res)):
#     #     print ord(res[i]),
#
#     checksum = (ord(res[-2]) << 8) | (ord(res[-1]) & 0xFF)
#     if checksum > SIGNED_INT_MAX:
#         checksum -= UNSIGNED_INT_MAX
#
#     byte_sum = 0
#     for x in res[:-2]:
#         byte_sum += ord(x)
#
#     # ser.close()
#
#     if ord(res[0]) != msg[0] or checksum != byte_sum:
#         print ("checksum error!")
#         return None
#     else:
#         return res


def get_arm_position():
    """################################################
    #      GET_arm_position                           #
    #        [0X48]                                   #
    #        receive:                                 #
    #        [0x48][POS_H][POS_L]                     #
    #              [Sent_number][Chsum_H][Chsum_L]    #
    ################################################"""
    get_arm_position_msg = [GET_ARM_POSITION]

    print "getting arm position"
    arm_position = serial_port.write_serial_message(get_arm_position_msg, GET_ARM_POSITION_SIZE)
    # arm_position = ser.command(get_arm_position_msg, GET_ARM_POSITION_SIZE)
    if arm_position is None:
        print("error getting arm position")
        arm_position = -1
    else:
        arm_position = (ord(arm_position[1]) << 8) | ord(arm_position[2])

    return arm_position


def set_arm_position(arm_position):
    """ ###############################################
    #           SET_Motor_Velocities_Control          #
    #               [0X42][Arm_H][Arm_L]              #
    #           receive:                              #
    #           [0x42][Sent_number][Chsum_H][Chsum_L] #
    ############################################### """
    if not (type(arm_position) is int):
        raise ValueError('arm_position is not an int')

    set_arm_position_msg = [SET_ARM_POSITION]
    arm_position = int(arm_position)
    set_arm_position_msg.extend([arm_position >> 8 & 0xFF, arm_position & 0xFF])

    set_arm_position_response = serial_port.write_serial_message(set_arm_position_msg, SET_ARM_POSITION_SIZE)
    if set_arm_position_response is None:
        return SetArmPositionResponse(False)
    else:
        print("Arm position (" + str(arm_position) + ") set successfully")
        return SetArmPositionResponse(True)


def get_velocity_motor_ticks():
    """################################################
    #      GET_Velocity_Motor_Ticks                   #
    #        [0X4A]                                   #
    #        receive:                                 #
    #        [0x4A][Rt_Tick_H][Rt_Tick_L]             #
    #              [Lf_Tick_H][Lf_Tick_L]             #
    #              [Sent_number][Chsum_H][Chsum_L]    #
    ################################################"""
    get_motor_ticks_msg = [GET_MOTOR_VELOCITY]

    # print "get wheels velocity"
    motor_ticks = serial_port.write_serial_message(get_motor_ticks_msg, GET_MOTOR_VELOCITY_SIZE)

    if motor_ticks is None:
        print "error getting motors velocities"
        motor_ticks_right, motor_ticks_left = -1, -1
    else:
        motor_ticks_right = serial_port.byte_to_number(motor_ticks[1], motor_ticks[2])
        motor_ticks_left = serial_port.byte_to_number(motor_ticks[3], motor_ticks[4])

    return [motor_ticks_right, motor_ticks_left]


def set_velocity_motor_ticks(motor_velocity):
    """ ###############################################
    #           SET_Motor_Velocities_Control          #
    #               [0X56][Rt_H][Rt_L]                #
    #               [Lft_H][Lft_L]                    #
    #           receive:                              #
    #           [0x56][Sent_number][Chsum_H][Chsum_L] #
    ############################################### """
    # if not (type(motor_velocity.motor_velocity_right) is int or type(motor_velocity.motor_velocity_left) is int):
    #     raise ValueError('motor velocities are not ints')

    set_motor_velocity_msg = [SET_MOTOR_VELOCITY]

    v_right, v_left = motor_velocity.velocity_right, motor_velocity.velocity_left
    motor_velocity_right = sign(v_right) * ratio * max(MIN_LINEAR_VELOCITY, abs(v_right))
    motor_velocity_left = sign(v_left) * ratio * max(MIN_LINEAR_VELOCITY, abs(v_left))

    print "motor_velocity_right, motor_velocity_left = %s, %s" % (motor_velocity_right, motor_velocity_left)

    if abs(motor_velocity_right) > MAX_REFERENCE_VELOCITY or abs(motor_velocity_left) > MAX_REFERENCE_VELOCITY:
        motor_velocity_max = max(abs(motor_velocity_right), abs(motor_velocity_left))
        r = float(MAX_REFERENCE_VELOCITY) / motor_velocity_max
        motor_velocity_right *= r
        motor_velocity_left *= r

    motor_velocity_right, motor_velocity_left = int(motor_velocity_right), int(motor_velocity_left)

    print "motor_velocity_right, motor_velocity_left = %s, %s" % (motor_velocity_right, motor_velocity_left)
    set_motor_velocity_msg.extend([motor_velocity_right >> 8 & 0xFF, motor_velocity_right & 0xFF,
                                   motor_velocity_left >> 8 & 0xFF, motor_velocity_left & 0xFF])

    set_motor_velocities_response = serial_port.write_serial_message(set_motor_velocity_msg, SET_MOTOR_VELOCITY_SIZE)
    if set_motor_velocities_response is None:
        print "Error setting motors velocity"
        return SetMotorVelocityResponse(False)
    else:
        # print "Motors velocities set successfully"
        return SetMotorVelocityResponse(True)


def get_firmware():
    """################################################
    #      GET_firmware                               #
    #        [0X20]                                   #
    #        receive:                                 #
    #        [0x20][FirmWare-> 25 Bytes]              #
    #              [Sent_number][Chsum_H][Chsum_L]    #
    #      where FirmWare:                            #
    #      "Board1 fw 1.00 2017/07/11"                #
    ################################################"""
    get_firmware_msg = [GET_FIRMWARE_VERSION]
    firmware_version = serial_port.write_serial_message(get_firmware_msg, GET_FIRMWARE_VERSION_SIZE)

    if firmware_version is None:
        print("error getting firmware version")
    else:
        firmware_version = firmware_version[1:-3]

    return firmware_version


if __name__ == "__main__":
    try:
        motors_state_publisher = rospy.Publisher('/testing_motors/motors_state', MotorsState, queue_size=10)
        # firmware_publisher = rospy.Publisher('motorsboard_firmware', String, queue_size=10)

        rospy.init_node('testing_motors')

        ratio = MAX_REFERENCE_VELOCITY / MAX_LINEAR_VELOCITY
        # set_motor_velocity_service = rospy.Service('/testing_motorsboard/set_motor_velocity', SetMotorVelocity,
        #                                           set_velocity_motor_ticks)
        # set_arm_position_service = rospy.Service('testing_motorsboard/set_arm_position', SetArmPosition,
        #                                         set_arm_position)
        rate = rospy.Rate(15)
        # serial_port = serial.Serial('/dev/ttyUSB0', baudrate=57600, timeout=.5)
        # serial_port = IDMindSerial('/dev/ttyUSB0', baudrate=57600, timeout=0.5)
        # msg = [0x48]

        while not rospy.is_shutdown():
            # arm_position = get_arm_position()
            arm_position = -1
            motor_velocity = get_velocity_motor_ticks()

            header = Header()
            header.stamp = rospy.Time.now()
            motors_state = MotorsState(header, arm_position, motor_velocity[0], motor_velocity[1])
            motors_state_publisher.publish(motors_state)

            # serial_port.write(bytearray(msg))
            # res = serial_port.read(6)
            # serial_port.reset_input_buffer()
            # serial_port.reset_output_buffer()

            # print ([ord(l) for l in res])

            rate.sleep()
        # rospy.spin()
    except rospy.ROSInterruptException:
        print "ROSInterruptException"
        pass
