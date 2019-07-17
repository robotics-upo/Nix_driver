#!/usr/bin/env python
import rospy
from testing_serial.testing_serial import TestingSerial
from std_msgs.msg import Float64, String, Header
from testing_sensorsboard.msg import Voltages
from testing_sensorsboard.srv import SetLights
from testing_sensorsboard.srv import SetRelayStatus

GET_VOLTAGES = 0x51
GET_VOLTAGES_SIZE = 8

GET_FIRMWARE_VERSION = 0X20
GET_FIRMWARE_VERSION_SIZE = 29

SET_RELAY_STATUS = 0x43
SET_RELAY_STATUS_SIZE = 4

SET_LIGHTS = 0x40
SET_LIGHTS_SIZE = 4

ELECTRONIC_BIT = 4
CABLE_BIT = 2
MOTOR_BIT = 1

SERIAL_PORT_ADDRESS = '/dev/idmind-sensorsboard'
BAUDRATE = 57600
TIMEOUT = 0.5

serial_port = TestingSerial(address=SERIAL_PORT_ADDRESS, baudrate=BAUDRATE, timeout=TIMEOUT)

is_electronic_relay_on = False
is_motor_relay_on = False
is_cable_relay_on = False
is_lights_on = False


def get_voltages():
    """##################################################
    #        Get_RaposaNG_Voltages                      #
    #    [0X51]                                         #
    #    receive:                                       #
    #    [0x51][Motor_Battery_Voltage]                  #
    #    [Electronic_Battery_Voltage]                   #
    #    [Cable_Voltage][Relay Status]                  # ordem??? motor electronic cable  VS  motor cable electronic
    #    [Sent_number][Chsum_H][Chsum_L]                #
    ##################################################"""
    get_voltages_msg = [GET_VOLTAGES]

    # print "getting voltages"
    voltages = serial_port.write_serial_message(get_voltages_msg, GET_VOLTAGES_SIZE)

    if voltages is None:
        print "error getting voltages"
        voltages = [-1, -1, -1, False, False, False]
    else:
        motor_voltage = ord(voltages[1])*.1
        electronic_voltage = ord(voltages[2])*.1
        cable_voltage = ord(voltages[3])*.1

        relay = ord(voltages[4])
        is_motor_relay_on = (relay & MOTOR_BIT) == 1
        is_cable_relay_on = ((relay & CABLE_BIT) >> 1) == 1
        is_electronic_relay_on = ((relay & ELECTRONIC_BIT) >> 2) == 1

        voltages = [motor_voltage, electronic_voltage, cable_voltage, is_motor_relay_on, is_electronic_relay_on, is_cable_relay_on]

    return voltages


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


def set_relay_status():
    """ ###############################################
    #    Set_RaposaNG_Power_Control                   #
    #    [0X43][Relay Status]                         #
    #    receive:                                     #
    #    [0x53][Sent_number][Chsum_H][Chsum_L]        #
    #    where [Relays Status] is                     #
    #    [1 0 0 0 0 electronic cable motor]           # ordem??? electronic cable motor  VS  electronic motor cable
    ################################################"""
    set_relay_status_msg = [SET_RELAY_STATUS]
    set_relay_status_msg.extend([1, 0, 0, 0, 0, is_electronic_relay_on, is_motor_relay_on, is_cable_relay_on])

    set_relay_status_response = serial_port.write_serial_message(set_relay_status_msg, SET_RELAY_STATUS_SIZE)
    if set_relay_status_response is None:
        print "Error setting relay status"
        return False
    else:
        return True


def set_lights():
    """################################################
    #    Set_RaposaNG_Ligths                          #
    #    [0X40][Lights_Enable]                        #
    #    receive:                                     #
    #    [0x40][Sent_number][Chsum_H][Chsum_L]        #
    ################################################"""
    set_lights_msg = [SET_LIGHTS, is_lights_on]

    set_lights_response = serial_port.write_serial_message(set_lights_msg, SET_LIGHTS_SIZE)
    if set_lights_response is None:
        print "Error setting lights"
        return False
    else:
        return True


if __name__ == "__main__":
    print "main"
    try:
        voltages_publisher = rospy.Publisher("voltages", Voltages, queue_size=10)
        firmware_publisher = rospy.Publisher('motorsboard_firmware', String, queue_size=10)

        rospy.init_node("testing_sensors")

        set_lights_service = rospy.Service('testing_sensors/set_lights', SetLights, set_lights)
        set_relay_service = rospy.Service('testing_sensors/set_relay', SetRelayStatus, set_relay_status)

        rate = rospy.Rate(1)

        print "before while"
        while not rospy.is_shutdown():
            # print "getting voltages..."
            voltages = get_voltages()

            # print "got voltages"
            header = Header()
            header.stamp = rospy.Time.now()

            voltages_message = Voltages(header, voltages[0], voltages[1], voltages[2],
                                        voltages[3], voltages[4], voltages[5])
            voltages_publisher.publish(voltages_message)

            rate.sleep()
    except rospy.ROSInterruptException:
        print "ROSInterruptException"
        pass
