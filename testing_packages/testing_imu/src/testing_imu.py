#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from testing_serial.testing_serial import TestingSerial
from tf_conversions import transformations
from serial import SerialException

# SERIAL_PORT_ADDRESS = '/dev/ttyACM0'
SERIAL_PORT_ADDRESS = '/dev/idmind-imu'
BAUDRATE = 115200
TIMEOUT = 1

ORIENTATION_COVARIANCE = 0.0025
ANGULAR_VELOCITY_COVARIANCE = 0.02
LINEAR_ACCELERATION_COVARIANCE = 0.04


class TestingImu:
    def __init__(self):
        try:
            self.serial_port = TestingSerial(address=SERIAL_PORT_ADDRESS, baudrate=BAUDRATE, timeout=TIMEOUT)
        except SerialException as e:
            print "Error! SerialException: %s" % str(e)
            raise e

        self.imu_publisher = rospy.Publisher("/testing_imu/imu", Imu, queue_size=10)

        self.imu_msg_initial = Imu()
        # Set the sensor covariances
        self.imu_msg_initial.orientation_covariance = [
            ORIENTATION_COVARIANCE, 0, 0,
            0, ORIENTATION_COVARIANCE, 0,
            0, 0, ORIENTATION_COVARIANCE
        ]
        self.imu_msg_initial.angular_velocity_covariance = [
            ANGULAR_VELOCITY_COVARIANCE, 0, 0,
            0, ANGULAR_VELOCITY_COVARIANCE, 0,
            0, 0, ANGULAR_VELOCITY_COVARIANCE
        ]
        self.imu_msg_initial.linear_acceleration_covariance = [
            LINEAR_ACCELERATION_COVARIANCE, 0, 0,
            0, LINEAR_ACCELERATION_COVARIANCE, 0,
            0, 0, LINEAR_ACCELERATION_COVARIANCE
        ]

        self.imu_offset = Quaternion()
        self.imu_offset.w = -1

    def get_imu_data(self):
        try:
            imu_data = self.serial_port.read_until("\r\n")

            if len(imu_data) == 0:
                print "IMU is not answering"
                return None
            else:
                imu_msg = self.imu_msg_initial
                dev_data = imu_data.split(" | ")

                for d in dev_data:
                    values = d.split(" ")
                    if values[0] == "Q:":
                        q1 = Quaternion()
                        q1.x = float(values[2])
                        q1.y = float(values[3])
                        q1.z = float(values[4])
                        q1.w = float(values[1])
                        q_off = self.imu_offset

                        euler = transformations.euler_from_quaternion([q1.x, q1.y, q1.z, q1.w])
                        # print "imu -> euler = (%s, %s, %s)" % (euler[0], euler[1], euler[2])

                        new_q = transformations.quaternion_multiply([q1.x, q1.y, q1.z, q1.w],
                                                                    [q_off.x, q_off.y, q_off.z, q_off.w])
                        imu_msg.orientation.x = new_q[0]
                        imu_msg.orientation.y = new_q[1]
                        imu_msg.orientation.z = new_q[2]
                        imu_msg.orientation.w = new_q[3]
                    elif values[0] == "A:":
                        imu_msg.linear_acceleration.x = float(values[1])
                        imu_msg.linear_acceleration.y = float(values[2])
                        imu_msg.linear_acceleration.z = float(values[3])
                    elif values[0] == "G:":
                        imu_msg.angular_velocity.x = float(values[1])
                        imu_msg.angular_velocity.y = float(values[2])
                        imu_msg.angular_velocity.z = float(values[3])
                    else:
                        print("{}: IMU is giving bad answers - {}".format(rospy.get_name(), imu_data), 5)
                        return None

                    imu_msg.header.frame_id = 'base_link_imu'
                    imu_msg.header.stamp = rospy.Time.now()
                    # ??? porque somar 0.5 s?

                    imu_msg_euler = transformations.euler_from_quaternion([imu_msg.orientation.x, imu_msg.orientation.y,
                                                                           imu_msg.orientation.z, imu_msg.orientation.w])
                    # print "imu -> imu_msg_euler = (%s, %s, %s)" % (imu_msg_euler[0], imu_msg_euler[1], imu_msg_euler[2])
                    return imu_msg
        except SerialException as e:
            print "Error! SerialException: %s" % str(e)
            return None
        except ValueError as e:
            print "Error! ValueError: %s" % str(e)
            return None

    def start_publishing(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            imu_data = self.get_imu_data()
            if not (imu_data is None):
                self.imu_publisher.publish(imu_data)
                rate.sleep()


if __name__ == "__main__":
    rospy.init_node("testing_imu")

    imu = TestingImu()
    imu.start_publishing()

    print "testing_imu stopped"
