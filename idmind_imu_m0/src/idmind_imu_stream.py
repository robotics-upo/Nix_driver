#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
#from idmind_arco.msg import Log
from serial import SerialException
from geometry_msgs.msg import Quaternion
import tf
from tf_conversions import transformations
from std_srvs.srv import Trigger, TriggerResponse
#from idmind_serial.idmind_serialport import IDMindSerial, IDMindSerialException
from idmind_serialport import IDMindSerial, IDMindSerialException
import math
VERBOSE = 3
LOGS = 3


class IDMindIMU:
    """
    This class extracts data from the Sparkfun Razor M0 9DoF IMU.
    The .ino file must be uploaded to the unit. It will publish to /imu the values of orientation, angular velocity
    and linear acceleration.
    In case the connection is lost, it will try to reconnect.

    TODO: Allow for calibration of components
    """
    def __init__(self):
        self.roll = 0.
        self.pitch = 0.
        self.yaw = 0.

        # angle accumulation

        self.angle_accum = 0
        self.accel_accum = 0
        # Logging
        #self.logging = rospy.Publisher("/idmind_logging", Log, queue_size=10)
        self.val_exc = 0

        # Connect to IMU
        self.connection()

        #number of iterations per calibration
        self.calibration_iters = 1000.0

        self.imu_reading = Imu()
        self.calibration = True
        self.imu_offset = Quaternion()
        self.imu_offset.w = 0.0
        self.imu_offset.x = 0.0
        self.imu_offset.y = 0.0
        self.imu_offset.z = 0.0
        
        # acceleration readings accumulator
        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0
        # gyro readings accumulator
        self.gx = 0.0
        self.gy = 0.0
        self.gz = 0.0
        self.imu_pub = rospy.Publisher("/imu", Imu, queue_size=10)

        rospy.Service("/idmind_razor/calibration", Trigger, self.request_calibration)

    def connection(self):
        """
        Function that connects to IMU port. Tries /dev/idmind-imu (created by udev rules) and then tries all ttyACM ports
        Repeats until found
        :return:
        """
        connected = False
        while not connected and not rospy.is_shutdown():
            try:
                self.ser = IDMindSerial("/dev/idmind-imu", baudrate=115200, timeout=1)
                connected = True
            except SerialException:
                self.log("Unable to connect to /dev/razor_m0", 2)
            except Exception as serial_exc:
                self.log(serial_exc, 2)
            if not connected:
                for i in range(0,10):
                    try:
                        self.ser = IDMindSerial("/dev/razor_m0"+str(i), baudrate=115200, timeout=0.5)
                        connected = True
                        break
                    except KeyboardInterrupt:
                        self.log("Node shutdown by user.", 2)
                        raise KeyboardInterrupt()
                    except SerialException:
                        self.log("Unable to connect to /dev/ttyACM"+str(i), 2)
                    except Exception as serial_exc:
                        self.log(serial_exc, 2)
            if not connected:
                self.log("IMU not found. Waiting 5 secs to try again.", 1)
                rospy.sleep(5)

    def log(self, msg, msg_level, log_level=-1):
        """
        Logging method for both verbose and logging topic
        :param msg: Message to be logged/displayed
        :param msg_level: if this value is lower than VERBOSE, display message on screen
        :param log_level: (optional) if this value is lower than LOGS, publish message
        :return:
        """
        return

    def request_calibration(self, _req):
        self.calibration = True
        return TriggerResponse(True, "Requesting calibration")

    def calibrate_imu(self):
        """
        This method will save the current orientation as the offset. All future publications will be adjusted in relation
        to the saved orientation
        :return:
        """
        print('Calibrating')
        self.log("Calibrating IMU", 3)
        r = rospy.Rate(100)
        calibrated = False
        # imu readings counters
        q_reads = 0
        a_reads = 0
        g_reads = 0
        # quaternion readings accumulator
        ow = 0.0
        ox = 0.0
        oy = 0.0 
        oz = 0.0
        # acceleration readings accumulator
        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0
        # gyro readings accumulator
        self.gx = 0.0
        self.gy = 0.0
        self.gz = 0.0
        while not calibrated and not rospy.is_shutdown():
            try:
                imu_data = self.ser.ser.read_until("\r\n").split(" | ")

                if q_reads < self.calibration_iters and a_reads < self.calibration_iters and g_reads < self.calibration_iters :
                    for s in imu_data:
                        s_data = s.split(" ")
                        if s_data[0] == "Q:" and q_reads < self.calibration_iters:
                            ow += float(s_data[1])
                            ox += float(s_data[2])
                            oy += float(s_data[3])
                            oz += float(s_data[4])
                            q_reads+=1
                        elif s_data[0] == "A:" and a_reads < self.calibration_iters:
                            self.ax += float(s_data[1])
                            self.ay += float(s_data[2])
                            self.az += float(s_data[3]) +1
                            a_reads+=1
                        elif s_data[0] == "G:" and g_reads < self.calibration_iters:
                            self.gx += float(s_data[1])
                            self.gy += float(s_data[2])
                            self.gz += float(s_data[3])
                            g_reads+=1
                        else:
                            rospy.logwarn("{}: IMU is giving bad answers - {}".format(rospy.get_name(), s_data[0]))
                else:
                    self.imu_offset.w = ow / self.calibration_iters
                    self.imu_offset.x = ox / self.calibration_iters
                    self.imu_offset.y = oy / self.calibration_iters
                    self.imu_offset.z = oz / self.calibration_iters
                    self.ax /= self.calibration_iters
                    self.ay /= self.calibration_iters
                    self.az /= self.calibration_iters
                    self.gx /= self.calibration_iters
                    self.gy /= self.calibration_iters
                    self.gz /= self.calibration_iters
                    calibrated = True
                    self.calibration = False
                    #Uncomment to get an idea of the bias 
                    #print("IMU Calibration offset: quat({:06.4f},{:06.4f},{:06.4f},{:06.4f}), acc({:06.4f},{:06.4f},{:06.4f}), gyro({:06.4f},{:06.4f},{:06.4f})".format(self.imu_offset.x, self.imu_offset.y, self.imu_offset.z, self.imu_offset.w,self.ax,self.ay,self.az,self.gx,self.gy,self.gz))

                    # rospy.loginfo(reads)
                r.sleep()

            except KeyboardInterrupt:
                raise KeyboardInterrupt()

    def update_imu(self):
        """
        Reads all characters in the buffer until finding \r\n
        Messages should have the following format: "Q: w x y z | A: x y z | G: x y z"
        :return:
        """
        # Create new message
        try:
            imuMsg = Imu()
            # Set the sensor covariances
            imuMsg.orientation_covariance = [
                0.0025, 0, 0,
                0, 0.0025, 0,
                0, 0, 0.0025
            ]
            imuMsg.angular_velocity_covariance = [
                0.02, 0, 0,
                0, 0.02, 0,
                0, 0, 0.02
            ]
            imuMsg.linear_acceleration_covariance = [
                0.04, 0, 0,
                0, 0.04, 0,
                0, 0, 0.04
            ]

            imu_data = self.ser.ser.read_until("\r\n")
            if len(imu_data) == 0:
                self.log("IMU is not answering", 2)
                return
            dev_data = imu_data.split(" | ")
            for d in dev_data:
                values = d.split(" ")
                if values[0] == "Q:":
                    pass
                elif values[0] == "A:":
                    imuMsg.linear_acceleration.x = float(values[1])- self.ax
                    imuMsg.linear_acceleration.y = float(values[2])- self.ay
                    imuMsg.linear_acceleration.z = float(values[3])- self.az
                    self.accel_accum += imuMsg.linear_acceleration.x * 9.8 * 0.01
                elif values[0] == "G:":
                    imuMsg.angular_velocity.x = float(values[1])- self.gx
                    imuMsg.angular_velocity.y = float(values[2])- self.gy
                    imuMsg.angular_velocity.z = float(values[3]) - self.gz
                    self.angle_accum += imuMsg.angular_velocity.z * 0.01
                else:
                    self.log("{}: IMU is giving bad answers - {}".format(rospy.get_name(), imu_data), 5)
                    self.log(values[0], 5)
                    return

            # integrate our own orientation
            self.roll += imuMsg.angular_velocity.x / 100
            roll = self.roll % 360 - 180
            self.pitch += imuMsg.angular_velocity.y / 100
            pitch = self.pitch % 360 - 180
            self.yaw += imuMsg.angular_velocity.z / 100
            yaw = self.yaw % 360 - 180
            q = transformations.quaternion_from_euler(math.radians(roll), math.radians(pitch), math.radians(yaw))
            imuMsg.orientation.x = q[0]
            imuMsg.orientation.y = q[1]
            imuMsg.orientation.z = q[2]
            imuMsg.orientation.w = q[3]

            # Handle message header
            imuMsg.header.frame_id = "base_link_imu"
            imuMsg.header.stamp = rospy.Time.now()+rospy.Duration(0.5)
            self.imu_reading = imuMsg

        except SerialException:
            self.log("SerialException while reading from IMU", 3)
            self.calibration = True
        except ValueError as val_err:
            self.log("Value error from IMU data - {}".format(val_err), 5)
            self.val_exc = self.val_exc + 1
        except Exception as imu_exc:
            self.log(imu_exc, 3)
            raise imu_exc


    def publish_imu(self):
        self.imu_pub.publish(self.imu_reading)

    def start(self):

        r = rospy.Rate(100) # changed from 100
        while not rospy.is_shutdown():
            try:
                self.log("Bytes waiting: {}".format(self.ser.ser.in_waiting), 5)
                if self.calibration:
                    self.calibrate_imu()
                else:
                    self.update_imu()
                    self.publish_imu()
                r.sleep()
            except KeyboardInterrupt:
                self.log("{}: Shutting down by user".format(rospy.get_name()), 2)
                break


if __name__ == "__main__":
    rospy.init_node("idmind_razor")
    imu = IDMindIMU()
    imu.start()
    rospy.loginfo("{}: Node stopped".format(rospy.get_name()))
