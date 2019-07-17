#!/usr/bin/env python

import time
import serial
from threading import Lock, ThreadError


class IDMindSerialException(Exception):
    def __init__(self, function=None, msg=""):
        self.function = function
        self.msg = msg

    def __str__(self):
        return "IDMindSerialException in {}: {}".format(self.function, self.msg)


class IDMindSerial:
    """Driver to connect to serial devices (imu, boards...)"""

    def __init__(self, addr, baudrate=115200, timeout=2, verbose=False):
        """
        Initiates the serial connection with the device in address addr, using a specified baudrate.
        :param addr:
        :param baudrate:
        :param timeout:
        """
        self.mutex = Lock()
        try:
            # Is there a way to run in another machine?
            self.ser = serial.Serial(addr, baudrate=baudrate, timeout=timeout)
            print "Connection to "+addr+" was successful"
        except serial.SerialException as e:
            print("Connection to "+addr+" failed with: " + str(e))
            self.ser_flag = False
            raise serial.SerialException(e)

    def toBytes(self, val):
        """
        Transforms an integer [-32767, 32767] to 2 bytes
        :param val:
        :return [high_byte, low_byte]:
        """
        val = int(val)
        return [(val >> 8) & 0xFF, val & 0xFF]

    def toNum(self, bval):
        """
        Transforms [bval_high, bval_low] to an integer [-32767, 32767]
        :param bval:
        :return res:
        """
        res = (ord(bval[0]) << 8) | (ord(bval[1]) & 0xFF)
        if res > 32767:
            res = res - 65536
        return res

    def sendCommand(self, msg, tries=5):
        """
        Sends a message through the serial port. The message should be a list of integer values.
        Returns the number of bytes written to the port.
        :param msg:
        :param tries:
        :return written_bytes:
        """
        """Receives a list. Checks if the port is open. If needed, formats the message.
        Tries to send the message multiple times"""
        if not self.ser.is_open:
            return 0
        res = 0
        t = 0
        if type(msg) == int or type(msg) == float:
            msg = [msg]
        while t < tries:
            try:
                self.mutex.acquire()
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                res = self.ser.write(bytearray(msg))
            except serial.SerialException:
                t = t + 1
                time.sleep(0.01)
            finally:
                self.mutex.release()

            if res == len(msg):
                return res
            elif t == 2:
                print "sendCommand() failed to send {}. Restarting Port.".format(msg)
                self.restart_port()
            elif t == 3:
                raise IDMindSerialException("sendCommand", "Unable to send message {}".format(msg))
        return 0

    def readCommand(self, nr_bytes, tries=5):
        """
        Reads a specified number of bytes from the port and returns as a list
        :param nr_bytes:
        :param tries:
        :return [byte_1, ... byte_nr_bytes]:
        """
        if not self.ser.is_open:
            return None
        res = []
        t = 0
        while t < tries:
            try:
                self.mutex.acquire()
                res = self.ser.read(nr_bytes)
            except serial.SerialException as e:
                t = t + 1
                time.sleep(0.01)
                if t == tries - 1:
                    self.mutex.release()
                    raise IDMindSerialException("readCommand", "Unable to read message")
            finally:
                self.mutex.release()
                return res

        return None

    def isOpen(self, tries=5):
        """
        Checks if the serial port is open
        :param tries:
        :return True/False:
        """
        t = 0
        while t < tries:
            try:
                return self.ser.is_open
            except Exception:
                t = t + 1

    def restart_port(self):
        addr = self.ser.port
        baudrate = self.ser.baudrate
        timeout = self.ser.timeout
        try:
            self.ser.close()
        except Exception:
            print "Unable to close Port. Restarting."
        try:
            self.ser = serial.Serial(addr, baudrate=baudrate, timeout=timeout)
            print "Connection to " + addr + " was successful"
        except serial.SerialException as e:
            print("Connection to " + addr + " failed with: " + str(e))
            self.ser_flag = False
            raise serial.SerialException(e)

    def __del__(self):
        try:
            self.mutex.release()
        except ThreadError:
            pass
        except AttributeError:
            pass

        try:
            if self.ser.is_open:
                self.ser.close()
                return True
            else:
                return False
        except serial.SerialException as e:
            if self.verbose:
                print "Exception while attempting to close port: "+str(e)
        except AttributeError:
            print "Port was not initiated"