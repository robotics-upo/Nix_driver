#!/usr/bin/env python

import time
import serial
from threading import Lock


class IDMindSerial(serial.Serial):
    """Driver to connect to serial devices (imu, boards...)"""

    def __init__(self, addr, baudrate=115200, timeout=2, verify_checksum=False, verbose=False):
        """
        Initiates the serial connection with the device in address addr, using a specified baudrate.
        :param addr:
        :param baudrate:
        :param timeout:
        """
        self.verbose = False
        self.verify_checksum = verify_checksum
        self.mutex = Lock()

        try:
            serial.Serial.__init__(self, port=addr, baudrate=baudrate, timeout=timeout)
            print "Connection to " + addr + " was successful"
        except serial.SerialException as e:
            print("Connection to "+addr+" failed with: " + str(e))
            raise serial.SerialException(e)

    @staticmethod
    def to_bytes(val):
        """
        Transforms an integer [-32767, 32767] to 2 bytes
        :param val:
        :return [high_byte, low_byte]:
        """
        val = int(val)
        return [(val >> 8) & 0xFF, val & 0xFF]

    @staticmethod
    def to_num(b_high, b_low):
        """
        Transforms [b_high, b_low] to an integer [-32767, 32767]
        :param b_high:  HIGH Byte
        :param b_low:   LOW Byte
        :return res:    Integer in range [-32767, 32767]
        """
        res = (ord(b_high) << 8) | (ord(b_low) & 0xFF)
        if res > 32767:
            res = res - 65536
        return res

    def command(self, msg, nr_bytes, tries=5):
        """

        :param msg:
        :param nr_bytes:
        :param tries:
        :return:
        """

        # Check if the port is open
        if not self.is_open:
            raise serial.SerialException(1, "Serial port is not open")

        # Convert message to bytearray
        try:
            len(msg)
        except TypeError:
            msg = bytearray([msg])
        else:
            msg = bytearray(msg)

        # Write message to serial port and wait for response. Raise Exception in case of failure
        try:
            self.mutex.acquire()
            b = self.send_command(msg=msg, tries=tries)
            if b == 0:
                raise serial.SerialException(2, "Unable to send command")
            elif b != len(msg):
                raise serial.SerialException(3, "Failed to send complete message")
            else:
                if self.verbose:
                    print "Message {} send".format(msg)

            res = self.read_command(nr_bytes=nr_bytes, tries=tries)
            if res == 0:
                raise serial.SerialException(4, "No response from device")
            elif res < nr_bytes:
                raise serial.SerialException(4, "Incomplete response from device")
            else:
                if self.verify_checksum:
                    checksum = self.to_num(res[-2], res[-1])
                    bytesum = reduce(lambda x, y: x + y, [ord(el) for el in res[:-2]])
                    if ord(res[0]) == msg[0] and checksum == (bytesum & 0xffff):
                        return res
                    else:
                        serial.SerialException(4, "Checksum error")
                else:
                    return res
        except Exception as e:
            raise e
        finally:
            self.mutex.release()

    def send_command(self, msg, tries=5):
        """
        Sends a message through the serial port. The message should be a bytearray.
        Returns the number of bytes written to the port.
        :param msg:
        :param tries:
        :return written_bytes:
        """
        t = 0
        while t < tries:
            try:
                self.reset_input_buffer()
                self.reset_output_buffer()
                res = self.write(msg)
                return res
            except serial.SerialException as e:
                if self.verbose:
                    print e
                t = t + 1
                time.sleep(0.01)

        return 0

    def read_command(self, nr_bytes, tries=5):
        """
        Reads a specified number of bytes from the port and returns as a list
        :param nr_bytes:
        :param tries:
        :return [byte_1, ... byte_nr_bytes]:
        """
        res = []
        t = 0
        while t < tries:
            try:
                res = self.read(nr_bytes)
                
                if self.verbose:
                    if len(res) > 0:
                        print("Reply: "),
                        for i in range(len(res)):
                            print ord(res[i]),
                        print ("")
                    else:
                        print("No reply received")

                return res
            except serial.SerialException as e:
                if self.verbose:
                    print e
                t = t + 1
                time.sleep(0.01)
        return res

    def restart_port(self):
        try:
            serial.Serial.__init__(self, port=self.port, baudrate=self.baudrate, timeout=self.timeout)
            print "Connection to " + self.port + " was successful"
        except serial.SerialException as e:
            print("Connection to "+self.port+" failed with: " + str(e))
            raise serial.SerialException(e)


if __name__ == ":_main__":
    s = IDMindSerial("/dev/ttyACM0")
