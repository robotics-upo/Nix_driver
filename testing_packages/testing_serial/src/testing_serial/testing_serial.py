#!/usr/bin/env python
import serial

import rospy

SERIAL_PORT_ADDRESS = '/dev/ttyUSB0'
BAUDRATE = 57600
TIMEOUT = .5

BYTE_SIZE = 255
SIGNED_INT_MAX = 32767
UNSIGNED_INT_MAX = 65536


class TestingSerial(serial.Serial):
    # constructor
    def __init__(self, address=SERIAL_PORT_ADDRESS, baudrate=BAUDRATE, timeout=TIMEOUT):
        try:
            # call to constructor of superclass required
            serial.Serial.__init__(self, port=address, baudrate=baudrate, timeout=timeout)
        except serial.SerialException, e:
            print("Error opening serial port")
            raise serial.SerialException(e)

    def write_serial_message(self, msg, res_byte_nr):
        if not self.is_open:
            print "Error: serial port not open"
            return None

        # print "%s Writing serial: %s" % (rospy.Time.now().secs, str(msg)[0:])
        # print str(msg)[1:-1]

        self.reset_input_buffer()
        self.reset_output_buffer()

        bytes_written = self.write(bytearray(msg))
        if bytes_written != len(msg):
            # raise IOError("Error writing serial message")
            print "Error writing serial message"
            return None

        res = self.read(res_byte_nr)

        # print "Serial port response: %s" % [ord(s) for s in res][0:]

        if len(res) != res_byte_nr:
            # raise IOError("Format error in serial response")
            print "response size is wrong: %s" % len(res)
            return None

        # print "len(res) = %s\t" % len(res)
        for i in range(len(res)):
             print ord(res[i]),
        print "\n"

        checksum = self.byte_to_number(res[-2], (res[-1]))
        if checksum > SIGNED_INT_MAX:
            checksum -= UNSIGNED_INT_MAX

        byte_sum = 0
        for x in res[:-2]:
            byte_sum += ord(x)

        # ser.close()

        if ord(res[0]) != msg[0] or checksum != byte_sum:
            print "checksum error!"
            return None
        else:
            return res

    @staticmethod
    def byte_to_number(byte_high, byte_low):
        res = (ord(byte_high) << 8) | (ord(byte_low) & 0xFF)
        if res > SIGNED_INT_MAX:
            res -= UNSIGNED_INT_MAX

        return res
