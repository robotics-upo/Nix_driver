#!/usr/bin/env python

import time
import subprocess
import serial.tools.list_ports
from idmind_serial2.idmind_serialport import IDMindSerial


class Sensors:
    def __init__(self, address="", baudrate=115200, verbose=10, timeout=1.0):

        self.verbose = verbose
        self.firmware = ""
        self.lights_enabled = False
        self.voltages =     {"motor_voltage": 0,
                            "motor_current": 0,
                            "electronic_voltage": 0,
                            "electronic_current": 0}

        connected = False
        # Try to connect to given port. If fails, search in all available ports
        while not connected:
            try:
                self.search_connect(baudrate)
                connected = True
            except KeyboardInterrupt:
                print("\tInterrupted by user")
                raise KeyboardInterrupt
            except Exception as err:
                print("\tException occurred:\n\t\t{}".format(err))

        # Set low latency
        try:
            subprocess.check_call(['setserial', self.s.port, 'low_latency'])
        except subprocess.CalledProcessError as err:
            if self.verbose > 2:
                print("\tUnable to set low latency - {}".format(err.returncode))
            time.sleep(0.5)

    ##########################
    #     Serial methods     #
    ##########################
    def search_connect(self, baudrate=115200):
        if self.verbose > 2:
            print("Searching for SensorBoard")        
        for addr in [comport.device for comport in serial.tools.list_ports.comports()]:
            # If the lsof call returns an output, then the port is already in use!
            try:
                subprocess.check_output(['lsof', '+wt', addr])
                continue
            except subprocess.CalledProcessError:
                # Port is not being used, try to connect
                try:
                    if self.verbose > 4:
                        print("Connecting to SensorsBoard on {}".format(addr))
                    self.s = IDMindSerial(addr=addr, baudrate=baudrate, timeout=1.0, verbose=self.verbose)
                    res = self.get_firmware()
                    if res:                        
                        if "Board1 fw 1.00 2019/01/15" in res:
                            if self.verbose > 3:
                                print("\tSensorBoard found in {}".format(addr))
                                time.sleep(0.5)
                            return
                        else:
                            if self.verbose > 3:
                                print("\tWrong firmware reply from {}".format(addr))
                                time.sleep(0.5)
                                self.s.close()
                            continue
                    else:
                        print("\tNo response from firmware request")
                        continue
                except Exception as err:
                    print("Exception detecting SensorBoard: {}".format(err))
                time.sleep(1.0)
        raise Exception("Sensor Board not found")

    def close(self):
        try:
            self.s.close()
            return True
        except Exception as err:
            print("\tException closing serial: {}".format(err))
            return False

    #######################
    #     SET methods     #
    #######################

    def nix_lights(self, state=1):
        """
        Sets Enable/Disable Electronic Battery
        Send: [0X40] Enable Lights
              [0x41] Disable Lights

        :param state:
        :return True/False:
        """
        msg = [0x40 if state else 0x41]        
        if self.s.send_command(bytearray(msg)) != len(msg):
            print("Error sending nix_lights")
            return False
        else:
            light_buffer = self.s.read_command(4)
            if len(light_buffer) == 4:
                checksum = self.s.to_num(light_buffer[-2], light_buffer[-1])
                bytesum = sum([ord(a) for a in light_buffer[0:-2]])
                if ord(light_buffer[0]) == msg[0] and bytesum == checksum:
                    self.lights_enabled = True if state else False
                    return True
                else:
                    print("Reply msg for lights returned wrong checksum")
                    return False
            else:
                print("Reply msg for lights returned wrong length")
                return False
            

    def get_voltages(self):
        msg = [0x30]
        if self.s.send_command(bytearray(msg)) != len(msg):
            print("Error getting voltages")
            return False
        else:
            light_buffer = self.s.read_command(12)
            if len(light_buffer) == 12:
                checksum = self.s.to_num(light_buffer[-2], light_buffer[-1])
                bytesum = sum([ord(a) for a in light_buffer[0:-2]])
                if ord(light_buffer[0]) == msg[0] and bytesum == checksum:
                    self.voltages["motor_voltage"] = self.s.to_num(light_buffer[1], light_buffer[2]) * 0.01
                    self.voltages["motor_current"] = self.s.to_num(light_buffer[3], light_buffer[4]) * 0.01
                    self.voltages["electronic_voltage"] = self.s.to_num(light_buffer[5], light_buffer[6]) * 0.01           
                    self.voltages["electronic_current"] = self.s.to_num(light_buffer[7], light_buffer[8]) * 0.01    
                    return self.voltages
                else:
                    print("Reading voltages reply with wrong checksum")
                    return False
            else:                
                print("Reading voltages reply with wrong length")
                return False

    def get_firmware(self):
        msg = [0x20]
        if self.s.send_command(bytearray(msg)) != 1:
            if self.verbose > 5:
                print("Error reading firmware")
            return False
        else:
            light_buffer = self.s.read_command(29)
            if len(light_buffer) == 29:
                checksum = self.s.to_num(light_buffer[-2], light_buffer[-1])
                bytesum = sum([ord(a) for a in light_buffer[0:-2]])
                if ord(light_buffer[0]) == 0x20 and bytesum == checksum:
                    self.firmware = str(light_buffer[1:-2])
                    return self.firmware
                else:
                    if self.verbose > 5:
                        print("Error with reading firmware response")
                    return False
            else:
                if self.verbose > 5:
                    print("Error receiving firmware response")
                return False


if __name__ == "__main__":
    s = Sensors()
    # s.nix_lights(True)
    print s.get_voltages()
    time.sleep(1)
    print s.nix_lights(True)
    time.sleep(1)
    print s.nix_lights(False)    