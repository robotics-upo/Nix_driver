#ROS package idmind_serial

## Description
This package contains the module for serial/usb communication nodes responsible for communication with devices.

## Usage
1. Install pyserial package
2. Clone the `idmind_serial2` package
3. Run `catkin_make`
4. The basic commands are:
    - Import using `from idmind_serial.idmind_serialport import IDMindSerial`  
    - Start the serial port with `IDMindSerial(addr, baudrate, timeout,  verify_checksum, verbose)`
    - Use `write_command(bytearray(msg))` and `read_command(nr_bytes)` for simple communication
    - Use `command(msg, nr_bytes)` for a robust communication  

### Contacts
- Carlos Neves - cneves@idmind.pt
- Henrique Póvoa - hpovoa@idmind.pt
