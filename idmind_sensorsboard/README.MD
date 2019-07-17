#ROS package idmind_sensorsboard

## Description
This package contains the module for ROS communication with IDMind's Raposa Sensors Board

## Usage
1. Install pyserial package
2. Clone the `idmind_serial2` package
3. Clone this package
4. Run `catkin_make`
5. Topics Published:
    - /idmind_sensors/voltages (Voltages) - Tension and Relay positions information
    - /idmind_sensors/lights (Bool) - Current status of Lights
6. Services Provided:
    - /idmind_sensors/switch_lights (Trigger) - Switches lights
    - /idmind_sensors/switch_motor_relay (Trigger) - Switches motor relay
    - /idmind_sensors/switch_cable_relay (Trigger) - Switches cable relay, if cable is connected
    - /idmind_sensors/switch_electronic_relay (Trigger) - Switches electronic relay, if cable is connected
    
### Contacts
- Carlos Neves - cneves@idmind.pt
- Henrique Póvoa - hpovoa@idmind.pt
