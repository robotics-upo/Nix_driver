#ROS package idmind_motorsboard

## Description
This package contains the module for ROS communication with IDMind's Raposa Motor Board

## Usage
1. Install pyserial package
2. Clone the `idmind_serial2` package
3. Clone this package
4. Run `catkin_make`
5. Topics Subscribed:
    - /idmind_motors/set_arm (Int32) - Position of the Arm to be set
    - /idmind_motors/set_velocities (WheelMB) - Velocities to set for each wheel, in m/s
6. Topics Published:
    - /idmind_motors/arm (Int32) - Current position of the Arm 
    - /idmind_motors/distances (WheelMB) - Distances covered by each wheel, in m    
    - /idmind_logging (idmind_robot/Log) - Logging messages
    
## Troubleshooting
#####Q: This node does not detect the Motors Board.
A: The most likely reason is that the udev rule did not activate. Did you copy the udev rule from idmind_robot to /etc/udev/rules.d? If so, are the baord details correct?  

#####Q: Velocities are publish in /idmind_motors/set_velocities, but the wheels don't move
A: This message needs to have a timestamp, for timestamp comparisons. In this case, if the controller fails, the motors stop.

#####Q: The distance travelled by the robot does not match the wheel odometry published
A: Remeber that wheel odometry is very noisy. However, some calibration may be necessary. See the "Calibration" topic below.

#####Q: This node does not apply to my robot
A: Change it, by talking to Carlos Marques and checking his API. If there are more variables to be handloed, create topics as necessary.

## Calibration
The distance travelled by a wheel is measured by the number of ticks. The ratio between number of ticks and distance may vary from motor to motor and with time. In order to calibrate:
1. Run `python src/calibration.py`
2. Lift the robot and turn the wheels 10 laps manually
3. Stop the program by pressing "Ctrl+C"
4. Write down the "Ticks/turn" information and replace them in `cfg/idmind_motors.yaml`

### Contacts
- Carlos Neves - cneves@idmind.pt
- Henrique Póvoa - hpovoa@idmind.pt
