# This message is used for communication with the MotorBoard.
# If used in /idmind_motors/set_velocities, it communicates desired velocity values in m/s, for each wheel to the motor board.
# If used in /idmind_motors/wheel_odom, it communicates travelled distance values in m, for each wheel.
std_msgs/Header header    # Typical header
string kinematics         # Kinematics should be 2wd or omni. If 2wd, only front_right and front_left are valid
float32 front_right       #
float32 front_left        #
float32 back_right        #
float32 back_left         #

