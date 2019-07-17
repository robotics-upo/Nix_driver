# idmind_navigation
This package will handle navigation and odometry of the robot


####Files
- **navigation_node.py**

    This node will read velocity goals and apply a smoother and limiter before sending the wheel velocities to Motor Board or to Gazebo
    
- **odometry_node.py**

    This node will read odometry information from the wheels and heading information from the IMU, if available, and publish estimated odometry of the robot.  

##Services Provided
- **/idmind_navigation/toggle_joystick (Trigger)**

    This service will activate or deactivate the joystick
    
- **/idmind_navigation/toggle_controller (Trigger)**

    This service will switch the controller, from move_base to idmind_controller
    
- **/idmind_navigation/calibrate_imu (Trigger)**

    IMU calibration service, sets current 3D orientation as the starting point 
    
##Topics Subscribed
- **/idmind_logging (/idmind_robot/Log)**

    Logging only

- **/cmd_vel (Twist)**
    
    Velocity goal given by move_base
- **/cmd_vel_idmind (Twist)**
    
    Velocity goal given by idmind_controller
- **/cmd_vel_joy (Twist)**
    
    Velocity goal given by joystick

- **/idmind_motors/wheel_odom (WheelOdom)**

    Odometry information for each wheel

- **/idmind_sensors/voltages (SystemVoltages)**

    Voltages information, published by the sensors board
    
- **/imu (IMU)**

    IMU information, used for robot odometry estimation
    
##Topics Published
- **/idmind_motors/set_velocities (WheelOdom)**

    Goal wheel velocity information, to be handled by the motor board
    
- **/gazebo/cmd_vel (Twist)**
    
    Goal platform velocity information, to be handled by Gazebo