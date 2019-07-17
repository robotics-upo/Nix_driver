# idmind_robot

Supervisor package for robots in IDMind.
This package includes the following directories:

### cfg/
This directory should include configuration files, namely udev rules for the robot hardware, robot configuration details (name, kinematics, base and wheel sizes, true maximum and minimum velocities) and a configuration file for RViz visualization.

### launch/
This directory includes the necessary launch files for the robot. For simplicity, there should be 3 files: starting the robot, starting RViz and starting gmapping.
If necessary, include other files (launching hardware or simulation) to be called from the main ones.

### msg/ and srv/
These directories should include message or service definitions that are used by multiple packages. (to be honest, we tend to store these under the packages that generate them)

### src/
Supervisor nodes should be stored in this directory. Not usually necessary, but might be. 

## Usage
1. Copy the udev rule file to /etc/udev/rules.d - if correct, it should show idmind_motorsboard, idmind_sensorsboard, idmind_imu and idmind_joystick (and others defined there)
2. Run start_gmapping.launch to create a laser based map. Run `rosrun map_server map_saver -f [filename]` to save the map.
3. Run start_robot.launch to start the robot (in reality or simulation, to be defined in the file)
4. Run start_rviz.launch to start RViz in the local machine. Remember to set ROS_MASTER_URI to the roscore machine.