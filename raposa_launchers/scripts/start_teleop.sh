#!/bin/bash

sleep 60

source /opt/ros/kinetic/setup.bash
source /home/raposa/catkin_raposa/devel/setup.bash

export ROS_HOSTNAME="raposa-idmind"
export ROS_IP=192.168.0.3
export ROS_MASTER_URI=192.168.4.2

if ping -q -c5 raposa-nav &> /dev/null
then
	export ROS_MASTER_URI=http://raposa-nav:11311
	export ROS_IP=10.5.5.2
	roslaunch raposa_launchers teleop.launch
	echo "true, found roscore"
else
	export ROS_MASTER_URI=http://localhost:11311
	roscore
	roslaunch raposa_launchers teleop.launch
	echo "roscore not found, starting own one"
fi
