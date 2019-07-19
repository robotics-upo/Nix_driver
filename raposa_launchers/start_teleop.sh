#!/usr/bin/env bash

sleep 1
source /opt/ros/kinetic/setup.bash
source /home/raposa/catkin_raposa/devel/setup.bash

export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME="raposa-idmind"

sleep 5

roslaunch raposa_launchers teleop.launch
