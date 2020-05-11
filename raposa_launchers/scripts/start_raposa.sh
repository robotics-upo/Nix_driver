#!/bin/bash
#TODO Maybe the best is to publish only /odom  (not tf) when there is navigation pc, and this last one will decide if publish or not odometry    
#TODO   So if there is no navigation pc, the drivers pc should publish /odom and the corresponding tf odom->base_link
#TODO   This can be passed as an arg to the launch and the launch pass it to the odometry node

source /opt/ros/kinetic/setup.bash
source /home/raposa/catkin_raposa/devel/setup.bash
export ROS_HOSTNAME=raposa-idmind
export ROS_IP=192.168.8.3
rosclean purge -y
#Check if we have ping to the raposa-nav host:

ping -c1 raposa-nav
if [ $? -eq 0 ]
then # If there is ping, wait for ros master
    export ROS_MASTER_URI=http://raposa-nav:11311
    until rostopic list ; do echo "Waiting for roscore..."; sleep 1; done
    sudo rm /home/raposa/.bashrc
    sudo cp /home/raposa/.bashrc_nav /home/raposa/.bashrc
    roslaunch raposa_launchers teleop.launch
else #If there is no ping, start the robot in basic mode (only drivers and roscore in the bottom pc)
    export ROS_MASTER_URI=http://raposa-idmind:11311
    sudo rm  /home/raposa/.bashrc
    sudo cp /home/raposa/.bashrc_isolated /home/raposa/.bashrc
    roslaunch raposa_launchers teleop.launch only_driver_pc:=true
fi

