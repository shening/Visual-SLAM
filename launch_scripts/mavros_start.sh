#!/bin/bash
source /opt/ros/kinetic/setup.bash
case $HOSTNAME in
	(safe50-UDOO-x86) export ROS_IP=192.168.0.220;roslaunch mavros px4.launch;;
	(*) echo "Computer not recognized!";;
esac
