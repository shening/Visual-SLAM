#!/bin/bash
source /opt/ros/kinetic/setup.bash
cd ~/Visual-SLAM/IMUFilter_ws
source devel/setup.bash
roslaunch imu_complementary_filter complementary_filter.launch
