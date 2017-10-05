#!/bin/bash
source /opt/ros/kinetic/setup.bash
export ROS_MASTER_URI=http://192.168.1.164:11311
export ROS_IP=192.168.1.24
rosrun rqt_robot_steering rqt_robot_steering
