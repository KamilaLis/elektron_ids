#!/bin/bash
source /opt/ros/kinetic/setup.bash
export ROS_MASTER_URI=http://192.168.8.102:11311
export ROS_IP=192.168.8.104
rosrun image_view image_view image:=/tower_top_kinect/rgb/image_raw
