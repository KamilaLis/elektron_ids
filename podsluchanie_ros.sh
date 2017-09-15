#!/bin/bash
source /opt/ros/kinetic/setup.bash
export ROS_MASTER_URI=http://192.168.1.241:11311
export ROS_IP=192.168.1.24
rosrun image_view image_view image:=/head_camera_rgb/image_raw
