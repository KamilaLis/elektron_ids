#include <iostream>

//ROS
#include <ros/ros.h>

//elektron_ids
#include "ComponentIDS.h"


int main(int argc, char* argv[]) {


    ROS_INFO("STARTED !!");

    // ROS loop
    ros::Rate rate(20.0);

    while (ros::ok())
    {

        rate.sleep();
    }

}
