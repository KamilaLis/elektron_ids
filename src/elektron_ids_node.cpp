
#include <ros/ros.h>
#include "elektron_ids/ComponentIDS.h"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "component_ids");
    ros::NodeHandle n;
    elektron_ids::ComponentIDS component_ids;

    ROS_INFO("ELEKTRON_IDS STARTED !!");

    // ROS loop
    ros::Rate rate(20.0);

    while (ros::ok())
    {
        component_ids.on_working();
        rate.sleep();
    }

}
