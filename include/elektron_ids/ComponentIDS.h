#ifndef COMPONENTIDS_H
#define COMPONENTIDS_H

#include <iostream>

//ROS
#include <iostream>

//ROS
#include <ros/ros.h>
#include <ros/topic_manager.h>

namespace elektron_ids {

class ComponentIDS
{
public:
    ComponentIDS();
    void preventInterception(const std::string &topic);
    void preventFabrication(const std::string &topic);
    void on_working();


private:
    ros::TopicManager* manager_;
//TODO: params from yaml file
};

}; /* namespace elektron_ids */


#endif //COMPONENTIDS_H
