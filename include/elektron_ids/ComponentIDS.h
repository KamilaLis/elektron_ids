#ifndef COMPONENTIDS_H
#define COMPONENTIDS_H

#include <iostream>

//ROS
#include <iostream>

//ROS
#include <ros/ros.h>
//#include <ros/topic_manager.h>
#include "XmlRpc.h"

namespace elektron_ids {

class ComponentIDS
{
public:
    ComponentIDS();
    XmlRpc::XmlRpcValue getSystemState();
    XmlRpc::XmlRpcValue getURI(const std::string& node_name);
    XmlRpc::XmlRpcValue getPubsName(const std::string& topic);
    XmlRpc::XmlRpcValue getSubsName(const std::string& topic);
    void preventInterception(const std::string &topic);
    void preventFabrication(const std::string &topic);
    void on_working();


private:

};

template<class T>
T getParam(const std::string &param_name, const T &default_val){
    ros::NodeHandle nh("~");
    std::string name_ = param_name;
    T value;
    nh.param<T>(name_, value, default_val);
    return value;
}

}; /* namespace elektron_ids */


#endif //COMPONENTIDS_H
