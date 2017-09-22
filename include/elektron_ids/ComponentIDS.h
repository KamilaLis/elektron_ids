#ifndef COMPONENTIDS_H
#define COMPONENTIDS_H

#include <iostream>
#include <stdlib.h>

//ROS
#include <ros/ros.h>
#include <std_msgs/String.h>

//#include <sstream>
#include "XmlRpc.h"


namespace elektron_ids {

class ComponentIDS
{
public:
    ComponentIDS();

    bool detectInterception(const std::string &topic, XmlRpc::XmlRpcValue & publishers);
    bool detectFabrication(const std::string &topic, XmlRpc::XmlRpcValue & subscribers);
    void detectInterruption(const std::string& topic);
    void warn(const std::string& msg);
    void on_working();

    XmlRpc::XmlRpcValue getSystemState();
    XmlRpc::XmlRpcValue getTopicType(const std::string& topic);
    XmlRpc::XmlRpcValue getURI(const std::string& node_name);
    XmlRpc::XmlRpcValue getParam(const std::string& param_name);
    XmlRpc::XmlRpcValue getPubsName(const std::string& topic, XmlRpc::XmlRpcValue & publishers);
    XmlRpc::XmlRpcValue getSubsName(const std::string& topic, XmlRpc::XmlRpcValue & subscribers);
    bool isAuthorizated(const std::string& node_name,const std::string& topic_name, bool sub);
    bool killNode(const std::string& node);

private:
    // publisher
    ros::Publisher warn_pub_;

    // parameters
    XmlRpc::XmlRpcValue par_IP_;
    std::string camera_image_;

    bool hasProperIP(const std::string& node_name);
    bool isOnWhiteList(const std::string& node_name, const std::string& topic_name, bool sub);

    // gray list
    std::vector<std::string> grayList;
    bool isOnGrayList(const std::string& node);
    void addToGrayList(const std::string& node);
};


}; /* namespace elektron_ids */


#endif //COMPONENTIDS_H
