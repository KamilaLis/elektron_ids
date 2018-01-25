#ifndef COMPONENTIDS_H
#define COMPONENTIDS_H

#include <iostream>
#include <stdlib.h>

//ROS
#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

#include "XmlRpc.h"
#include "manager_api/AlertManagement.h"
#include "manager_api/Manager.h"


namespace elektron_ids {

class ComponentIDS
{
public:
    ComponentIDS();
    void on_working();

    void detectNodeSubstitution(std::map<std::string,int> current_nodes);
    bool detectInterception(const std::string &topic, XmlRpc::XmlRpcValue & publishers);
    bool detectFabrication(const std::string &topic, XmlRpc::XmlRpcValue & subscribers);
    void detectInterruption(const std::string& topic);

    // system state
    XmlRpc::XmlRpcValue getSystemState();
    std::map<std::string,int> getNodes(XmlRpc::XmlRpcValue system_state);
    XmlRpc::XmlRpcValue getTopicType(const std::string& topic);
    XmlRpc::XmlRpcValue getURI(const std::string& node_name);
    int getPid(const std::string& node);
    
    XmlRpc::XmlRpcValue getParam(const std::string& param_name);
    XmlRpc::XmlRpcValue getPubsName(const std::string& topic, XmlRpc::XmlRpcValue & publishers);
    XmlRpc::XmlRpcValue getSubsName(const std::string& topic, XmlRpc::XmlRpcValue & subscribers);
    
    bool isAuthorizated(const std::string& node_name,const std::string& topic_name,XmlRpc::XmlRpcValue par);
    bool killNode(const std::string& node);
    void kill(const std::string& node);
    void do_killPublisher(const std::string &topic);
    void do_killSubsriber(const std::string &topic, const std::string &module_name);

    // alerts
    // void alertCallback(const diagnostic_msgs::DiagnosticStatus::ConstPtr& msg);
    bool handleAlert(manager_api::Manager::Request  &req, manager_api::Manager::Response &res);

private:
    //publisher (for hypothetical usage with diagnostics package (http://wiki.ros.org/diagnostics))
    manager_api::AlertManagement manager_pub_ = manager_api::AlertManagement("elektron_ids");
    
    // parameters
    XmlRpc::XmlRpcValue par_IP_;
    XmlRpc::XmlRpcValue par_subscribers_;
    XmlRpc::XmlRpcValue par_publishers_;
    std::string camera_image_;

    // nodes and its pids
    std::map<std::string,int> nodes_;

    std::string exec(const char* cmd);
    bool hasProperIP(const std::string& node_name);
    bool hasProperPid(const std::string& node);
    bool isOnWhiteList(const std::string& node_name, const std::string& topic_name, XmlRpc::XmlRpcValue par);
    
    // gray list
    std::vector<std::string> grayList;
    bool isOnGrayList(const std::string& node);
    void addToGrayList(const std::string& node);
};


}; /* namespace elektron_ids */


#endif //COMPONENTIDS_H
