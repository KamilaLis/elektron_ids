#include "elektron_ids/ComponentIDS.h"
#include "elektron_ids/EchoCallback.h"

namespace elektron_ids {

ComponentIDS::ComponentIDS()
{
    // Read local parameters
    ros::NodeHandle local_nh("~");

    par_IP_ = getParam("while_list");
    camera_image_ = static_cast<std::string>(getParam("camera_image"));

    // advertise
    warn_pub_ = local_nh.advertise<std_msgs::String>("warnings", 1);
}

// Retrieve list representation of system state (i.e. publishers, subscribers, and services).
XmlRpc::XmlRpcValue ComponentIDS::getSystemState()
{
    XmlRpc::XmlRpcValue request("/component_ids");
    XmlRpc::XmlRpcValue response;
    XmlRpc::XmlRpcValue payload;

    bool success = ros::master::execute("getSystemState", request,
    response, payload, false);

    ROS_ASSERT(payload.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(payload.size() == 3);
    return payload;
}

// Get type of messages published on topic
XmlRpc::XmlRpcValue ComponentIDS::getTopicType(const std::string& topic)
{
    XmlRpc::XmlRpcValue request("/component_ids");
    XmlRpc::XmlRpcValue response;
    XmlRpc::XmlRpcValue payload;

    bool success = ros::master::execute("getTopicTypes", request,
    response, payload, false);
    // payload: [[topicName, topicType],...]
    for (int i=0; i<payload.size(); ++i)
    {
        if (payload[i][0]== topic) return payload[i][1];
    }
}

// Get XML-RPC URI of node
XmlRpc::XmlRpcValue ComponentIDS::getURI(const std::string& node_name)
{
    XmlRpc::XmlRpcValue request;
    request[0] = "/component_ids";
    request[1] = node_name;
    XmlRpc::XmlRpcValue response;
    XmlRpc::XmlRpcValue payload;

    bool success = ros::master::execute("lookupNode", request,
    response, payload, false);

    return payload;
}


// Get parametrs from configuration file
XmlRpc::XmlRpcValue ComponentIDS::getParam(const std::string& param_name)
{
    ros::NodeHandle nh("~");
    std::string name_ = param_name;

    XmlRpc::XmlRpcValue value;

    if (!nh.hasParam(name_))
    {
        std::string message("Parameter \"");
        message.append(name_);
        message.append("\" not defined.");
        throw std::runtime_error(message.c_str());       
    }
    
    nh.getParam(name_, value);
    return value;
}


// Get list of subscribers (names) of given topic
XmlRpc::XmlRpcValue ComponentIDS::getSubsName(const std::string& topic, 
                                                XmlRpc::XmlRpcValue & subscribers)
{
    std::vector<XmlRpc::XmlRpcValue> sub;
    for(int i=0; i<subscribers.size(); ++i){
        if(subscribers[i][0]==topic){
            sub.push_back(subscribers[i]);
        }
    }
    if(!sub.empty()) return sub[0][1];
    XmlRpc::XmlRpcValue empty();
    return empty;
}


// Get list of publishers (names) of given topic
XmlRpc::XmlRpcValue ComponentIDS::getPubsName(const std::string& topic,
                                                XmlRpc::XmlRpcValue & publishers)
{
    std::vector<XmlRpc::XmlRpcValue> pub;
    /*
        pub: [topic_name,[pub1, pub2, ...]]
    */
    for(int i=0; i<publishers.size(); ++i){
        if(publishers[i][0]==topic) pub.push_back(publishers[i]);
    }
    if(!pub.empty()) return pub[0][1];
    XmlRpc::XmlRpcValue empty();
    return empty;
}


// Get return value from system()
std::string exec(const char* cmd) {
    char buffer[128];
    std::string result = "";
    FILE* pipe = popen(cmd, "r");
    if (!pipe) throw std::runtime_error("popen() failed!");
    try {
        while (!feof(pipe)) {
            if (fgets(buffer, 128, pipe) != NULL)
                result += buffer;
        }
    } catch (...) {
        pclose(pipe);
        throw;
    }
    pclose(pipe);
    return result;
}

// Check if node's IP is on white list
bool ComponentIDS::hasProperIP(const std::string& node_name)
{
    std::string URI = getURI(node_name);
    if (URI.size()<=7)
    {// connection just has been finished (returns "http://")
        return true;
    }
    std::size_t pos1 = URI.find("//");
    std::string ip_port = URI.substr(pos1+2);//fix string::npos.
    std::size_t pos2 = ip_port.find(":");
    std::string ip = ip_port.substr(0,pos2);
    
    if(ip.find(".") == std::string::npos)
    { // if it is host name instead of IP address
        std::string command = "getent hosts "+ip;
        std::string address = exec(command.c_str());
        std:size_t pos3 = address.find(" ");
        ip = address.substr(0,pos3);
    }
    for(int i=0; i<this->par_IP_.size(); ++i){
        std::string pub_ip = par_IP_[i];        
        if(pub_ip==ip) return true;
    }
    ROS_INFO("NODE %s IP: %s", node_name.c_str(), ip.c_str());
    return false;
}

// Check if node is on list of sub/pub of topic
bool ComponentIDS::isOnWhiteList(const std::string& node_name, const std::string& topic_name, bool sub)
{
    XmlRpc::XmlRpcValue list;
    try{
        if(sub) list = getParam("subscribers/"+topic_name);
        else list = getParam("publishers/"+topic_name);
        for(int i=0; i<list.size(); ++i)
        {
            if(node_name==static_cast<std::string>(list[i])) return true;
        }
        return false;
    }
    catch (const std::runtime_error& error)
    {
        ROS_WARN("%s not found in whitelist", topic_name.c_str());
    }
}

// 
bool ComponentIDS::isAuthorizated(const std::string& node_name,const std::string& topic_name, bool sub)
{
    if((hasProperIP(node_name) && isOnWhiteList(node_name,topic_name,sub)) || isOnGrayList(node_name)){
        return true;
    } 
    return false;
}

// Check if user wanted this node to stay 
bool ComponentIDS::isOnGrayList(const std::string& node)
{
    return std::find(grayList.begin(), grayList.end(), node)!= grayList.end();
}


void ComponentIDS::addToGrayList(const std::string& node)
{
    grayList.push_back(node);
}

/*
*  ON WORKING
*/
void ComponentIDS::on_working()
{
    // pobierz listę wszystkich topicow (stan systemu), na ktore ktos cos publikuje
    XmlRpc::XmlRpcValue system_state = getSystemState();
    // mozna by sprawdzic czy cos sie zmienilo od ostatniego sprawdzenia
    // jesli nie to wyjsc, jesli tak kontynuuj
    // dla każdego sprawdz czy ma upowaznione pubsy i subsy
    for(int i=0; i<system_state[0].size(); ++i){
        std::string topic = system_state[0][i][0];
        //ROS_INFO("TOPIC: %s", topic.c_str());
        XmlRpc::XmlRpcValue & publishers = system_state[0];
        if(detectFabrication(topic, publishers)) break;
        XmlRpc::XmlRpcValue & subscribers = system_state[1];
        if(detectInterception(topic, subscribers)) break;
    }
    detectInterruption(this->camera_image_);//"/head_camera_rgb/image_raw"); 
}

// Kill node if needed, return true if killed
bool ComponentIDS::killNode(const std::string& node)
{
    std::string command = "echo 'Would you like to kill it? y/n: '";
    std::string response;
    system(command.c_str());
    std::cin>>response;
    if(response=="y" || response=="Y"){
        command = "rosnode kill " + node;
        system(command.c_str());
        warn("Killed node: "+node);
        return true;
    }
    // add node to gray list
    addToGrayList(node);
    warn("Node "+node+" added to grayList");
    return false;
}

//  Check if topic has only allowed subscribers
bool ComponentIDS::detectInterception(const std::string& topic,
                             XmlRpc::XmlRpcValue & subscribers)
{
    XmlRpc::XmlRpcValue sub = getSubsName(topic, subscribers);
    if(sub.getType()== XmlRpc::XmlRpcValue::TypeArray){
        for (int s=0; s<sub.size(); ++s){
            std::string node = sub[s];
            //ROS_INFO("* node: %s", node.c_str());
            if(!isAuthorizated(node,topic, true)){
                ROS_WARN("Unauthorizated node %s subscribe data from %s", node.c_str(), topic.c_str());
                // kill that node
                if(killNode(node)) return true;
            }
        }
    }
    return false;
}


// Check if topic has only allowed publishers
bool ComponentIDS::detectFabrication(const std::string& topic,
                             XmlRpc::XmlRpcValue & publishers)
{
    XmlRpc::XmlRpcValue pub = getPubsName(topic, publishers);
    if(pub.getType()== XmlRpc::XmlRpcValue::TypeArray){
        for (int p=0; p<pub.size(); ++p){
            std::string node = pub[p];
            if(!isAuthorizated(node,topic, false)){
                ROS_WARN("Unauthorizated node %s publish data on %s", node.c_str(), topic.c_str());
                if(killNode(node)) return true;
            }
        }
    }
    return false;
}

// Check if camera image is published
void ComponentIDS::detectInterruption(const std::string& topic)
{
    //print warning if nothing received for two seconds
    //bool use_sim_time = getParam("use_sim_time");
    bool use_sim_time = true;
    //std::string type = getTopicType(topic);
    //ROS_INFO("Topic type: %s", static_cast<std::string>(getTopicType(topic)).c_str());
    boost::shared_ptr<echo::EchoCallback> callback_echo(new echo::EchoCallback(topic));
    if(use_sim_time)
    {
        ros::Time timeout = ros::Time::now() + ros::Duration(2.0);
        while(ros::Time::now()<timeout)// && callback_echo.count_==0 && !callback_echo.done_)
        {
            ros::Duration(0.1).sleep();
            ros::spinOnce();
        }
        if(callback_echo->count_==0 && !callback_echo->done_)
        {
            ROS_WARN("Potential interruption: no messages received on %s",topic.c_str());
            warn("Potential interruption: no messages received on "+topic);
        }
    }

}

// Send warning to warn_pub
void ComponentIDS::warn(const std::string& msg)
{
    std_msgs::String message;
    message.data = msg.c_str();
    warn_pub_.publish(message);
}

}; /* namespace elektron_ids */

