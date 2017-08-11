#include "elektron_ids/ComponentIDS.h"

namespace elektron_ids {

ComponentIDS::ComponentIDS()
{
    this->par_pubs_ = getParam("publishers");
    this->par_subs_ = getParam("subscribers");
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
    //XmlRpc::XmlRpcValue payload = getSystemState();
    //XmlRpc::XmlRpcValue & subscribers = system_state[1];
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
    //XmlRpc::XmlRpcValue & publishers = system_state[0];
    std::vector<XmlRpc::XmlRpcValue> pub;
    /*
        pub: [topic_name,[pub1, pub2, ...]]
    */
    for(int i=0; i<publishers.size(); ++i){
        if(publishers[i][0]==topic) pub.push_back(publishers[i]);
    }
    return pub[0][1];
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
bool ComponentIDS::isAuthorizated(const std::string& node_name)
{
    std::string URI = getURI(node_name);
    std::size_t pos1 = URI.find("//");
    std::string ip_port = URI.substr(pos1+2);
    std::size_t pos2 = ip_port.find(":");
    std::string ip = ip_port.substr(0,pos2);

    if(ip.find(".") == std::string::npos){
        std::string command = "getent hosts "+ip;
        std::string address = exec(command.c_str());
        std:size_t pos3 = address.find("       ");
        ip = address.substr(0,pos3);
    }
    //TODO:
    //substr: __pos (which is 1) > this->size() (which is 0)
    for(int i=0; i<this->par_pubs_.size(); ++i){
        std::string pub_ip = par_pubs_[i];        
        if(pub_ip==ip) return true;
    }
    return false;
}


/*
*  ON WORKING
*/
void ComponentIDS::on_working()
{
    // pobierz listę wszystkich topicow (stan systemu?), na ktore ktos cos publikuje
    XmlRpc::XmlRpcValue system_state = getSystemState();
    // mozna by sprawdzic czy cos sie zmienilo od ostatniego sprawdzenia
    // jesli nie to wyjsc, jesli tak kontynuuj
     // dla każdego sprawdz czy ma upowaznione pubsy i subsy
    for(int i=0; i<system_state[0].size(); ++i){
        std::string topic = system_state[0][i][0];
        //ROS_INFO("TOPIC: %s", topic.c_str());
        XmlRpc::XmlRpcValue & publishers = system_state[0];
        detectFabrication(topic, publishers);
        XmlRpc::XmlRpcValue & subscribers = system_state[1];
        detectInterception(topic, subscribers);
    }
}


//  Check if topic has only allowed subscribers
void ComponentIDS::detectInterception(const std::string& topic,
                             XmlRpc::XmlRpcValue & subscribers)
{
    XmlRpc::XmlRpcValue sub = getSubsName(topic, subscribers);
    if(sub.getType()== XmlRpc::XmlRpcValue::TypeArray){
        for (int s=0; s<sub.size(); ++s){
            std::string node = sub[s];
            //ROS_INFO("* node: %s", node.c_str());
            if(!isAuthorizated(node)){
                ROS_WARN("Unauthorizated node %s subscribe data from %s", node.c_str(), topic.c_str());
                // kill that node
                std::string command = "read -p 'Would you like to kill it? y/n: \n' command";
                std::string response = exec(command.c_str());
                ROS_INFO("response: %s", response.c_str());
                //if(response=="y" || response=="Y"){
                //command = "rosnode kill " + node;
                //system(command.c_str());
                //}
            }
        }
    }
}


// Check if topic has only allowed publishers
void ComponentIDS::detectFabrication(const std::string& topic,
                             XmlRpc::XmlRpcValue & publishers)
{
    XmlRpc::XmlRpcValue pub = getPubsName(topic, publishers);
    if(pub.size()>0){
        for (int p=0; p<pub.size(); ++p){
            std::string node = pub[p];
            if(!isAuthorizated(node)){
                ROS_WARN("Unauthorizated node %s publish data on %s", node.c_str(), topic.c_str());
                // kill that node
                std::string command = "echo 'Would you like to kill it? y/n: '";
                std::string response;
                system(command.c_str());
                std::cin>>response;
                if(response=="y" || response=="Y"){
                    command = "rosnode kill " + node;
                    system(command.c_str());
                }
            }
        }
    }
}

}; /* namespace elektron_ids */

