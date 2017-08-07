#include "elektron_ids/ComponentIDS.h"

namespace elektron_ids {

ComponentIDS::ComponentIDS(){
    
}
// Retrieve list representation of system state (i.e. publishers, subscribers, and services).
XmlRpc::XmlRpcValue ComponentIDS::getSystemState(){
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
XmlRpc::XmlRpcValue ComponentIDS::getURI(const std::string& node_name){
    XmlRpc::XmlRpcValue request;
    request[0] = "/component_ids";
    request[1] = node_name;
    XmlRpc::XmlRpcValue response;
    XmlRpc::XmlRpcValue payload;
    bool success = ros::master::execute("lookupNode", request,
    response, payload, false);
    return payload;
}

// Get list of subscribers (names) of given topic
XmlRpc::XmlRpcValue ComponentIDS::getSubsName(const std::string& topic){
    XmlRpc::XmlRpcValue payload = getSystemState();
    XmlRpc::XmlRpcValue & subscribers = payload[1];
    std::vector<XmlRpc::XmlRpcValue> sub;
    for(int i=0; i<subscribers.size(); ++i){
        std::string topic_name = subscribers[i][0];
        if(topic_name==topic){
            sub.push_back(subscribers[i]);
        }
    }
    return sub[0][1];
}

// Get list of publishers (names) of given topic
XmlRpc::XmlRpcValue ComponentIDS::getPubsName(const std::string& topic){
    XmlRpc::XmlRpcValue payload = getSystemState();
    XmlRpc::XmlRpcValue & publishers = payload[0];
    std::vector<XmlRpc::XmlRpcValue> pub;
    /*
        pub: [topic_name,[pub1, pub2, ...]]
    */
    for(int i=0; i<publishers.size(); ++i){
        if(publishers[i][0]==topic) pub.push_back(publishers[i]);
    }
    return pub[0][1];
}

/*
*  ON WORKING
*/
void ComponentIDS::on_working(){
    //preventFabrication("/chatter");
    //preventInterception("/rosout");
    XmlRpc::XmlRpcValue pub = getPubsName("/rosout");
    for(int i=0; i<pub.size(); ++i){
        std::string name = pub[i];
        std::string uri = getURI(name);
        ROS_INFO("i: %d, pub: %s", i,name.c_str());// static_cast<std::string>(pub[i]).c_str());
        ROS_INFO("URI: %s", uri.c_str());
    }
}

/*
*  Check if topic has only allowed subscribers
*/
void ComponentIDS::preventInterception(const std::string& topic){
    //TODO: liczba dozwolonych topicow z pliku konfiguracyjnego
    //ROS_INFO("%s(%d subscribers)",topic.c_str(), int(ros::TopicManager::instance()->getNumSubscribers(topic)));
    
}

/*
*  Check if topic has only allowed publishers
*/
void ComponentIDS::preventFabrication(const std::string& topic){
    
}

}; /* namespace elektron_ids */

