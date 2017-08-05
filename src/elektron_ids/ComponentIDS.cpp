#include "elektron_ids/ComponentIDS.h"

namespace elektron_ids {

ComponentIDS::ComponentIDS(){

}

void ComponentIDS::on_working(){
    preventFabrication("/chatter");
    //preventInterception("/rosout");
}

/*
*  Check if topic has only allowed subscribers
*/
void ComponentIDS::preventInterception(const std::string& topic){
    //TODO: liczba dozwolonych topicow z pliku konfiguracyjnego
    ROS_INFO("%s(%d subscribers)",topic.c_str(), int(ros::TopicManager::instance()->getNumSubscribers(topic)));
}

/*
*  Check if topic has only allowed publishers
*/
void ComponentIDS::preventFabrication(const std::string& topic){
    ROS_INFO("%s(%d publisher)", topic.c_str(), int(ros::TopicManager::instance()->getNumPublishers(topic)));
}

}; /* namespace elektron_ids */
