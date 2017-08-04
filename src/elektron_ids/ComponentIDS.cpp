#include "elektron_ids/ComponentIDS.h"

namespace elektron_ids {

ComponentIDS::ComponentIDS(){
    manager_= new ros::TopicManager();
}

void ComponentIDS::on_working(){
    preventFabrication("/robot1/component_simulation/cmd_vel");
    preventInterception("/robot1/camera_1/rgb/image_raw");
}

/*
*  Check if topic has only one subscriber
*/
void ComponentIDS::preventInterception(const std::string& topic){
    //TODO: liczba dozwolonych topicow z pliku konfiguracyjnego
    if(manager_->getNumSubscribers(topic)>1){
        ROS_INFO("This topic has more than one subscriber!");
    }
}

/*
*  Check if topic has only one publisher
*/
void ComponentIDS::preventFabrication(const std::string& topic){
    //TODO: liczba dozwolonych topicow z pliku konfiguracyjnego
    if(manager_->getNumPublishers(topic)>1){
        ROS_INFO("This topic has more than one publisher!");
    }
}

}; /* namespace elektron_ids */
