#ifndef ECHOCALLBACK_H
#define ECHOCALLBACK_H

#include <iostream>
#include <stdlib.h>

//ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>

namespace echo
{

class EchoCallback
{
private:
    ros::Subscriber sub_;

public:

    bool done_;
    int count_;

    EchoCallback(const std::string& topic):
        done_(false),
        count_(0)
    {
        ros::NodeHandle n;
        sub_ = n.subscribe(topic, 1000, &EchoCallback::callback_image, this);
    }


    void callback_image(const sensor_msgs::Image::ConstPtr& msg)
    {
        if(msg)
        {
            this->count_ += 1;
            this->done_=true;
        } 

    }

    void callback_control(const geometry_msgs::Twist::ConstPtr& msg)
    {
        if(msg)
        {
            this->count_ += 1;
            this->done_=true;
        } 
    }


};
};//namespace


#endif //ECHOCALLBACK_H