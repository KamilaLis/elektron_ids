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
    //ros::Timer timer;

public:

    bool done_;
    int count_;

    EchoCallback(const std::string& topic):
        done_(false),
        count_(0)
    {
        ros::NodeHandle n;
        sub_ = n.subscribe(topic, 1000, &EchoCallback::imageCallback, this);
        //timer = n.createTimer(ros::Duration(2.0), &EchoCallback::timerCallback, this);
    }


    void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
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

/*    void timerCallback(const ros::TimerEvent& e){
        if(this->count_==0 && !this->done_)
        { //nothing recived in 2 seconds

        }
    }*/
};
};//namespace


#endif //ECHOCALLBACK_H