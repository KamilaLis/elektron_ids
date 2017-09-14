#include <fstream>
#include <ros/ros.h>

#include "elektron_ids/ComponentIDS.h"


std::string saveNodeWhiteList(XmlRpc::XmlRpcValue system_state, bool sub)
{
    std::string list;
    for(int i=0; i<system_state[sub].size(); ++i)
    {
        std::string topic = system_state[sub][i][0];
        //std::string size = std::to_string(system_state[0][i][1].size());
        std::string line = "  "+topic.substr(1) + ": [";//+size+"\n";
        for(int x=0; x<system_state[sub][i][1].size(); ++x)
        {
            line = line + "'"+ static_cast<std::string>(system_state[sub][i][1][x]) +"',";
        }
        list =  list + line + "]\n";
    }
    return list;
}



int main(int argc, char* argv[]) {
    
    ros::init(argc, argv, "component_ids");
    ros::NodeHandle n;

    elektron_ids::ComponentIDS component_ids;

    if(static_cast<std::string>(argv[1])=="save")
    {
    // saving state to file
        std::ofstream myfile;
        myfile.open ("/home/klis/STUDIA/catkin_ws/src/elektron_ids/config/example.yaml");

        XmlRpc::XmlRpcValue system_state = component_ids.getSystemState();

        myfile << "publishers:\n";
        std::string list = saveNodeWhiteList(system_state, false);
        myfile << list;

        myfile << "\nsubscribers:\n";
        list = saveNodeWhiteList(system_state, true);
        myfile << list;

        myfile.close();
    }
    else
    {
        ROS_INFO("ELEKTRON_IDS STARTED !!");

        // ROS loop
        ros::Rate rate(20.0);

        while (ros::ok())
        {
            component_ids.on_working();
            rate.sleep();
        }

    }//else

}//main


