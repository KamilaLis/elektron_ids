#include <fstream>

#include <signal.h>
#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>

#include "elektron_ids/ComponentIDS.h"


std::string saveNodeWhiteList(XmlRpc::XmlRpcValue system_state, bool sub)
{
    std::string list;
    for(int i=0; i<system_state[sub].size(); ++i)
    {
        std::string topic = system_state[sub][i][0];
        std::string line = "  "+topic.substr(1) + ": [";
        for(int x=0; x<system_state[sub][i][1].size(); ++x)
        {
            line = line + "'"+ static_cast<std::string>(system_state[sub][i][1][x]) +"',";
        }
        list =  list + line + "]\n";
    }
    return list;
}

// Replacement "shutdown" XMLRPC callback
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  int num_params = 0;
  if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
    num_params = params.size();
  if (num_params > 1)
  {
    std::string reason = params[1];
    ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
  }

  result = ros::xmlrpc::responseInt(1, "", 0);
}



int main(int argc, char* argv[]) {
    
    ros::init(argc, argv, "elektron_ids");

    // override XMLRPC shutdown (used by rosnode kill)
    ros::XMLRPCManager::instance()->unbind("shutdown");
    ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);

    elektron_ids::ComponentIDS component_ids;

    // handle alerts from other modules
    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("manager", &elektron_ids::ComponentIDS::handleAlert, &component_ids);

    if(static_cast<std::string>(argv[1])=="true")
    {
    // saving state to file (nodes)
        std::ofstream myfile;
        std::string filename = argv[2];
        myfile.open(filename);
        if(!myfile.is_open()) ROS_INFO("Could not open configuration file!");

        XmlRpc::XmlRpcValue system_state = component_ids.getSystemState();

        myfile << "publishers:\n";
        std::string list = saveNodeWhiteList(system_state, false);
        myfile << list;

        myfile << "\nsubscribers:\n";
        list = saveNodeWhiteList(system_state, true);
        myfile << list;

        myfile.close();
        ROS_INFO("White list sucessfully saved!");
    }
    else
    { // start ids
        ROS_INFO("ELEKTRON_IDS STARTED !!");

        // ROS loop
        ros::Rate rate(20.0);

        while (ros::ok())
        {
            component_ids.on_working();
            ros::spinOnce();
            rate.sleep();
        }
    }
}


