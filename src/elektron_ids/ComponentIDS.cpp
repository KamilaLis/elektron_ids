#include "elektron_ids/ComponentIDS.h"
#include "elektron_ids/EchoCallback.h"

namespace elektron_ids {

ComponentIDS::ComponentIDS()
{
    // Read local parameters
    ros::NodeHandle local_nh("~");
    
    par_IP_ = getParam("while_list");
    par_subscribers_ = getParam("subscribers");
    par_publishers_ = getParam("publishers");
    camera_image_ = static_cast<std::string>(getParam("camera_image"));

    ROS_ASSERT(par_subscribers_.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_ASSERT(par_publishers_.getType() == XmlRpc::XmlRpcValue::TypeStruct);

    // just to make sure that all nodes are running
    std::string command = "rosnode list";
    std::string pid_str = exec(command.c_str());

    nodes_ = getNodes(getSystemState());
    //for(auto elem : nodes_)
    //    ROS_INFO("%s, %d",(elem.first).c_str(), elem.second);

    // advertise
    manager.initPublisher(local_nh);
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

// TODO: niezbyt to eleganckie
// Get the PID of node 
int ComponentIDS::getPid(const std::string& node)
{
    std::string command = "rosnode info "+node+" 2>/dev/null | grep Pid| cut -d' ' -f2";
    std::string pid_str = exec(command.c_str());  
    return atoi(pid_str.c_str());
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


// Get all nodes
std::map<std::string,int> ComponentIDS::getNodes(XmlRpc::XmlRpcValue system_state)
{
    std::set<std::string> node_set;
    for(int i=0; i<system_state.size(); ++i)
    {
        for(int j = 0; j < system_state[i].size(); ++j)
        {
            XmlRpc::XmlRpcValue val = system_state[i][j][1];
            for (int k = 0; k < val.size(); ++k)
            {
                std::string name = system_state[i][j][1][k];
                node_set.insert(name);                
            }
        }
    }
    std::map<std::string,int> nodes;
    for(auto n : node_set)
    {
        nodes.insert(std::pair<std::string,int>(n,getPid(n)));
    }
    return nodes;
}


// Get return value from system()
std::string ComponentIDS::exec(const char* cmd) {
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
    std::string ip_port = URI.substr(pos1+2);
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

// UNUSED
// Check if node's PID is the same as in nodes_ map
bool ComponentIDS::hasProperPid(const std::string& node)
{
    return nodes_[node] == getPid(node);
}


// Get params from struct as array
XmlRpc::XmlRpcValue getListFromPar(XmlRpc::XmlRpcValue par, 
                    const std::string& topic_name)
{
    std::string str = "/";
    //ROS_INFO("topic_name: %s", topic_name.c_str());
    std::size_t pos = topic_name.find(str);
    if(pos!=std::string::npos)
    { //there is '/' in topic name
        return getListFromPar(par[topic_name.substr(0,pos)],topic_name.substr(pos+str.length()));
    }
    else
    {
        XmlRpc::XmlRpcValue list (par[topic_name]);
        //ROS_INFO_STREAM("list: " << list);
        return list;
    }
}


// Check if node is on list of sub/pub of topic
bool ComponentIDS::isOnWhiteList(const std::string& node_name, const std::string& topic_name, XmlRpc::XmlRpcValue par)
{
//    try{
        //if(sub) list = getParam("subscribers/"+topic_name);
        //else list = getParam("publishers/"+topic_name);
        XmlRpc::XmlRpcValue nodes = getListFromPar(par,topic_name.substr(1));
        if(nodes.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {// if 
            for(int i=0; i<nodes.size(); ++i)
            {
                if(node_name==static_cast<std::string>(nodes[i])) return true;
            }
        }
        //ROS_INFO("Node %s not found in whitelist as pub/sub of topic %s", node_name.c_str(), topic_name.c_str());
        return false;
//    }
//    catch (const std::runtime_error& error)
//    {
//        ROS_WARN("%s not found in whitelist", topic_name.c_str());
//    }
}

// 
bool ComponentIDS::isAuthorizated(const std::string& node_name,const std::string& topic_name, XmlRpc::XmlRpcValue par)
{
    return (hasProperIP(node_name) && isOnWhiteList(node_name,topic_name,par)) || isOnGrayList(node_name);
}

// Check if user wanted this node to stay 
bool ComponentIDS::isOnGrayList(const std::string& node)
{
    return std::find(grayList.begin(), grayList.end(), node)!= grayList.end();
}


void ComponentIDS::addToGrayList(const std::string& node)
{
    grayList.push_back(node);
    nodes_[node] = getPid(node);
}

/* -----------------------------------------------
*       ON WORKING
* -----------------------------------------------*/
void ComponentIDS::on_working()
{
    XmlRpc::XmlRpcValue system_state = getSystemState();
    std::map<std::string,int> current_nodes = getNodes(system_state);

    detectNodeSubstitution(current_nodes);
    detectInterruption(this->camera_image_);

    for(int i=0; i<system_state[0].size(); ++i){
        std::string topic = system_state[0][i][0];
        XmlRpc::XmlRpcValue & publishers = system_state[0];
        if(detectFabrication(topic, publishers)) break;
        XmlRpc::XmlRpcValue & subscribers = system_state[1];
        if(detectInterception(topic, subscribers)) break;
    }
}

// ----------------------------------------
// FIX: czy blokować przerwania do czasu otrzymania odpowiedzi?
// a może odczekać jakiś czas i jeśli nic nie dostanę to podjąć jakąś decyzję?
// ----------------------------------------
// Kill node if needed, return true if killed
bool ComponentIDS::killNode(const std::string& node)
{
    std::string command;// = "echo 'Would you like to kill it? y/n: '";
    std::string response;
    //system(command.c_str());
    ROS_INFO("Would you like to kill it? y/n:");
    std::cin>>response;
    if(response=="y" || response=="Y"){
        command = "rosnode kill " + node;
        system(command.c_str());
        std::string info = "Node "+ node +" killed by operator.";
        ROS_INFO("%s",info.c_str());
        manager.ok(info);
        return true;
    }
    // add node to gray list
    addToGrayList(node);
    std::string info = "Node "+ node +" accepted by operator. Added to grayList.";
    ROS_INFO("%s",info.c_str());
    manager.ok(info);
    return false;
}

// Check if nodes from nodes_ are working with right pid
void ComponentIDS::detectNodeSubstitution(std::map<std::string,int> current_nodes)
{
    std::string msg = std::string();
    for(auto elem : nodes_)
    {
        std::string node = elem.first;
        int pid = elem.second;

        if(current_nodes.find(node) == current_nodes.end())
        { // node from nodes_ is not alive
            ROS_WARN("Node %s seems to be killed",(node).c_str());
            manager.error("Node "+ node +" seems to be killed");
        }
        else if(current_nodes[node] != pid)
        { // node has wrong pid 
            ROS_WARN("Registered new node [pid:%d] instead of %s [pid:%d]",current_nodes[node],(node).c_str(),pid);
            manager.error("Registered new node instead of "+ node +".");
            killNode(node);
        }
    }
}


//  Check if topic has only allowed subscribers (detect new nodes)
bool ComponentIDS::detectInterception(const std::string& topic,
                             XmlRpc::XmlRpcValue & subscribers)
{
    //ROS_INFO("detectInterception");
    XmlRpc::XmlRpcValue sub = getSubsName(topic, subscribers);
    if(sub.getType()== XmlRpc::XmlRpcValue::TypeArray){
        for (int s=0; s<sub.size(); ++s){
            std::string node = sub[s];
            //ROS_INFO("subs:");
            if(!isAuthorizated(node,topic,par_subscribers_)){
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
    //ROS_INFO_STREAM("pub "<<pub);
    if(pub.getType()== XmlRpc::XmlRpcValue::TypeArray){
        for (int p=0; p<pub.size(); ++p){
            //ROS_INFO("size:%d",pub.size());
            std::string node = pub[p];
            //ROS_INFO("pubs");
            if(!isAuthorizated(node,topic,par_publishers_)){
                ROS_WARN("Unauthorizated node %s publish data on %s", node.c_str(), topic.c_str());
                if(killNode(node)) return true;
            }
        }
    }
    return false;
}

// ----------------------------------------
// FIX: to opóźnia każdą pętlę o 2 sek.!!!
// ----------------------------------------
// Check if camera image is published
void ComponentIDS::detectInterruption(const std::string& topic)
{   //print warning if nothing received for two seconds
    bool use_sim_time = true;
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
            manager.warn("Potential interruption: no messages received on "+topic);
        }
    }

}

// ----------------------------------------
// FIX: czy to ma w ogóle sens?? 
// jak obsłużyć najlepiej alarm od visual_odom?
// gdzie odróżniać modyfikację od fabrykacji? czy w ogóle to robić? mam jak zareagować na fabrykację? 
// ----------------------------------------
// React on warning 
void ComponentIDS::alertCallback(const diagnostic_msgs::DiagnosticStatus::ConstPtr& msg)
{
    if (msg->level == 2)
    { // it is an error alert
        ROS_ERROR("%s: %s", (msg->name).c_str(),(msg->message).c_str());
        std::string request = msg->values[0].key;
        std::string topic = msg->values[0].value;
        manager_api::Message key = manager_api::getEnumForText(request);
        switch (key){
            case(manager_api::Message::killPublisher): do_killPublisher(topic);break;
            case(manager_api::Message::killSubsriber): do_killSubsriber(topic);break;
            case(manager_api::Message::rosTime): ;break;
        }

    }
}

void ComponentIDS::do_killPublisher(const std::string &topic)
{
    XmlRpc::XmlRpcValue system_state = getSystemState();
    XmlRpc::XmlRpcValue pub = getPubsName(topic, system_state[0]);
    if(pub.getType()== XmlRpc::XmlRpcValue::TypeArray){
        for (int p=0; p<pub.size(); ++p){
            killNode(pub[p]);
        }
    }
}

void ComponentIDS::do_killSubsriber(const std::string &topic)
{
    XmlRpc::XmlRpcValue system_state = getSystemState();
    XmlRpc::XmlRpcValue sub = getSubsName(topic, system_state[1]);
    if(sub.getType()== XmlRpc::XmlRpcValue::TypeArray){
        for (int s=0; s<sub.size(); ++s){
            killNode(sub[s]);
        }
    }
}

}; /* namespace elektron_ids */

