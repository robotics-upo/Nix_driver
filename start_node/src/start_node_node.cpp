#include <ros/ros.h>
#include <std_srvs/Trigger.h>

std::string system_call("rosrun idmind_navigation odometry_node.py &");

bool startCb(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &rep)
{
    std::system(system_call.c_str());
    rep.message = "runned node";
    rep.success = true;
    return true;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "start_node_node");

    ros::NodeHandle n("~");

    //n.param("system_call", system_call, (std::string) "rosrun idmind_navigation odometry_node &");

    ros::ServiceServer server = n.advertiseService("start_node", startCb);

    ros::spin();
    return 0;
}
