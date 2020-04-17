#include <ros/ros.h>
#include <std_srvs/Trigger.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "start_node_node_client");

    ros::NodeHandle n("~");

    ros::ServiceClient  client = n.serviceClient<std_srvs::Trigger>("/start_node_node/start_node");
    client.waitForExistence();
    std_srvs::Trigger trig;

    client.call(trig);
    if(trig.response.success){
        exit(0);
    }

    ros::spin();

    return 0;
}
