#include "ros/ros.h"
#include "manipulator_h_base_module_msgs/GetAdaptiveLine.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_Line_phi");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<manipulator_h_base_module_msgs::GetAdaptiveLine>("/robotis/base/get_adaptive_line");
    manipulator_h_base_module_msgs::GetAdaptiveLine srv;
    srv.request.command = atoll(argv[1]);
    if(client.call(srv))
    {
        ROS_INFO("Sending Command");
    }else
    {
        ROS_ERROR("Failed");
    }
    return 0;
}