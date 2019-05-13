#include "ros/ros.h" 
#include "comm_stm32/gripper_cmd.h" // #include "package_name/service_file_name.h"  

using namespace std;
int main(int argc, char **argv) 
{ 
    ros::init(argc, argv, "stm_client");    // node name
    if (argc != 2) 
    { 
        ROS_INFO("usage: rosrun comm_stm32 stm_client [val]"); 
        return 1; 
    } 

    ros::NodeHandle n; 
    ros::service::waitForService("gripper_service");
    ros::ServiceClient client = n.serviceClient<comm_stm32::gripper_cmd>("gripper_service"); //comment1
    comm_stm32::gripper_cmd srv;        // Declare a service file object
    srv.request.val = atoll(argv[1]); 

    if (client.call(srv)) 
    { 
        if(srv.response.success)
            ROS_INFO("[client] Send command success." ); 
        else
            ROS_ERROR_STREAM("[client] Send command failed." ); 
    } 
    else 
    { 
        ROS_ERROR("Failed to call stm_service"); 
        return 1; 
    } 

    return 0; 
}

// ===== comment1 =====
// Declare a service-client
// n.serviceClient<pkg_name::srv_file_name>("service-name");
// service-name is service's name, just like topic-name, not srv's file name