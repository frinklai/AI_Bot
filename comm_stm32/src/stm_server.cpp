#include "ros/ros.h" 
#include "comm_stm32/gripper_cmd.h" 
#include <serial/serial.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
serial::Serial ser;
#define	sBUFFERSIZE	1               //send    buffer size 
#define	rBUFFERSIZE	1               //receive buffer size 
unsigned char s_buffer[sBUFFERSIZE];//send    buffer
unsigned char r_buffer[rBUFFERSIZE];//receive buffer

bool send_cmd(unsigned char number)
{ 
    memset(s_buffer, 0, sizeof(s_buffer));
    s_buffer[0] = number;
    ser.write(s_buffer, sBUFFERSIZE);
    try
    {
        ROS_INFO("[Write] Server send cmd = %d", s_buffer[0]); 
        return true;
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Failed to write data!");
        return false; 
    }
    
} 
  
bool service_request(comm_stm32::gripper_cmd::Request  &req, comm_stm32::gripper_cmd::Response &res) 
{ 
    // ROS_INFO("Request cmd = %d",req.val); 
    res.success = send_cmd(req.val); 
    return res.success; 
} 

int main(int argc, char **argv) 
{ 
    ros::init(argc, argv, "stm_server"); 
    ros::NodeHandle n; 

    ros::ServiceServer service = n.advertiseService("gripper_service", service_request); 
    ROS_INFO("The stm Service is Ready."); 

    // ros::spin();  

    
    // Init Port
    try
    {
        ser.setPort("/dev/ttyUSB1");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
        ROS_INFO("Ready to initialize serial port..."); 
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port!");
        return -1;
    }
    

    if(ser.isOpen())
	{
        ROS_INFO_STREAM("===========================");
        ROS_INFO_STREAM("Open Serial Port Success!!!");
        ROS_INFO_STREAM("===========================");
    }
	else
	{
        ROS_ERROR_STREAM("Failed to open port!!!");
        return -1;
    }
    
    
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        ros::spinOnce();
        if(ser.available())
        {
            try
            {
                ser.read(r_buffer, rBUFFERSIZE);
                ROS_INFO("[Read] Receieve gripper feedback = %d", r_buffer[0]); 
                memset(r_buffer, 0, rBUFFERSIZE);
            }
            catch (serial::IOException& e)
            {
                ROS_ERROR_STREAM("Failed to read data!!!");
            }
			

            
        }
		loop_rate.sleep();
    }
    return 0; 
} 

