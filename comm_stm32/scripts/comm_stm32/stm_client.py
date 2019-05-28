#!/usr/bin/env python     
# Every Python ROS Node will have the command above, which declaration at the top.
# The first line makes sure your script is executed as a Python script.
from comm_stm32.srv import *
from std_msgs.msg import UInt8
import rospy
import sys

class Gripper:
    def __init__(self):
        self.CATCH_LOOSEN_WAIT_TIME  = 3
        self.BASE_ROTATION_WAIT_TIME = 2
        self.TASK_DELAY_TIME = 0.5

    def Send_Gripper_Command(self, cmd_id):
        num = self.Conver_cmd_to_id(cmd_id)
        try:
            control_gripper = rospy.ServiceProxy('gripper_service', gripper_cmd)    
            resp1 = control_gripper(int(num))         
            if(cmd_id != 'Stop'):
                rospy.loginfo("[client] Send command success, wait for gripper do task" ); 
                if((cmd_id=='2D_mode')or(cmd_id=='3D_mode')):
                    rospy.sleep(self.BASE_ROTATION_WAIT_TIME)    # base rotation, wait 2s
                else:
                    rospy.sleep(self.CATCH_LOOSEN_WAIT_TIME)     # catch or loosen, wait 3s

                self.Send_Gripper_Command('Stop')        # focus stop 
                rospy.sleep(self.TASK_DELAY_TIME)   
        

        except rospy.ServiceException, e:
            rospy.logerr("[client] Service call failed" ); 
            print "Service call failed: %s"%e
                         

    def Connect_to_Server(self):
        rospy.wait_for_service('gripper_service')   

    def Conver_cmd_to_id(self, cmd):
        # print("aaaaaa\n")
        if   (cmd == 'Catch_all'):      return 1
        elif (cmd == 'Loosen_all'):     return 2

        elif (cmd == 'Catch_no1'):      return 3
        elif (cmd == 'Catch_no2'):      return 4
        elif (cmd == 'Catch_no3'):      return 5

        elif (cmd == 'Loosen_no1'):     return 6
        elif (cmd == 'Loosen_no2'):     return 7
        elif (cmd == 'Loosen_no3'):     return 8
        
        elif (cmd == 'Stop'):           return 0
        elif (cmd == 'Disable'):        return 9      # now useless
        elif (cmd == '3D_mode'):        return 10
        elif (cmd == '2D_mode'):        return 11
        elif (cmd == '2D_catch'):       return 12
        elif (cmd == '2D_loosen'):      return 13
        
        

if __name__ == '__main__':
    rospy.init_node('stm_client_', anonymous=True)
    
    gripper = Gripper()
    gripper.Connect_to_Server()
    
    gripper.Send_Gripper_Command('Catch_all')

    print('Mission Complete!!!')


        
