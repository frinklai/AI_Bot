#!/usr/bin/env python

"""Use to generate arm task and run."""

import os
import sys
import copy
from math import degrees

import time
import rospy
from std_msgs.msg import Bool, Int32
from arm_control  import ArmTask, SuctionTask
from yolo_v3.msg  import ROI_array  
from yolo_v3.msg  import ROI  
from comm_stm32   import Gripper

count   = 0
box_cnt = 1

PICKORDER = 0
SPEED_L     = 100
# ===========================
idle        = 0
busy        = 1
initPose    = 2
back_home   = 3
pose1       = 4         # IK FAILED TIMES = 1 (No solution 1 & 2 !!!)
pose2       = 5
pose3       = 6
pose4       = 7     
pose5       = 8
pose6       = 9
pose7       = 10
pose8       = 11
pose9       = 12
pose10      = 13
pose11      = 14
pose12      = 15
pose13      = 16
pose14      = 17
pose15      = 18
x = 0
y = 0

class stockingTask:
    def __init__(self, _name = '/robotis'):
        """Initial object."""
        self.en_sim = False
        # if len(sys.argv) >= 2:
        #     print(type(sys.argv[1]))
        #     if sys.argv[1] == 'True':
        #         rospy.set_param('self.en_sim', sys.argv[1])
        #         self.en_sim = rospy.get_param('self.en_sim')
        self.name = _name
        self.state = initPose
        self.nowState = initPose 
        self.nextState = idle
        self.reGripCnt = 0
        self.arm = ArmTask(self.name + '_arm')
        self.pos   = [0, 0, 0]
        self.euler = [0, 0, 0]
        self.phi   = 0
        self.sucAngle = 0
        self.is_send_cmd = False
        self.finish =  False
        self.iter=0
        if self.name == 'left':
            self.is_right = -1
            self.speed = SPEED_L
    
    def move_arm(self, pos, euler, phi):
        self.arm.ikMove(mode= 'p2p', pos = pos, euler = euler, phi = phi) 
        self.is_send_cmd = True
        while(self.arm.is_busy or self.is_send_cmd == True):
            self.is_send_cmd = False
            pass
        

    def process(self):
        if self.arm.is_stop:
            self.finish =  True                  
            return  

        elif self.state == idle:
            if(self.finish):
                pass
            else:
                print('Mission Complete !!!')  
                self.finish =  True

        elif self.state == busy:
            if self.arm.is_busy:
                return
            else:
                self.state    = self.nextState
                self.nowState = self.nextState
                return  

        elif self.state == initPose:
            print('self.state == initPose')
            self.state = busy
            self.nextState = pose1
            self.arm.ikMove(mode= 'p2p', pos = [-0.3, 0.6, -0.45], euler = [0,0,0], phi=0)  

        elif self.state == back_home:
            print('self.state == back_home')
            self.state = busy
            self.nextState = idle
            self.nextState = pose1
            self.arm.back_home() 
            self.iter+=1
            print('================ iter =====================', self.iter) 

        # ============================================================================================

        elif self.state == pose1:
            print('self.state == pose1')
            self.state = busy
            self.nextState = pose2
            self.arm.ikMove(mode= 'p2p', pos = [0.3, 0.6, -0.55], euler = [0,0,0], phi=0)        

        elif self.state == pose2:
            print('self.state == pose2')
            self.state = busy
            self.nextState = pose3
            self.arm.ikMove(mode= 'p2p', pos = [0.1, 0.2, -0.45], euler = [0,0,0], phi=0)  

        elif self.state == pose3:
            print('self.state == pose3')
            self.state = busy
            self.nextState = pose4
            self.arm.ikMove(mode= 'p2p', pos = [0.1, 0.2, -0.6], euler = [0,0,0], phi=0)  

        # ============================================================================================

        elif self.state == pose4:
            print('self.state == pose4')
            self.state = busy
            self.nextState = pose5
            self.arm.ikMove(mode= 'p2p', pos = [0.4, 0.3, -0.2], euler = [0,0,0], phi=0)   

        elif self.state == pose5:
            print('self.state == pose5')
            self.state = busy
            self.nextState = pose6
            self.arm.ikMove(mode= 'p2p', pos = [0.2, 0.3, -0.5], euler = [0,0,0], phi=0)  

        elif self.state == pose6:
            print('self.state == pose6')
            self.state = busy
            self.nextState = pose7
            self.arm.ikMove(mode= 'p2p', pos = [-0.15, 0.3, -0.45], euler = [0,0,0], phi=0)  

        # ============================================================================================

        elif self.state == pose7:
            print('self.state == pose7')
            self.state = busy
            self.nextState = pose8
            self.arm.ikMove(mode= 'p2p', pos = [0.2, 0.5, -0.55], euler = [0,0,0], phi=0)  

        elif self.state == pose8:
            print('self.state == pose8')
            self.state = busy
            self.nextState = pose9
            self.arm.ikMove(mode= 'p2p', pos = [0.2, 0.2, -0.4], euler = [0,0,0], phi=0)  

        elif self.state == pose9:
            print('self.state == pose9')
            self.state = busy
            self.nextState = pose10
            self.arm.ikMove(mode= 'p2p', pos = [0.2, 0, -0.55], euler = [0,0,0], phi=0)  
        # ============================================================================================

        elif self.state == pose10:
            print('self.state == pose10')
            self.state = busy
            self.nextState = pose11
            self.arm.ikMove(mode= 'p2p', pos = [0.2, 0, -0.45], euler = [0,0,0], phi=0)  

        elif self.state == pose11:
            print('self.state == pose11')
            self.state = busy
            self.nextState = pose12
            self.arm.ikMove(mode= 'p2p', pos = [0.3, 0.4, -0.3], euler = [0,0,0], phi=0)  

        elif self.state == pose12:
            print('self.state == pose12')
            self.state = busy
            self.nextState = pose13
            self.arm.ikMove(mode= 'p2p', pos = [0.015, 0.4, -0.3], euler = [0,0,0], phi=0)  

        # ============================================================================================

        elif self.state == pose13:
            print('self.state == pose13')
            self.state = busy
            self.nextState = pose14
            self.arm.ikMove(mode= 'p2p', pos = [0.4, 0.4, -0.4], euler = [0,0,0], phi=0)  

        elif self.state == pose14:
            print('self.state == pose14')
            self.state = busy
            self.nextState = pose15
            self.arm.ikMove(mode= 'p2p', pos = [0.4, 0.4, -0.45], euler = [0,0,0], phi=0)  

        elif self.state == pose15:
            print('self.state == pose15')
            self.state = busy
            self.nextState = back_home
            self.arm.ikMove(mode= 'p2p', pos = [0.4, 0.4, -0.35], euler = [0,0,0], phi=0)  
        # ============================================================================================
        else:
            print("UNKNOW STATE!!!\n")




if __name__ == '__main__':
    rospy.init_node('ik_test', anonymous=True)
    
    left  = stockingTask('left')       # Set up left arm controller
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        try:
            left.process()
        except rospy.ROSInterruptException:
            print('error')
            pass
            break
        rate.sleep()

