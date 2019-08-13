#!/usr/bin/env python
# -*- coding: UTF-8 -*-

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
from speech.msg   import SR

count   = 0

PICKORDER = 0
SPEED_L     = 30

idle                    = 0
busy                    = 1
initPose                = 2             #(第一個動作)
goto                    = 3
down                    = 4             #第一段下去
up                      = 5
wait_img_pos            = 6             #等影像資料
move_to_obj             = 7             #移到物品上方
down_sec                = 8             #第二段下去         
wait_speech_recognition = 9
pickObject              = 10

x = 0
y = 0

class stockingTask:
    def __init__(self, _name = '/robotis'):
        """Initial object."""
        self.en_sim = False
        if len(sys.argv) >= 2:
            print(type(sys.argv[1]))
            if sys.argv[1] == 'True':
                rospy.set_param('self.en_sim', sys.argv[1])
                self.en_sim = rospy.get_param('self.en_sim')
        self.img_data = ROI()
        self.img_data_list = []
        self.check = SR()
        self.speech_obj_name = ' '
        self.check.speech_check = 0
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
        self.No_Object_count = 0
        self.obj_name = ''
        self.standby_safeFlag = True 
        self.close_box = False             
        self.finish_pos = False
        self.mode_2D = False
        self.finish = False
        if self.name == 'left':
            self.is_right = -1
            self.speed = SPEED_L
            self.faster_speed = 40
        self.init_pub_sub()
        if self.en_sim:
            self.suction = SuctionTask(self.name + '_gazebo')
        else:
            self.suction = SuctionTask(self.name)
            rospy.on_shutdown(self.suction.gripper_vaccum_off)
    
    def init_pub_sub(self):
        rospy.Subscriber('/speech/check',     SR       , self.get_speech_info)
        rospy.Subscriber('/object/ROI_array', ROI_array, self.get_obj_info_cb)

    def process(self):
        if self.arm.is_stop:
            self.finish =  True
            print('!!! Robot is stop !!!')                         
            return  

        if self.state == idle:
            if self.finish:
                return    

        if self.state == busy:
            if self.arm.is_busy:
                return
            else:
                self.state    = self.nextState
                self.nowState = self.nextState
                return  

        #(第一個動作)
        elif self.state == initPose:
            print('self.state == initPose')
            self.state = busy
            self.arm.set_speed(self.speed)
            self.pos   = [0.4, 0.5, -0.3]
            self.euler = [0, 0, 0]
            self.phi = 0
            self.nextState = wait_speech_recognition
            self.arm.ikMove(mode= 'p2p', pos = self.pos, euler = self.euler, phi = self.phi) 
            self.check.speech_check = 0 

        elif self.state == wait_speech_recognition:
            print('self.state == wait_speech_recognition')
            self.state = busy
            while self.check.speech_check == 0:
                print('wait speech_check')
            if self.check.speech_check != 0:
                if self.check.speech_check == 1:
                    self.speech_obj_name = 'bottle'
                elif self.check.speech_check == 2:
                    self.speech_obj_name = 'cellphone'
                elif self.check.speech_check == 3:
                    self.speech_obj_name = 'mouse'
                self.nextState = move_to_obj
                self.No_Object_count = 0
                print('catch ' + self.speech_obj_name)
                time.sleep(1)
            else:
                print('wait speech_check')
                self.nextState = wait_speech_recognition
                

        elif self.state == wait_img_pos:        # wait_img_pos
            print('self.state == wait_img_pos')
            self.state = busy

            if(len(self.img_data_list)!=0):
                for i in range(len(self.img_data_list)):
                    if (self.img_data.min_x > 638) and (self.img_data.min_y > 156) and (self.img_data.Max_x < 1188) and (self.img_data.Max_y < 811):
                        if self.speech_obj_name == self.img_data.object_name:
                            print("----- stra detect object_" + str(i) + " ----- ")
                            print("object_name = " + str(self.img_data.object_name))
                            print("score = " + str(self.img_data.score))
                            print("min_xy = [ " +  str( [self.img_data.min_x, self.img_data.min_y] ) +  " ]" )
                            print("max_xy = [ " +  str( [self.img_data.Max_x, self.img_data.Max_y] ) +  " ]" )
                            self.nextState = move_to_obj
                            self.obj_name = self.img_data.object_name
                            x = (self.img_data.min_x + self.img_data.Max_x)/2
                            y = (self.img_data.min_y + self.img_data.Max_y)/2    
                            time.sleep(1)
                        else:
                            print('not this object')
                            self.No_Object_count += 1
                    else:
                        print('object over range!!')
            else:
                print('no object!!!')
                self.nextState = wait_img_pos
                self.No_Object_count += 1
            if self.No_Object_count == 100:      #判斷桌上沒有此物件
                self.nextState = initPose
                self.No_Object_count = 0
                print('沒有此物件')
                time.sleep(2)
                # self.close_box = True                          

        # # 防止影像狀態機怪怪的空狀態(必定接在wait_img_pos後面)
        # elif self.state == goto:              
        #     print('self.state == goto')
        #     self.state = busy
        #     if self.standby_safeFlag == True:
        #         self.nextState = standby_safe_point
        #     else:
        #         self.nextState = move_standby
        

        elif self.state == move_to_obj:          
            global x
            global y
            print('self.state == move_to_obj')
            self.arm.set_speed(self.speed)
            
            posX , posY = self.Image_transform(x, y)

            self.state = busy
            self.nextState = down
            self.pos   = [round(posX, 4), round(posY, 4), -0.45]
            # self.pos   = [0.3, 0.4, -0.45]
            print(self.pos)
            self.euler = [0, 0, 0]
            self.phi = 0
            self.arm.ikMove(mode= 'p2p', pos = self.pos, euler = self.euler, phi = self.phi)    

        elif self.state == down:               # down
            print('self.state == down')
            self.arm.set_speed(self.faster_speed)
            self.state = busy
            self.nextState = down_sec
            self.pos   = [0, 0, -0.1]
            self.arm.relative_move_pose(mode='p2p', pos=self.pos)
            rospy.sleep(.1)

        elif self.state == down_sec:
            print('self.state == down_sec')
            self.arm.set_speed(self.speed)
            self.state = busy
            self.nextState = up  
            self.pos  = [0, 0, -0.1]
            self.arm.relative_move_pose(mode='p2p', pos=self.pos)

        elif self.state == pickObject:
            print('self.state == pickObject')
            self.state = busy
            self.nextState = up 
            self.suction.gripper_vaccum_on()
            time.sleep(1)
            # self.suction.gripper_vaccum_off()
            
            # rospy.sleep(1)
            # if 'lunchbox' in objectName[self.pickList]:
            #     self.arm.set_speed(30)
            # else:
            #     self.arm.set_speed(3)
            # self.arm.noa_move_suction('line', suction_angle=0, n=0, o=0, a=0.03)
            # rospy.sleep(.1)

        elif self.state == up:               # up
            print('self.state == up')
            self.arm.set_speed(self.faster_speed)
            self.state = busy
            self.nextState = initPose
            self.pos   = [0, 0, 0.2]
            self.arm.relative_move_pose(mode='p2p', pos=self.pos)  

    #-------------------------座標轉換------------------------------------------------------
    def Image_transform(self, Camera_Image_X, Camera_Image_Y):
        Arm_posX = (866 - Camera_Image_Y)*0.000889 - 0.4795     
        Arm_posY = (974 - Camera_Image_X)*0.000889 + 0.3960     
        return Arm_posX, Arm_posY


    #-------------------------------------------------------------------------------
    def get_speech_info(self, data):
        self.check = SR()
        self.check.speech_check = data.speech_check
        return self.check

    def get_obj_info_cb(self, data):
        self.img_data = ROI()
        self.img_data_list = data.ROI_list
        #print("Detected object number = " + str(len(data.ROI_list)))
        for i in range(len(data.ROI_list)):
            #if (data.ROI_list[i].min_x > 400) and (data.ROI_list[i].min_y) > 20 and (data.ROI_list[i].Max_x < 1440) and (data.ROI_list[i].Max_y < 990):
            self.img_data.object_name = data.ROI_list[i].object_name
            self.img_data.score       = data.ROI_list[i].score
            self.img_data.min_x = data.ROI_list[i].min_x
            self.img_data.min_y = data.ROI_list[i].min_y
            self.img_data.Max_x = data.ROI_list[i].Max_x
            self.img_data.Max_y = data.ROI_list[i].Max_y
            return self.img_data


if __name__ == '__main__':
    rospy.init_node('gripperasd', anonymous=True)
    
    left  = stockingTask('left')       # Set up left arm controller
    rospy.sleep(.3)

    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        try:
            left.process()
        except rospy.ROSInterruptException:
            print('error')
            pass
            break
        rate.sleep()
