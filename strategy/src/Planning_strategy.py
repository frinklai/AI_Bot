#!/usr/bin/env python
# -*- coding: UTF-8 -*-

"""Use to generate arm task and run."""

import os
import sys
import copy
import math
from math import degrees

import time
import rospy
from std_msgs.msg import Bool, Int32, String, Empty
from arm_control  import ArmTask, SuctionTask

import yaml
import rospkg
rospack = rospkg.RosPack()
g_path2package = rospack.get_path('dope')
from geometry_msgs.msg import PoseStamped,Point

#=================state define==================
idle            = 0
busy            = 1
initPose        = 2             #(第一個動作)



SPEED_L     = 30
#===============================================
class Multi_subscriber():
    def __init__(self,_model,_type):
        self.model = _model
        self.feedback = _type()
        self.flag = False

    def sub_cp(self,data):
        print("get data from subscriber")
        self.feedback=data
        self.flag = True

    def get_data(self):
        if self.flag == True:
            # print("return data")
            return self.feedback
        else:
            # print("return None")
            return None
    
    def shut_flag(self):
        self.feedback=None
        self.flag = False

class stockingTask:
    def __init__(self, _name = '/robotis',_params=None):
        """Initial object."""
        self.en_sim = False
        if len(sys.argv) >= 2:
            if sys.argv[1] == 'True':
                rospy.set_param('self.en_sim', sys.argv[1])
                self.en_sim = rospy.get_param('self.en_sim')
        
        self.name = _name
        self.state = initPose
        self.nowState = initPose 
        self.nextState = idle

        self.params = _params
        self.pos   = [0, 0, 0]
        self.euler = [0, 0, 0]
        self.phi   = 0


        self.ori_point = Point()
        self.ori_point.x, self.ori_point.y, self.ori_point.z = 0.0,0.0,0.0
        self.limit_min_x,self.limit_max_x = -10,10
        self.limit_min_y,self.limit_max_y = -10,10
        self.limit_min_z,self.limit_max_z = 80,90
        # self.target={}

        self.arm = ArmTask(self.name + '_arm')
        if self.name == 'left':
            self.is_right = -1
            self.speed = SPEED_L
            self.faster_speed = 40
        if self.en_sim:
            self.suction = SuctionTask(self.name + '_gazebo')
        else:
            self.suction = SuctionTask(self.name)

        
        self.model_list = params['weights']
        self.sub_cb={}
        self.sub={}
        self.init_sub()

        
    
    #======================initial subscriber======================
    def init_sub(self):
        for model in self.model_list:
            self.create_sub('/{}/pose_{}'.format(params['topic_publishing'], model),model)


    def create_sub(self,topic,model):
        self.sub_cb[model]=Multi_subscriber(model,PoseStamped)
        self.sub[model] = \
            rospy.Subscriber(
                topic, 
                PoseStamped, 
                self.sub_cb[model].sub_cp
            )
    #==============================================================
    def choose_target(self):
        index = 0
        target={}
        for model in self.model_list:
            if self.sub_cb[model].get_data() is not None:
                if self.limit_min_x < self.sub_cb[model].get_data().pose.position.x <=self.limit_max_x:
                    if self.limit_min_y < self.sub_cb[model].get_data().pose.position.y <=self.limit_max_y:
                        if self.limit_min_z < self.sub_cb[model].get_data().pose.position.z <=self.limit_max_z:
                            if index == 0 :
                                target["name"] = model
                                target["location"] = self.sub_cb[model].get_data().pose.position
                                target["pose"] = self.sub_cb[model].get_data().pose.orientation
                                target["distance"] = self.threeD_distance(self.ori_point,self.sub_cb[model].get_data().pose.position)
                            if self.threeD_distance(self.ori_point,self.sub_cb[model].get_data().pose.position) < target["distance"]:
                                target["name"] = model
                                target["location"] = self.sub_cb[model].get_data().pose.position
                                target["pose"] = self.sub_cb[model].get_data().pose.orientation
                                target["distance"] = self.threeD_distance(self.ori_point,self.sub_cb[model].get_data().pose.position)
            index = index+1
            self.sub_cb[model].shut_flag()
            if target == {}:
                return None
            return target

    def threeD_distance(self,A_point,B_point):
        return math.sqrt(  (pow(abs(A_point.x-B_point.x),2)) + (pow(abs(A_point.y-B_point.y),2)) + (pow(abs(A_point.z-B_point.z),2))  )
    #==============================================================
    def test_process(self):
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
            #self.nextState = wait_img_pos
            self.arm.set_speed(self.speed)
            # self.pos   = [0.4, 0.5, -0.3]
            # self.euler = [0, 0, 0]
            # self.phi = 0
            self.nextState = standby_open
            self.pos   = [0.48, 0.25, -0.4]
            self.euler = [0, 0, 0]
            self.phi = 90
            self.arm.ikMove(mode= 'p2p', pos = self.pos, euler = self.euler, phi = self.phi)  

        #(第二個動作)準備開抽屜的位置
        elif self.state == standby_open:
            print('self.state == standby_open')
            self.arm.set_speed(self.faster_speed)
            self.state = busy
            self.nextState = drag_grasp
            self.pos   = [0.4, 0.42, -0.63]
            self.euler = [0, 0, 90]
            self.phi = 90
            self.arm.ikMove(mode= 'p2p', pos = self.pos, euler = self.euler, phi = self.phi)          

    #-------------------------座標轉換------------------------------------------------------
    def Image_transform(self, Camera_Image_X, Camera_Image_Y):
        Arm_posX = (866 - Camera_Image_Y)*0.000889 - 0.4795     
        Arm_posY = (974 - Camera_Image_X)*0.000889 + 0.3960     
        return Arm_posX, Arm_posY
    #-------------------------------------------------------------------------------

if __name__ == '__main__':
    #input yaml part
    if len(sys.argv) > 1:
        config_name = sys.argv[1]
    else:
        config_name = "config_pose.yaml"
    
    params = None
    yaml_path = g_path2package + '/config/{}'.format(config_name)
    with open(yaml_path, 'r') as stream:
        try:
            print("Loading DOPE parameters from '{}'...".format(yaml_path))
            params = yaml.load(stream)
            print('    Parameters loaded.')
        except yaml.YAMLError as exc:
            print(exc)

    #ros part
    rospy.init_node('Planning', anonymous=True)
    
    left  = stockingTask('left',params)       # Set up left arm controller
    rospy.sleep(.3)

    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        try:
            # left.process()
            left.test_process
        except rospy.ROSInterruptException:
            print('error')
            pass
        rate.sleep()

