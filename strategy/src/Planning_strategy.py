#!/usr/bin/env python
# -*- coding: UTF-8 -*-

"""Use to generate arm task and run."""

import os
import sys
import copy
from math import degrees

import time
import rospy
from std_msgs.msg import Bool, Int32, String, Empty
from arm_control  import ArmTask, SuctionTask

import yaml
import rospkg
rospack = rospkg.RosPack()
g_path2package = rospack.get_path('dope')
from geometry_msgs.msg import PoseStamped

class Multi_subscriber():
    def __init__(self,_model,_type):
        self.model = _model
        self.feedback = _type()

    def sub_cp(self,data):
        self.feedback=data

    def get_data(self):
        return self.feedback

class stockingTask:
    def __init__(self, _name = '/robotis',_params=None):
        """Initial object."""
        self.name = _name
        self.arm = ArmTask(self.name + '_arm')
        self.params = _params

        self.pos   = [0, 0, 0]
        self.euler = [0, 0, 0]
        self.phi   = 0
        
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
    def testing(self):
        for model in self.model_list:
            showoff = self.sub_cb[model].get_data()
            print(showoff)
    #==============================================================
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
            a=1+1
            # left.process()
            left.testing()
        except rospy.ROSInterruptException:
            print('error')
            pass
        rate.sleep()

