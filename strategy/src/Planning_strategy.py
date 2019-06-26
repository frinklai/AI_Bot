#!/usr/bin/env python
# -*- coding: UTF-8 -*-

"""Use to generate arm task and run."""

import os
import sys
import copy
import math
from math import degrees
import numpy as np

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
initPose        = 2             #(1st movement)
M_Target_Top    = 3             #(2nd movement)
FM_Tool         = 4             #(3rd movement)
Enable_Sucker   = 5             #(4th movement)
RM_Close_Target = 6             #(5th movement)
RM_Leave_Target = 7             #(6th movement)
M_Answer        = 8             #(7th movement)
RM_Put_down     = 9             #(8th movement)
Disable_Sucker  = 10            #(9th movement)
M_Pull_up       = 11            #(10th movement)

SPEED_L     = 30

target_object = {}
#===============================================
class Multi_subscriber():
    def __init__(self,_model,_type):
        self.model = _model
        self.feedback = _type()
        self.flag = False

    def sub_cp(self,data):
        self.feedback = data
        self.feedback.pose.position.x=data.pose.position.x/100
        self.feedback.pose.position.y=data.pose.position.y/100
        self.feedback.pose.position.z=data.pose.position.z/100
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
        self.limit_min_x,self.limit_max_x = -0.3,0.3
        self.limit_min_y,self.limit_max_y = 0.2,0.7
        self.limit_min_z,self.limit_max_z = -0.88,-0.78
        
        #set the transformer and rotation for coordination
        self.trans_x =  -0.555
        self.trans_y =  +0.42
        self.trans_z = 0
        self.rot_x_axis = -70-90
        self.rot_y_axis = 0
        self.rot_z_axis = -90
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
                world_corr = self.transform_2_world(self.sub_cb[model].get_data().pose.position)
                if self.limit_min_x < world_corr.x <=self.limit_max_x:
                    if self.limit_min_y < world_corr.y <=self.limit_max_y:
                        if self.limit_min_z < world_corr.z <=self.limit_max_z:
                            if index == 0 :
                                index = index+1
                                target["name"] = model
                                target["location"] = self.sub_cb[model].get_data().pose.position
                                target["world_location"] = world_corr
                                target["pose"] = self.sub_cb[model].get_data().pose.orientation
                                target["distance"] = self.threeD_distance(self.ori_point,self.sub_cb[model].get_data().pose.position)
                            if self.threeD_distance(self.ori_point,self.sub_cb[model].get_data().pose.position) < target["distance"]:
                                target["name"] = model
                                target["location"] = self.sub_cb[model].get_data().pose.position
                                target["world_location"] = world_corr
                                target["pose"] = self.sub_cb[model].get_data().pose.orientation
                                target["distance"] = self.threeD_distance(self.ori_point,self.sub_cb[model].get_data().pose.position)
            self.sub_cb[model].shut_flag()
        if target == {}:
            return None
        return target

    def threeD_distance(self,A_point,B_point):
        return math.sqrt(  (pow(abs(A_point.x-B_point.x),2)) + (pow(abs(A_point.y-B_point.y),2)) + (pow(abs(A_point.z-B_point.z),2))  )
    
    def transform_2_world(self,location):
        print("camera_coordination::"+str(location)+"\n")
        loca = np.array([[location.x],
                         [location.y],
                         [location.z],
                         [1]])
        trans = self.transform(self.trans_x,self.trans_y,self.trans_z)
        rota_x = self.Rotation('x',self.rot_x_axis)
        rota_y = self.Rotation('y',self.rot_y_axis)
        rota_z = self.Rotation('z',self.rot_z_axis)

        loca = np.dot(rota_x,loca)
        loca = np.dot(rota_y,loca)
        loca = np.dot(rota_z,loca)
        loca = np.dot(trans,loca)

        location.x = loca[0]
        location.y = loca[1]
        location.z = loca[2]
        print(location)
        return location
    
    def transform(self, x, y, z):
        return np.array([[1, 0, 0, x],
                         [0, 1, 0, y],
                         [0, 0, 1, z],
                         [0, 0, 0, 1]])
    
    def Rotation(self,axis, deg):
        deg = deg * math.pi/180
        if axis == 'x':
            return [[1              ,0             , 0            ,    0],
                    [0              ,math.cos(deg) ,-math.sin(deg),    0],
                    [0              ,math.sin(deg) , math.cos(deg),    0],
                    [0              ,0             , 0            ,    1]]
        if axis == 'y':
            return [[ math.cos(deg) ,0             , math.sin(deg),    0],
                    [ 0             ,1             , 0            ,    0],
                    [-math.sin(deg) ,0             , math.cos(deg),    0],
                    [0              ,0             , 0            ,    1]]
        if axis == 'z':
            return [[ math.cos(deg) ,-math.sin(deg), 0            ,    0],
                    [ math.sin(deg) , math.cos(deg), 0            ,    0],
                    [0              , 0            , 1            ,    0],
                    [0              , 0            , 0            ,    1]]
    #==============================================================
    def test_process(self):
        global target_object
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

        #(1st movement) Move Home point
        elif self.state == initPose:
            print('1st:self.state == initPose')
            self.state = busy
            self.arm.set_speed(self.speed)
            self.nextState = M_Target_Top
            self.pos   = [0.48, 0.25, -0.4]
            self.euler = [0, 0, 0]
            self.phi = 90
            self.arm.ikMove(mode= 'p2p', pos = self.pos, euler = self.euler, phi = self.phi)  

        #(2nd movement) Move Tartget Top Point
        elif self.state == M_Target_Top:
            print('2nd:self.state == M_Target_Top')
            target_object = self.choose_target()
            if target_object == None:
                print("No object detected")
                return
            print("selected {} model to pick".format(target_object["name"]))
            self.arm.set_speed(self.speed)
            self.state = busy
            self.nextState = FM_Tool
            self.pos   = [target_object["world_location"].x, target_object["world_location"].y, target_object["world_location"].z]
            self.euler = [0, 0, 90]
            self.phi = 90
            self.arm.ikMove(mode= 'p2p', pos = self.pos, euler = self.euler, phi = self.phi)

        #(3rd movement) Fine Tune Tool's position
        elif self.state == FM_Tool:
            print('3rd:self.state == FM_Tool')
            self.arm.set_speed(self.faster_speed)
            self.state = busy
            self.nextState = Enable_Sucker
            self.pos   = [target_object["world_location"].x, target_object["world_location"].y, target_object["world_location"].z]
            self.euler = [0, 0, 90]
            self.phi = 90
            self.quater = target_object["pose"]
            self.arm.ikMove(mode= 'p2p', pos = self.pos, euler = self.euler, phi = self.phi, quater = self.quater)

        #(4th movement) Enable Sucker
        elif self.state == Enable_Sucker:
            print('4th:self.state == Enable_Sucker')
            # self.suction.gripper_vaccum_on()
            self.arm.set_speed(self.faster_speed)
            self.state = busy
            self.nextState = RM_Close_Target
            self.pos   = [target_object["world_location"].x, target_object["world_location"].y, target_object["world_location"].z]
            self.euler = [0, 0, 90]
            self.phi = 90
            self.quater = target_object["pose"]
            self.arm.ikMove(mode= 'p2p', pos = self.pos, euler = self.euler, phi = self.phi, quater = self.quater)

        #(5th movement) Relative move close to target
        elif self.state == RM_Close_Target:
            print('5th:self.state == RM_Close_Target')
            self.arm.set_speed(self.faster_speed)
            self.state = busy
            self.nextState = RM_Leave_Target
            # ask need use tool coordinate
            self.pos   = [0.4, 0.42, -0.63]
            self.euler = [0, 0, 90]
            self.phi = 90
            self.arm.ikMove(mode= 'p2p', pos = self.pos, euler = self.euler, phi = self.phi)

        #(6th movement) Relative move far to target
        elif self.state == RM_Leave_Target:
            print('6th:self.state == RM_Leave_Target')
            self.arm.set_speed(self.faster_speed)
            self.state = busy
            self.nextState = M_Answer
            # ask need use tool coordinate
            self.pos   = [0.4, 0.42, -0.63]
            self.euler = [0, 0, 90]
            self.phi = 90
            self.arm.ikMove(mode= 'p2p', pos = self.pos, euler = self.euler, phi = self.phi)

        #(7th movement) Move Answer Place
        elif self.state == M_Answer:
            print('7th:self.state == M_Answer')
            self.arm.set_speed(self.faster_speed)
            self.state = busy
            self.nextState = RM_Put_down
            self.pos   = [0.4, 0.42, -0.63]
            self.euler = [0, 0, 90]
            self.phi = 90
            self.arm.ikMove(mode= 'p2p', pos = self.pos, euler = self.euler, phi = self.phi)

        #(8th movement) Move Put it down
        elif self.state == RM_Put_down:
            print('8th:self.state == RM_Put_down')
            self.arm.set_speed(self.faster_speed)
            self.state = busy
            self.nextState = Disable_Sucker
            self.pos   = [0.4, 0.42, -0.63]
            self.euler = [0, 0, 90]
            self.phi = 90
            self.arm.ikMove(mode= 'p2p', pos = self.pos, euler = self.euler, phi = self.phi)

        #(9th movement) Disable Sucker
        elif self.state == Disable_Sucker:
            print('9th:self.state == Disable_Sucker')
            # self.suction.gripper_vaccum_off()
            self.arm.set_speed(self.faster_speed)
            self.state = busy
            self.nextState = M_Pull_up
            self.pos   = [0.4, 0.42, -0.63]
            self.euler = [0, 0, 90]
            self.phi = 90
            self.arm.ikMove(mode= 'p2p', pos = self.pos, euler = self.euler, phi = self.phi)    
        
        #(10th movement) Move Pull up Tool
        elif self.state == M_Pull_up:
            print('10th:self.state == M_Pull_up')
            self.arm.set_speed(self.faster_speed)
            self.state = busy
            self.nextState = initPose
            self.pos   = [0.4, 0.42, -0.63]
            self.euler = [0, 0, 90]
            self.phi = 90
            self.arm.ikMove(mode= 'p2p', pos = self.pos, euler = self.euler, phi = self.phi)    

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
            left.test_process()
        except rospy.ROSInterruptException:
            print('error')
            pass
        rate.sleep()

