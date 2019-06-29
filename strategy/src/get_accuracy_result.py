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
import tf

import yaml
import rospkg
rospack = rospkg.RosPack()
g_path2package = rospack.get_path('dope')
from geometry_msgs.msg import PoseStamped,Point

import pandas as pd 
import os

#=================state define==================
idle            = 0
busy            = 1
initPose        = 2             #(1st movement)
Fixed_Pose    = 3             #(2nd movement)
Cul_match         = 4             #(3rd movement)
ADD_point   = 5             #(4th movement)
SAVE_data = 6             #(5th movement)
CALCULATE_accuracy = 7             #(6th movement)
END        = 8             #(7th movement)
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
        self.limit_min_x,self.limit_max_x = -0.4,0.07
        self.limit_min_y,self.limit_max_y = 0.268,0.7
        self.limit_min_z,self.limit_max_z = -0.88,-0.65
        
        #set the transformer and rotation for coordination
        self.trans_x =  -0.5750 
        self.trans_y =  +0.42 -0.02
        self.trans_z = 0
        self.rot_x_axis = -70-90+1
        self.rot_y_axis = 0
        self.rot_z_axis = -90 +3
        # self.target={}

        self.First_pose = [-0.3,0.3,-0.8530]
        self.movement_x = 0.028 *2
        self.movement_y = 0.021 *2
        self.movement_z = 0.03
        self.movement_x_cnt = 0
        self.movement_y_cnt = 0
        self.movement_z_cnt = 0

        self.wait_time = 0
        self.accuracy ={"point":[],"location_accuracy":[],"roll_accuracy":[],"pitch_accuracy":[],"yaw_accuracy":[]} 

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
        target={}
        for model in self.model_list:
            if self.sub_cb[model].get_data() is not None:
                world_corr = self.transform_2_world(self.sub_cb[model].get_data().pose.position)
                if self.limit_min_x < world_corr.x <=self.limit_max_x:
                    if self.limit_min_y < world_corr.y <=self.limit_max_y:
                        if self.limit_min_z < world_corr.z <=self.limit_max_z:
                            target["name"] = model
                            target["location"] = self.sub_cb[model].get_data().pose.position
                            target["world_location"] = world_corr
                            target["pose"] = self.sub_cb[model].get_data().pose.orientation
                            # print("AAA:"+str(target["pose"]))
                            target["world_pose"] = self.rota_2_world(self.sub_cb[model].get_data().pose.orientation)
                            # print("BBB:"+str(target["world_pose"]))
                            target["distance"] = self.threeD_distance(self.ori_point,self.sub_cb[model].get_data().pose.position)
            self.sub_cb[model].shut_flag()
        if target == {}:
            return None
        return target

    def threeD_distance(self,A_point,B_point):
        return math.sqrt(  (pow(abs(A_point.x-B_point.x),2)) + (pow(abs(A_point.y-B_point.y),2)) + (pow(abs(A_point.z-B_point.z),2))  )
    
    def rota_2_world(self,quaternion):
        # print(quaternion)
        quat = np.array([quaternion.x,
                         quaternion.y,
                         quaternion.z,
                         quaternion.w])

        trans = self.transform(self.trans_x,self.trans_y,self.trans_z)
        rota_x = self.Rotation('x',self.rot_x_axis)
        rota_y = self.Rotation('y',self.rot_y_axis)
        rota_z = self.Rotation('z',self.rot_z_axis)

        quat_matrix = rota_y
        # quat = np.dot(rota_y,quat )
        quat_matrix = np.dot(rota_x,quat_matrix )
        quat_matrix = np.dot(rota_z,quat_matrix )
        quat_matrix = np.dot(trans,quat_matrix)
        
        quat2world = tf.transformations.quaternion_from_matrix(quat_matrix)
        world_quat = tf.transformations.quaternion_multiply(quat2world,quat)
        # q_matix_eulr = tf.transformations.euler_from_quaternion(quat_matrix ,"rxyz") 
        # print("q_matix_eulr",np.multiply(q_matix_eulr,(180/math.pi)))
        # world_quat =quat_matrix
        

        quaternion.x = world_quat[0]
        quaternion.y = world_quat[1]
        quaternion.z = world_quat[2]
        quaternion.w = world_quat[3]

        return quaternion

    def transform_2_world(self,location):
        # print("camera_coordination:\n"+str(location)+"\n")
        loca = np.array([[location.x],
                         [location.y],
                         [location.z],
                         [1]])
        trans = self.transform(self.trans_x,self.trans_y,self.trans_z)
        rota_x = self.Rotation('x',self.rot_x_axis)
        rota_y = self.Rotation('y',self.rot_y_axis)
        rota_z = self.Rotation('z',self.rot_z_axis)

        loca = np.dot(rota_y,loca)
        loca = np.dot(rota_x,loca)
        loca = np.dot(rota_z,loca)
        loca = np.dot(trans,loca)

        location.x = loca[0]
        location.y = loca[1]
        location.z = loca[2]
        # print(location)
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
            self.suction.gripper_vaccum_on()
            print('==initPose==')
            self.state = busy
            # self.state = M_Target_Top
            self.arm.set_speed(self.speed)
            self.nextState = Fixed_Pose
            self.pos   = [-0.17, 0.4, -0.42-0.06]
            self.euler = [0, 0, 0]
            self.phi = 0
            self.arm.ikMove(mode= 'p2p', pos = self.pos, euler = self.euler, phi = self.phi)  

        #(2nd movement) Move Tartget Top Point
        elif self.state == Fixed_Pose:
            print('==Fixed_Pose==')
            self.state = busy
            # self.state = M_Target_Top
            self.arm.set_speed(self.speed)
            self.nextState = Cul_match
            print(self.movement_x_cnt)
            print(self.movement_y_cnt)
            print(self.movement_z_cnt)
            self.pos   = [self.First_pose[0] + (self.movement_x*self.movement_x_cnt),
                          self.First_pose[1] + (self.movement_y*self.movement_y_cnt),
                          self.First_pose[2] + (self.movement_z*self.movement_z_cnt)]
            print(self.pos)
            self.euler = [0, -30, 0]
            self.phi = 0
            self.arm.ikMove(mode= 'line', pos = self.pos, euler = self.euler, phi = self.phi)  
            self.wait_time = 0

        elif self.state == Cul_match:
            print('==Cul_match==')
            target_object = self.choose_target()
            
            if target_object == None :
                self.wait_time = self.wait_time + 1
                print(self.wait_time)
                if self.wait_time >= 200:
                    print("next")
                    self.state = ADD_point
                print("No object detected")
                return
                # self.accuracy["point"].append(self.pos)
                # self.accuracy["location_accuracy"].append(-1)
                # self.accuracy["roll_accuracy"].append(-1)
                # self.accuracy["ptich_accuracy"].append(-1)
                # self.accuracy["yaw_accuracy"].append(-1)
                 
            
            self.state = ADD_point
            l = Point()
            l.x = self.pos[0]
            l.y = self.pos[1]
            l.z = self.pos[2]
            q = [np.float64(target_object["world_pose"].x),
                 np.float64(target_object["world_pose"].y),
                 np.float64(target_object["world_pose"].z),
                 np.float64(target_object["world_pose"].w)]


            p_matrix = tf.transformations.projection_matrix((0,0,0),(0,0,1)) #projected on xy
            p_quat = tf.transformations.quaternion_from_matrix(p_matrix)
            world_quat = tf.transformations.quaternion_multiply(p_quat,q)
            p_euler = tf.transformations.euler_from_quaternion(world_quat)
            p_euler = np.multiply(p_euler,(180/math.pi))
            print("p_euler:",p_euler)
            # q_euler = tf.transformations.euler_from_quaternion(q)
            # q_euler = np.multiply(q_euler,(180/math.pi))
            # print("q_euler:",q_euler)
            
            self.accuracy["point"].append(self.pos)
            location_match = self.threeD_distance(target_object["world_location"],l)
            self.accuracy["location_accuracy"].append(location_match)
            match_roll,match_pitch,match_yaw = self.pose_match(self.euler,p_euler)
            self.accuracy["roll_accuracy"].append(match_roll)
            self.accuracy["pitch_accuracy"].append(match_pitch)
            self.accuracy["yaw_accuracy"].append(match_yaw)

        elif self.state == ADD_point:
            print('==ADD_point==')
            self.state = Fixed_Pose
            if self.movement_y_cnt < 5:
                self.movement_y_cnt = self.movement_y_cnt +1
                return
            else:
                self.movement_y_cnt = 0
            if self.movement_x_cnt < 5:
                self.movement_x_cnt = self.movement_x_cnt +1
                return
            else: 
                self.movement_x_cnt = 0
            if self.movement_z_cnt < 1: 
                self.movement_z_cnt = self.movement_z_cnt +1
                return
            else:
                self.movement_z_cnt = 0 
                self.state = SAVE_data
        
        elif self.state == SAVE_data:
            print('==SAVE_data==')
            self.suction.gripper_vaccum_off()
            data_df = pd.DataFrame(self.accuracy)
            print(data_df)
            data_df.loc['avg'] = data_df.mean()
            print("accuracy is :",data_df.loc['avg'])
            
            now_work_path = os.getcwd()
            data_df.to_csv(str(now_work_path)+ "/ACon_manupulator.csv")
            print("FINISH!\n[" +
                  "ACon_manupulator.csv] saving in " + str(now_work_path))
            self.state = END
        elif self.state == END:
            return
        

    #-------------------------座標轉換------------------------------------------------------
    def Image_transform(self, Camera_Image_X, Camera_Image_Y):
        Arm_posX = (866 - Camera_Image_Y)*0.000889 - 0.4795     
        Arm_posY = (974 - Camera_Image_X)*0.000889 + 0.3960     
        return Arm_posX, Arm_posY

    def pose_match(self, arm_pos,target_pos ):
        return  abs(arm_pos[0]-(-target_pos[2])) , abs(arm_pos[1]-(-target_pos[1])), abs(arm_pos[2]-target_pos[0])
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

