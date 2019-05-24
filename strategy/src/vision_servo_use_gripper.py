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
SPEED_L     = 30

idle            = 0
busy            = 1
initPose        = 2
goto            = 3
down            = 4
up              = 5
back_home       = 6
wait_img_pos    = 7
grasping        = 8
move_standby    = 9
box_standby     = 10
move_to_box     = 11
box_down        = 12
box_grasp       = 13
box_up          = 14
move_to_obj     = 15
move_to_init    = 16
down_sec        = 17

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
        if self.name == 'left':
            self.is_right = -1
            self.speed = SPEED_L
        self.init_pub_sub()
        if self.en_sim:
            self.suction = SuctionTask(self.name + '_gazebo')
        else:
            self.suction = SuctionTask(self.name)
    
    def init_pub_sub(self):
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

        elif self.state == initPose:
            print('self.state == initPose')
            self.state = busy
            self.nextState = wait_img_pos
            self.arm.set_speed(self.speed)
            self.pos   = [0.4, 0.5, -0.3]
            self.euler = [0, 0, 0]
            self.phi = 0
            self.arm.ikMove(mode= 'p2p', pos = self.pos, euler = self.euler, phi = self.phi)        


        elif self.state == wait_img_pos:        # wait_img_pos
            # print('self.state == wait_img_pos')
            self.state = wait_img_pos
            # print('len(self.img_data_list = ', len(self.img_data_list))
            #print('aaaaa')
            #rospy.Subscriber('/object/ROI_array', ROI_array, self.get_obj_info_cb)

            #if(len(self.img_data_list)!=0):
            #print("str(self.img_data.object_name): ", str(self.img_data.object_name))
            

            if(len(self.img_data_list)!=0):
                #print('bbbb')
                for i in range(len(self.img_data_list)):
                    # if(i!=0):
                    #     print("\n")
                    if (self.img_data.min_x > 400) and (self.img_data.min_y) > 20 and (self.img_data.Max_x < 1440) and (self.img_data.Max_y < 990):        
                        print("----- stra detect object_" + str(i) + " ----- ")
                        print("object_name = " + str(self.img_data.object_name))
                        print("score = " + str(self.img_data.score))
                        print("min_xy = [ " +  str( [self.img_data.min_x, self.img_data.min_y] ) +  " ]" )
                        print("max_xy = [ " +  str( [self.img_data.Max_x, self.img_data.Max_y] ) +  " ]" )
                        self.nextState = goto
                        obj_name = self.img_data.object_name
                        x = (self.img_data.min_x + self.img_data.Max_x)/2
                        y = (self.img_data.min_y + self.img_data.Max_y)/2    
                        time.sleep(1)
            else:
                self.nextState = wait_img_pos
            self.state = self.nextState                            

        elif self.state == goto:               # goto
            print('self.state == goto')
            self.state = busy
            self.nextState = move_standby

        elif self.state == move_standby:
            print('self.state == move_standby')
            self.state = busy
            self.nextState = move_to_obj
            self.pos   = [0, 0.5, -0.4]
            self.euler = [0, 0, 0]
            self.phi = 0
            self.arm.ikMove(mode = 'p2p', pos = self.pos, euler = self.euler, phi = self.phi)    

        elif self.state == move_to_obj:          
            global x
            global y
            # pos_x = [0.05, -0.06, -0.17]
            # pos_y = [0.68, 0.77, 0.67]
            # pos_z = [-0.4, -0.39, -0.4]
            print('self.state == move_to_obj')
            # posX = (729 - y)*0.000518 - 0.336
            # posY = (1149 - x)*0.000518 + 0.35
            posX = (706 - y)*0.000621 - 0.317
            posY = (1044 - x)*0.000621 + 0.398
            self.state = busy
            self.nextState = down
            self.pos   = [round(posX, 4), round(posY, 4), -0.45]
            #self.pos   = [ 0.05, 0.68, -0.4]
            print(self.pos)
            self.euler = [0, 0, 0]
            self.phi = 0
            self.arm.ikMove(mode= 'p2p', pos = self.pos, euler = self.euler, phi = self.phi)    

        elif self.state == down:               # down
            print('self.state == down')
            self.state = busy
            self.nextState = down_sec
            self.pos   = [0, 0, -0.07]
            self.arm.relative_move_pose(mode='p2p', pos=self.pos)
            rospy.sleep(.1)

        elif self.state == down_sec:
            print('self.state == down_sec')
            self.state = busy
            self.nextState = grasping
            self.pos  = [0, 0, -0.06]
            self.arm.relative_move_pose(mode='p2p', pos=self.pos)

        elif self.state == grasping:
            print('self.state == ready to grasping')
            self.arm.clear_cmd()
            self.state = busy
            gripper.Send_Gripper_Command('Catch_all')
            self.nextState = up
            # if self.suction.is_grip or self.en_sim:
            #     print('self.state == grasping')
            #     self.arm.clear_cmd()
            #     self.state = busy
            #     self.nextState = up
                

        elif self.state == up:               # up
            print('self.state == up')
            self.state = busy

            self.nextState = box_standby
            self.pos   = [0, 0, 0.13]
            self.arm.relative_move_pose(mode='p2p', pos=self.pos)  

        elif self.state == box_standby:
            print('self.state == box_standby')
            self.state = busy
            self.nextState = move_to_box  
            self.pos   = [0, 0.5, -0.4]
            self.euler = [0, 0, 0]
            self.phi = 0
            self.arm.ikMove(mode = 'p2p', pos = self.pos, euler = self.euler, phi = self.phi)      

        elif self.state == move_to_box:
            #global count
            print('self.state == move_to_box')
            self.state = busy
            self.nextState = box_down  
            #self.pos   = [-0.2 + count, 0.4, -0.4]  
            self.pos   = [0.4, 0.4, -0.4]
            self.euler = [0, 0, 0]
            self.phi   = 0
            self.arm.ikMove(mode= 'p2p', pos = self.pos, euler = self.euler, phi = self.phi)
            #count += 0.2    

        elif self.state == box_down:
            print('self.state == box_down')
            self.state = busy
            self.nextState = box_grasp
            self.pos = [0, 0, -0.05]
            self.arm.relative_move_pose(mode = 'p2p', pos = self.pos)

        elif self.state == box_grasp:
            print('self.state == box_grasp')
            self.state = busy
            gripper.Send_Gripper_Command('Loosen_all')
            self.nextState = box_up


        elif self.state == box_up:
            global box_cnt
            print('self.state == box_up')
            self.state = busy

            # if box_cnt == 4:    
            #     self.nextState = move_to_init
            #     self.finish = True
            # else:            
            #     self.nextState = wait_img_pos  
            #     #self.nextState = goto  

            self.nextState = wait_img_pos  # add this line for do infinite obj pick
            print('box_cnt = ', box_cnt)

            self.pos = [0, 0, 0.05]
            self.arm.relative_move_pose(mode = 'p2p', pos = self.pos)   
            box_cnt +=1  

        elif self.state == move_to_init:
            print('self.state == move_to_init')
            self.state = busy
            self.nextState = idle  
            
            # self.pos   = [0, 0.5, -0.4]  
            # self.euler = [0, 0, 0]
            self.pos   = [0.4, 0.5, -0.3]
            self.euler = [0, 0, 0]
            self.phi   = 0
            self.arm.ikMove(mode= 'p2p', pos = self.pos, euler = self.euler, phi = self.phi)    

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
    rospy.init_node('gripperasd', anonymous=True)
    gripper = Gripper()
    # gripper.Send_Gripper_Command('Loosen_all')
    # gripper.Send_Gripper_Command('rot_to_parll')
    # gripper.Send_Gripper_Command('rot_to_norm')
    
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

