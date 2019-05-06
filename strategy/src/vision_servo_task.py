#!/usr/bin/env python

"""Use to generate arm task and run."""

import os
import sys
import copy
from math import degrees

import rospy
from std_msgs.msg import Bool, Int32
from arm_control import ArmTask, SuctionTask
from yolo_v3.msg   import ROI_array  
from yolo_v3.msg   import ROI  



PICKORDER = 0
SPEED_R     = 60
SPEED_L     = 30
LUNCHBOX_H = 0.045
# The lesser one
lunchQuan = 1              
drinkQuan = 1
riceQuan  = 2

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
move_to_box     = 10
box_down        = 11
box_up          = 12
move_to_obj     = 13
move_to_init    = 14

count = 0
box_cnt = 1
obj_pos = 0
obj_name = ''
x = 0
y = 0

objectName = ['lunchbox', 'lunchbox', 'lunchbox', 'lunchbox',
              'drink',    'drink',    'drink',    'drink',
              'riceball', 'riceball', 'riceball', 'riceball']

lunchboxPos = [[-0.43,  0.165, -0.69],
               [-0.42,  0.15, -0.69],
               [-0.42,  0.15, -0.69],
               [-0.42,  0.15, -0.69]]

drinkPos =    [[-0.2, 0.09, -0.61],
               [-0.295, 0.09, -0.61],                   
               [-0.2, 0.19, -0.61],                              
               [-0.295, 0.19, -0.61]]

riceballPos = [[-0.17,  -0.221, -0.69],
               [-0.267,  -0.223, -0.69],
               [-0.17,  -0.1, -0.69],                             
               [-0.27,  -0.1, -0.69]]

lunchboxEu = [150, 0, 0]

drinkEu =    [0, 0, 0]
            
riceballXXEu = [45, 0, 0]
riceballEu   = [30, 0, 0]

               
objectPos = [lunchboxPos, drinkPos, riceballPos]
objectEu  = [lunchboxEu,  drinkEu,  riceballEu]

topRight    = [0.365, -0.1, -0.225]
topLeft     = [0.365,  0.1, -0.225]
middleRight = [0.445, -0.1, -0.545]
middleLeft  = [0.445,  0.1, -0.545]
bottomRight = [0.5, -0.2, -1.015]
bottomLeft  = [0.5,  0.2, -1.015]

topRightEu    = [-175, 35, 25]
topLeftEu     = [-160, 45, 35]
middleRightEu = [0, 90,  -45]
middleLeftEu  = [0, 90,  -30]
bottomRightEu = [0, 90,  30]
bottomLeftEu  = [0, 90, -30]

topRightPhi    = -45 
topLeftPhi     = -45
middleRightPhi = 40
middleLeftPhi  = 45
bottomRightPhi = 25
bottomLeftPhi  = 25

topRightSuc   = -68 
topLeftSuc    = -60


def setQuantity():
    for index in range(lunchQuan):
        objectName[index] = 'lunchboxXX'
        lunchboxPos[index][1] *= -1
        lunchboxPos[lunchQuan - index -1][2] += LUNCHBOX_H * index
    for index in range(4 - lunchQuan):
        lunchboxPos[4 - index -1][2] += LUNCHBOX_H * index
    for index in range(drinkQuan):
        objectName[index+4] = 'drinkXX'
    for index in range(riceQuan):
        objectName[index+8] = 'riceballXX'
    print lunchboxPos
    print objectName


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
        self.pickListAll = len(lunchboxPos) + len(riceballPos) + len(drinkPos)
        self.pickList = PICKORDER
        self.pos   = [0, 0, 0]
        self.euler = [0, 0, 0]
        self.phi   = 0
        self.sucAngle = 0
        if self.name == 'right':
            self.is_right = 1
            self.speed = SPEED_R
        if self.name == 'left':
            self.is_right = -1
            self.speed = SPEED_L
        self.init_pub_sub()
        if self.en_sim:
            self.suction = SuctionTask(self.name + '_gazebo')
        else:
            self.suction = SuctionTask(self.name)
            rospy.on_shutdown(self.suction.gripper_vaccum_off)
        
    @property
    def finish(self):
        return self.pickList == self.pickListAll

    def init_pub_sub(self):
        rospy.Subscriber('/object/ROI_array', ROI_array, self.get_obj_info_cb)

    def getRearSafetyPos(self):
        self.pos   = [0, -0.5*self.is_right, -0.5]
        self.euler = [-90*self.is_right, -20, 30*self.is_right]
        self.phi   = -60*self.is_right

    def getFrontSafetyPos(self):
        self.pos   = (0.1, -0.4*self.is_right, -0.45)
        self.euler = (0, 0, 0*self.is_right)
        self.phi   = 45*self.is_right

    def getObjectPos(self):
        if self.finish:
            return
        while objectPos[self.pickList/4][self.pickList%4][1]*self.is_right > 0:
            self.pickList += 1 
            if self.finish:
                return
        self.pos   = objectPos[self.pickList/4][self.pickList%4][:]
        self.euler = objectEu[self.pickList/4][:]
        if objectName[self.pickList] == 'riceballXX':
            self.euler = riceballXXEu[:]
        self.euler[0] *= self.is_right
        self.euler[2] *= self.is_right
        self.phi   = -30*self.is_right
        if self.reGripCnt != 0:
            if self.reGripCnt == 1:
                if self.pickList == 4 or self.pickList == 6 or self.pickList == 8 or self.pickList == 10:
                    self.pos[0] += 0.005
                else:
                    self.pos[0] += 0.02
                self.pos[1] += 0.01
            if self.reGripCnt == 2:
                if self.pickList == 4 or self.pickList == 6 or self.pickList == 8 or self.pickList == 10:
                    self.pos[0] += 0.005
                else:
                    self.pos[0] += 0.02
                self.pos[1] -= 0.01
            if self.reGripCnt == 3:
                self.pos[0] -= 0.01
                self.pos[1] -= 0.01

    def getPlacePos(self):
        if objectName[self.pickList] == 'lunchboxXX':
            self.pos   = bottomRight[:]
            self.euler = bottomRightEu[:]
            self.phi   = bottomRightPhi*self.is_right
            self.sucAngle = -90
            self.pos[2] += ((self.pickList%4))*0.05

        elif objectName[self.pickList] == 'lunchbox':
            self.pos   = bottomLeft[:]
            self.euler = bottomLeftEu[:]
            self.phi   = bottomLeftPhi*self.is_right
            self.sucAngle = -90
            self.pos[2] += ((self.pickList%4) - lunchQuan)*0.05

        elif objectName[self.pickList] == 'drinkXX':
            self.pos   = middleRight[:]
            self.euler = middleRightEu[:]
            self.phi   = middleRightPhi*self.is_right
            self.sucAngle = -90
            self.pos[0] += (drinkQuan - (self.pickList%4) - 1)*0.1

        elif objectName[self.pickList] == 'drink':
            self.pos   = middleLeft[:]
            self.euler = middleLeftEu[:]
            self.phi   = middleLeftPhi*self.is_right
            self.sucAngle = -90
            self.pos[0] += (4 - (self.pickList%4) - 1)*0.1

        elif objectName[self.pickList] == 'riceballXX':
            self.pos   = topLeft[:]
            self.euler = topLeftEu[:]
            self.phi   = topLeftPhi*self.is_right
            self.sucAngle = topLeftSuc
            self.pos[0] += (riceQuan - (self.pickList%4) - 1)*0.045

        elif objectName[self.pickList] == 'riceball':
            self.pos   = topRight[:]
            self.euler = topRightEu[:]
            self.phi   = topRightPhi*self.is_right
            self.sucAngle = topRightSuc
            self.pos[0] += (4 - (self.pickList%4) - 1)*0.045      

    def process(self):
        if self.arm.is_stop:                                       # must be include in your strategy
            self.finish = True                                     # must be include in your strategy
            print "!!! Robot is stop !!!"                          # must be include in your strategy
            # self.suction.gripper_vaccum_off()                      # must be include in your strategy
            return                                                 # must be include in your strategy

        if self.state == idle:
            self.suction.gripper_vaccum_off()
            if self.finish:
                return

        elif self.state == busy:
            # print('self.state == busy')
            if self.arm.is_busy:
                # if (self.nowState == leaveBin or self.nowState == frontSafetyPos or self.nowState == move2Shelf) and not self.suction.is_grip and not self.en_sim:
                #     self.state = missObj
    #            print('self.state == busy')
                return
            else:
                self.state    = self.nextState
                self.nowState = self.nextState
                return

        elif self.state == initPose:
            print('self.state == initPose')
            self.state = busy
            self.nextState = wait_img_pos
            #self.arm.set_speed(self.speed)
            self.arm.set_speed(10)
            self.pos   = [0, 0.5, -0.5]
            self.euler = [0, 0, 0]
            self.phi = 0
            self.arm.ikMove(mode= 'p2p', pos = self.pos, euler = self.euler, phi = self.phi)  
            #self.arm.jointMove(0, (0, -1, 0, 1.57, 0, -0.57, 0))
            # self.suction.gripper_suction_deg(0)

        elif self.state == wait_img_pos:        # wait_img_pos
            # print('self.state == wait_img_pos')
            self.state = wait_img_pos

            #aaa
            #----------------------------------------------------------
            
            self.pos   = [0, 0.85, -0.02]
            self.euler = [0, 0, 90]
            self.phi = 0
            self.arm.ikMove(mode='p2p', pos = self.pos, euler =  self.euler, phi = self.phi)
            #----------------------------------------------------------

            # print('len(self.img_data_list = ', len(self.img_data_list))
            if(len(self.img_data_list)!=0):
                for i in range(len(self.img_data_list)):
                    if(i!=0):
                        print("\n")
                    print("----- stra detect object_" + str(i) + " ----- ")
                    print("object_name = " + str(self.img_data.object_name))
                    print("score = " + str(self.img_data.score))
                    print("min_xy = [ " +  str( [self.img_data.min_x, self.img_data.min_y] ) +  " ]" )
                    print("max_xy = [ " +  str( [self.img_data.Max_x, self.img_data.Max_y] ) +  " ]" )
                    self.nextState = goto
                    obj_name = self.img_data.object_name
                    x = (self.img_data.min_x + self.img_data.Max_x)/2
                    y = (self.img_data.min_y + self.img_data.Max_y)/2
            else:
                self.nextState = wait_img_pos
            self.state = self.nextState

        elif self.state == goto:               # goto
            print('self.state == goto')
            self.state = busy
            self.nextState = move_standby
        #    self.nextState = down
        #    self.pos   = [0, 0.4, -0.55]
        #    self.euler = [0, 0, 0]
        #    self.phi = 0
        #    self.arm.ikMove(mode= 'p2p', pos = self.pos, euler = self.euler, phi = self.phi)
        #aaa
        #    self.pos   = [0.1, 0, 0.1]
        #    self.arm.relative_move_pose(mode='line', pos=self.pos)
        #--------------------------------------------------------------------------
        #    self.pos = [0, -0.45, -0.55]
        #    self.euler = [0, 0, -90]
        #    self.phi = 0
        #    self.arm.relative_move(mode='p2p', euler=self.euler, pos =self.pos, phi=self.phi)        

        elif self.state == move_standby:
            print('self.state == move_standby')
            self.state = busy
            self.nextState = move_to_obj
            self.pos   = [0, 0.6, -0.4]
            self.euler = [0, 0, 0]
            self.phi = 0
            self.arm.ikMove(mode = 'p2p', pos = self.pos, euler = self.euler, phi = self.phi)

        elif self.state == move_to_obj:
            global obj_pos
            global obj_name
            global x
            global y
        #    pos_x = [0.05, -0.06, -0.17]
        #    pos_y = [0.68, 0.77, 0.67]
        #    pos_z = [-0.4, -0.39, -0.4]
            print('self.state == move_to_obj')
            
            posX = (320-x)*0.0012-0.01
            posY = (y-240)*0.0012+0.6
            print('---------')
            print(posX,posY)
            if obj_name == 'tea':
                pos_z = -0.39
            else:
                pos_z = -0.4 
            self.state = busy
            self.nextState = down
            self.pos   = [round(posX, 4), round(posY, 4), pos_z]
           # self.pos = [-0.3208, 0.6467999999999999, -0.39]
            print(self.pos)
            self.euler = [0, 0, 0]
            self.phi = 0
            self.arm.ikMove(mode= 'p2p', pos = self.pos, euler = self.euler, phi = self.phi)  
              
        #--------------------------------------------------------------------------
        
        elif self.state == down:               # down
            print('self.state == down')
        #aaa    
            self.suction.gripper_vaccum_on()
            self.state = grasping
        #----------------------------------
            self.state = busy
            self.nextState = grasping
        #----------------------------------
            self.pos   = [0, 0, -0.16]
            self.arm.relative_move_pose(mode='line', pos=self.pos)
            rospy.sleep(.1)

        elif self.state == grasping:
            print('self.state == ready to grasping')
            if self.suction.is_grip or self.en_sim:
                print('self.state == grasping')
                self.arm.clear_cmd()
                self.state = busy
                self.nextState = up
                # self.reGripCnt = 0
            # elif not self.arm.is_busy:
            #     self.state = missObj

        elif self.state == up:               # up
            print('self.state == up')
            self.state = busy
        #aaa
        #    self.nextState = back_home
        #-------------------------------------
            self.nextState = move_to_box
        #-------------------------------------
            self.pos   = [0, 0, 0.16]
            self.arm.relative_move_pose(mode='line', pos=self.pos)

        elif self.state == back_home:               # back_home
            print('self.state == back_home')
            self.suction.gripper_vaccum_off()
            self.state = busy
            self.nextState = idle
            self.arm.jointMove(0, (0, -1, 0, 1.57, 0, -0.57, 0))
            self.arm.relative_move_pose(mode='line', pos=self.pos)

        #aaa
        #------------------------------------------------------------
        elif self.state == move_to_box:
            global count
            global obj_name
            print('self.state == move_to_box')
            self.state = busy
            self.nextState = box_down  
            print(obj_name)
            if obj_name == 'soda':
                print('soddddddddddddddddddddaaaaaaaaaaaaaaaaaaaa')
                self.pos = [-0.2, 0.4, -0.4]
            elif obj_name == 'tea':
                print('ttttttttttttttttttteeeeeeeeeeeeeeaaaaaaaaa')
                self.pos = [0, 0.4, -0.4]
            elif obj_name == 'bottle':
                print('boooooooooooooooooooooooooooooootllllllle')
                self.pos = [0.2, 0.4, 0.4]
            else:
                print('aaaaaaaaaaaaa')
        #    self.pos   = [-0.2 + count, 0.4, -0.4]  
            self.euler = [0, 0, 0]
            self.phi   = 0
            self.arm.ikMove(mode= 'p2p', pos = self.pos, euler = self.euler, phi = self.phi)
            count += 0.2

        elif self.state == box_down:
            print('self.state == box_down')
            self.state = busy
            self.nextState = box_up
            self.pos = [0, 0, -0.05]
            self.arm.relative_move_pose(mode = 'line', pos = self.pos)

        elif self.state == box_up:
            global box_cnt
            print('self.state == box_up')
            self.suction.gripper_vaccum_off()
            self.state = busy
            if box_cnt == 3:    
                self.nextState = move_to_init
            else:            
                self.nextState = wait_img_pos   
            self.pos = [0, 0, 0.05]
            self.arm.relative_move_pose(mode = 'line', pos = self.pos)   
            box_cnt +=1  

        elif self.state == move_to_init:
            print('self.state == move_to_box')
            self.state = busy
            self.nextState = idle  
            
            self.pos   = [0, 0.5, -0.4]  
            self.euler = [0, 0, 0]
            self.phi   = 0
            self.arm.ikMove(mode= 'p2p', pos = self.pos, euler = self.euler, phi = self.phi)    
        #------------------------------------------------------------

    def get_obj_info_cb(self, data):
        self.img_data = ROI()
        self.img_data_list = data.ROI_list
        # print("Detected object number = " + str(len(data.ROI_list)))
        for i in range(len(data.ROI_list)):
            self.img_data.object_name = data.ROI_list[i].object_name
            self.img_data.score       = data.ROI_list[i].score
            self.img_data.min_x = data.ROI_list[i].min_x
            self.img_data.min_y = data.ROI_list[i].min_y
            self.img_data.Max_x = data.ROI_list[i].Max_x
            self.img_data.Max_y = data.ROI_list[i].Max_y
            return self.img_data
            # if(i!=0):
            #     print("\n")
            # print("----- object_" + str(i) + " ----- ")
            # print("object_name = " + str(object_name))
            # print("score = " + str(score))
            # print("min_xy = [ " +  str(min_xy) +  " ]" )
            # print("max_xy = [ " +  str(max_xy) +  " ]" )

def start_callback(msg):
    global is_start
    if msg.data == 1 and not is_start:
        is_start = True


if __name__ == '__main__':
    rospy.init_node('example')        # enable this node

    

    # right = stockingTask('right')      # Set up right arm controller
    left  = stockingTask('left')       # Set up left arm controller
    rospy.sleep(.3)
    # setQuantity()

    # while not rospy.is_shutdown() and not is_start:
    #     rospy.loginfo('waiting for start signal')
    #     rospy.sleep(.5)
    
    # SuctionTask.switch_mode(True)

    rate = rospy.Rate(50)  # 30hz
    
    # while not rospy.is_shutdown() and (not left.finish):
    while not rospy.is_shutdown():
        try:
            left.process()
        except rospy.ROSInterruptException:
            print('error')
            pass
            break
        rate.sleep()
