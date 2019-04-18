#!/usr/bin/env python

"""Use to generate arm task and run."""

import os
import sys
import copy
from math import degrees

import rospy
from std_msgs.msg import Bool, Int32
from arm_control import ArmTask, SuctionTask





PICKORDER = 0
SPEED_R     = 60
SPEED_L     = 70
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
# frontSafetyPos  = 3
# rearSafetyPos   = 4
# move2Bin        = 5
# move2Shelf      = 6
# moveIn2Shelf    = 7
# leaveBin        = 8
# leaveShelf      = 9
# move2Object     = 10
# move2PlacedPos  = 11
# pickObject      = 12
# placeObject     = 13
# safePose1       = 14
# safePose2       = 15
# safePose3       = 16 
# riceballEuler   = 17
# rearSafetyPos2  = 18
# leavePlacePos   = 19
# grasping        = 20
# missObj         = 21
# safePose4       = 22

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
        if self.en_sim:
            self.suction = SuctionTask(self.name + '_gazebo')
        else:
            self.suction = SuctionTask(self.name)
            rospy.on_shutdown(self.suction.gripper_vaccum_off)
        
    @property
    def finish(self):
        return self.pickList == self.pickListAll

    # def setQuantity(self):
    #     for index in range(lunchQuan):
    #         objectName[index] = 'lunchboxXX'
    #         lunchboxPos[index][1] *= -1
    #         lunchboxPos[lunchQuan - index -1][2] += LUNCHBOX_H * index
    #     for index in range(4 - lunchQuan):
    #         lunchboxPos[4 - index -1][2] += LUNCHBOX_H * index
    #         print LUNCHBOX_H * index
    #     for index in range(drinkQuan):
    #         objectName[index+4] = 'drinkXX'
    #     for index in range(riceQuan):
    #         objectName[index+8] = 'riceballXX'
    #     print lunchboxPos

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
            if self.finish:
                return

        elif self.state == initPose:
            print('self.state == initPose')
            self.state = busy
            self.nextState = goto
            self.arm.set_speed(self.speed)
            self.arm.jointMove(0, (0, -1, 0, 1.57, 0, -0.57, 0))
            # self.suction.gripper_suction_deg(0)

        elif self.state == busy:
            # print('self.state == busy')
            if self.arm.is_busy:
                # if (self.nowState == leaveBin or self.nowState == frontSafetyPos or self.nowState == move2Shelf) and not self.suction.is_grip and not self.en_sim:
                #     self.state = missObj
                return
            else:
                self.state    = self.nextState
                self.nowState = self.nextState
                return

        elif self.state == goto:               # goto
            print('self.state == goto')
            self.state = busy
            self.nextState = down
            self.pos   = [0.1, 0, 0.1]
            self.arm.relative_move_pose(mode='line', pos=self.pos)
        
        elif self.state == down:               # down
            print('self.state == down')
            self.state = busy
            self.nextState = up
            self.pos   = [0, 0, -0.1]
            self.arm.relative_move_pose(mode='line', pos=self.pos)

        elif self.state == up:               # up
            print('self.state == up')
            self.state = busy
            self.nextState = back_home
            self.pos   = [0, 0, 0.1]
            self.arm.relative_move_pose(mode='line', pos=self.pos)

        elif self.state == back_home:               # up
            print('self.state == back_home')
            self.state = busy
            self.nextState = idle
            self.arm.jointMove(0, (0, -1, 0, 1.57, 0, -0.57, 0))
            self.arm.relative_move_pose(mode='line', pos=self.pos)


def start_callback(msg):
    global is_start
    if msg.data == 1 and not is_start:
        is_start = True


if __name__ == '__main__':
    rospy.init_node('example')        # enable this node

    is_start = False
    rospy.Subscriber(
        'scan_black/dualarm_start_1',
        Int32,
        start_callback,
        queue_size=1
    )
    pub = rospy.Publisher(
        'scan_black/strategy_behavior',
        Int32,
        queue_size=1
    )

    # right = stockingTask('right')      # Set up right arm controller
    left  = stockingTask('left')       # Set up left arm controller
    rospy.sleep(.3)
    setQuantity()

    # while not rospy.is_shutdown() and not is_start:
    #     rospy.loginfo('waiting for start signal')
    #     rospy.sleep(.5)
    
    # SuctionTask.switch_mode(True)

    rate = rospy.Rate(30)  # 30hz
    
    while not rospy.is_shutdown() and (not left.finish):
        try:
            left.process()
            rate.sleep()
        except rospy.ROSInterruptException:
            print('error')
            pass
            break

    # robot arm back home
    # if right.arm.is_stop is not True:
    #     rospy.loginfo('back home')
    #     left.arm.wait_busy()
    #     left.arm.jointMove(0, (0, -1, 0, 2, 0, -0.7, 0))

    #     right.arm.wait_busy()
    #     right.arm.jointMove(0, (0, -1, 0, 2, 0, -0.7, 0))
    
    #     left.arm.wait_busy()
    #     left.arm.jointMove(0, (0, 0, 0, 0, 0, 0, 0))

    #     right.arm.wait_busy()
    #     right.arm.jointMove(0, (0, 0, 0, 0, 0, 0, 0))

    # SuctionTask.switch_mode(False)
