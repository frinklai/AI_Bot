#!/usr/bin/env python

"""Use to generate arm task and run."""

import os
import sys
import copy
from math import degrees

import rospy
from std_msgs.msg import Bool, Int32
from arm_control  import ArmTask, SuctionTask
from yolo_v3.msg  import ROI_array  
from yolo_v3.msg  import ROI  
from comm_stm32   import Gripper



if __name__ == '__main__':
    rospy.init_node('gripperasd', anonymous=True)
    gripper = Gripper()
    gripper.Send_Gripper_Command('Loosen_all')
    

