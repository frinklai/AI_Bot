#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from get_image.srv import *
import datetime 
#sys.path.insert(1, "/home/iclab-arm/.local/lib/python3.5/site-packages/") 
import cv2
from cv_bridge import CvBridge, CvBridgeError
import sys
import time 
import argparse
import numpy as np
import os

parser = argparse.ArgumentParser()
parser.add_argument('--Object_Name', type=str, default='.', help='Class name of training object.')
FLAGS = parser.parse_args()

Object_Name = FLAGS.Object_Name
Train_Data_Dir = os.path.dirname(os.path.realpath(__file__)) + '/Training_Data/' + \
    str(datetime.datetime.now()) + '_' + Object_Name + '/'


class Get_image():
    def __init__(self):
            rospy.init_node('get_image_from_FLIR', anonymous=True)
            self.bridge = CvBridge()
            self.image = np.zeros((0,0,3), np.uint8)
            self.take_picture_counter = 0

            #s = rospy.Service("request FLIR", FLIR_image, self.service_callback)
            rospy.Subscriber("/camera/image_color", Image, self.callback)

            if not os.path.exists(Train_Data_Dir):
                os.makedirs(Train_Data_Dir)
            rospy.spin()

    def callback(self, image):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)
        cv2.namedWindow("result", cv2.WINDOW_NORMAL)
        cv2.imshow("result", self.cv_image)
        self.get_image(self.cv_image)
        cv2.waitKey(1)
    
    def get_image(self, crop_image):
        if cv2.waitKey(33) & 0xFF == ord('s'):
            name = str(Train_Data_Dir + str(datetime.datetime.now()) + '_' + Object_Name + '_' + str(self.take_picture_counter+1) + ".jpg")
            cv2.imwrite(name,crop_image)
            print("[Save] ", name)
            self.take_picture_counter += 1
        else:
            pass

if __name__ == '__main__':

    print('python version is: ', sys.version)
    listener = Get_image()
    cv2.destroyAllWindows()
