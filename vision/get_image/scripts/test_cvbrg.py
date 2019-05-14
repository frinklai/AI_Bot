#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from get_image.srv import *
import datetime 
import os
sys.path.insert(1, "/home/iclab-arm/.local/lib/python3.5/site-packages/") 

import cv2
# from cv_bridge import CvBridge, CvBridgeError

if __name__ == '__main__':
    rospy.init_node('aa', anonymous=True)
    print('python version is: ', sys.version)
    print('cv version is: ', cv2.__version__)
    

    # bridge = CvBridge()
    vid = cv2.VideoCapture(0)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        return_value, frame = vid.read()
        cv2.imshow("result", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        rate.sleep()
    vid.release()
    cv2.destroyAllWindows()