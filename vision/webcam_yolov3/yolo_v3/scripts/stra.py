#!/usr/bin/env python     
# Every Python ROS Node will have the command above, which declaration at the top.
# The first line makes sure your script is executed as a Python script.

import rospy
from turtlesim.msg   import Pose    
from yolo_v3.msg   import ROI_array  

def get_obj_info_cb(data):
    print("\n========== Detected object number = " + str(len(data.ROI_list)) + " ========== ")
    for i in range(len(data.ROI_list)):
        object_name = data.ROI_list[i].object_name
        score       = data.ROI_list[i].score
        min_xy = [data.ROI_list[i].min_x, data.ROI_list[i].min_y]
        max_xy = [data.ROI_list[i].Max_x, data.ROI_list[i].Max_y]

        if(i!=0):
            print("\n")
        print("----- object_" + str(i) + " ----- ")
        print("object_name = " + str(object_name))
        print("score = " + str(score))
        print("min_xy = [ " +  str(min_xy) +  " ]" )
        print("max_xy = [ " +  str(max_xy) +  " ]" )

if __name__ == '__main__':
    """ Initialize ros node and publish cmd """
    try:
        rospy.init_node('stra', anonymous=True)
        rospy.loginfo('start to get obj info')
        rospy.Subscriber('/object/ROI_array', ROI_array, get_obj_info_cb)
        rospy.spin()


    except rospy.ROSInterruptException:
        rospy.loginfo('error')
        pass