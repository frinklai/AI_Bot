#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Class definition of YOLO_v3 style detection model on image and video
"""
import sys
sys.path.insert(1, "/usr/local/lib/python3.5/dist-packages/")
sys.path.insert(0, '/opt/installer/open_cv/cv_bridge/lib/python3/dist-packages/')

import colorsys
import os
from timeit import default_timer as timer

import numpy as np
from keras import backend as K
from keras.models import load_model
from keras.layers import Input
from sensor_msgs.msg import Image as rosimage
from PIL import Image, ImageFont, ImageDraw

from yolo3.model import yolo_eval, yolo_body, tiny_yolo_body
from yolo3.utils import letterbox_image
import os
from keras.utils import multi_gpu_model
import sys
import argparse
import path_set as pth
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
# ros
import rospy
from yolo_v3.msg import ROI
from yolo_v3.msg import ROI_array

#sys.path.insert(1, pth.python3_site_pkg_path) 

class YOLO(object):
    _defaults = {
        "model_path"  : pth.model_data_path + 'model_data/yolo.h5',
        "anchors_path": pth.model_data_path + 'model_data/yolo_anchors.txt',
        "classes_path": pth.model_data_path + 'model_data/task_class.txt',

        "score" : 0.05,
        "iou" : 0.45,
        "model_image_size" : (416, 416),
        "gpu_num" : 1,
    }

    @classmethod
    def get_defaults(cls, n):
        if n in cls._defaults:
            return cls._defaults[n]
        else:
            return "Unrecognized attribute name '" + n + "'"
    def FLIR_Callback(self, rosimage):
        self.cv_image = self.bridge.imgmsg_to_cv2(rosimage, "bgr8")

    def __init__(self, **kwargs):
        self.__dict__.update(self._defaults) # set up default values
        self.__dict__.update(kwargs) # and update with user overrides
        self.object_name = self._get_class()
        self.anchors = self._get_anchors()
        self.sess = K.get_session()
        self.boxes, self.scores, self.classes = self.generate()
        self.bridge = CvBridge()
        self.cv_image = np.zeros((1024, 768, 3), np.uint8)
        self.mtx = np.load(pth.model_data_path + 'model_data/camera_calibration_mtx.npy')
        self.dist = np.load(pth.model_data_path + 'model_data/camera_calibration_dist.npy')
        self.newcameramtx = np.load(pth.model_data_path + 'model_data/camera_calibration_newcameramtx.npy')
        self.dst_roi_x, self.dst_roi_y, self.dst_roi_w, self.dst_roi_h  = np.load(pth.model_data_path + 'model_data/camera_calibration_roi.npy')

    def _get_class(self):
        classes_path = os.path.expanduser(self.classes_path)
        with open(classes_path) as f:
            object_name = f.readlines()
        object_name = [c.strip() for c in object_name]
        return object_name

    def _get_anchors(self):
        anchors_path = os.path.expanduser(self.anchors_path)
        
        with open(anchors_path) as f:
            anchors = f.readline()
        anchors = [float(x) for x in anchors.split(',')]
        return np.array(anchors).reshape(-1, 2)

    def generate(self):
        model_path = os.path.expanduser(self.model_path)
        
        assert model_path.endswith('.h5'), 'Keras model or weights must be a .h5 file.'

        # Load model, or construct model and load weights.
        num_anchors = len(self.anchors)
        num_classes = len(self.object_name)
        is_tiny_version = num_anchors==6 # default setting
        try:
            self.yolo_model = load_model(model_path, compile=False)
        except:
            self.yolo_model = tiny_yolo_body(Input(shape=(None,None,3)), num_anchors//2, num_classes) \
                if is_tiny_version else yolo_body(Input(shape=(None,None,3)), num_anchors//3, num_classes)
            self.yolo_model.load_weights(self.model_path) # make sure model, anchors and classes match
        else:
            assert self.yolo_model.layers[-1].output_shape[-1] == \
                num_anchors/len(self.yolo_model.output) * (num_classes + 5), \
                'Mismatch between model and given anchor and class sizes'

        print('{} model, anchors, and classes loaded.'.format(model_path))

        # Generate colors for drawing bounding boxes.
        hsv_tuples = [(x / len(self.object_name), 1., 1.)
                      for x in range(len(self.object_name))]
        self.colors = list(map(lambda x: colorsys.hsv_to_rgb(*x), hsv_tuples))
        self.colors = list(
            map(lambda x: (int(x[0] * 255), int(x[1] * 255), int(x[2] * 255)),
                self.colors))
        np.random.seed(10101)  # Fixed seed for consistent colors across runs.
        np.random.shuffle(self.colors)  # Shuffle colors to decorrelate adjacent classes.
        np.random.seed(None)  # Reset seed to default.

        # Generate output tensor targets for filtered bounding boxes.
        self.input_image_shape = K.placeholder(shape=(2, ))
        if self.gpu_num>=2:
            self.yolo_model = multi_gpu_model(self.yolo_model, gpus=self.gpu_num)
        boxes, scores, classes = yolo_eval(self.yolo_model.output, self.anchors,
                len(self.object_name), self.input_image_shape,
                score_threshold=self.score, iou_threshold=self.iou)
        return boxes, scores, classes

    def detect_image(self, image):
        ROI = None
        ROI_array = list()
        start = timer()

        if self.model_image_size != (None, None):
            assert self.model_image_size[0]%32 == 0, 'Multiples of 32 required'
            assert self.model_image_size[1]%32 == 0, 'Multiples of 32 required'
            boxed_image = letterbox_image(image, tuple(reversed(self.model_image_size)))
        else:
            new_image_size = (image.width - (image.width % 32),
                              image.height - (image.height % 32))
            boxed_image = letterbox_image(image, new_image_size)
        image_data = np.array(boxed_image, dtype='float32')

        # print("\nimage_data.shape = " + str(image_data.shape))
        image_data /= 255.
        image_data = np.expand_dims(image_data, 0)  # Add batch dimension.

        out_boxes, out_scores, out_classes = self.sess.run(
            [self.boxes, self.scores, self.classes],
            feed_dict={
                self.yolo_model.input: image_data,
                self.input_image_shape: [image.size[1], image.size[0]],
                K.learning_phase(): 0
            })

        # print('Found {} boxes for {}'.format(len(out_boxes), 'img'))

        font = ImageFont.truetype(font = pth.font_path +'font/FiraMono-Medium.otf',
                    size=np.floor(3e-2 * image.size[1] + 0.5).astype('int32'))
        thickness = (image.size[0] + image.size[1]) // 300

        for i, c in reversed(list(enumerate(out_classes))):
            predicted_class = self.object_name[c]
            box = out_boxes[i]
            score = out_scores[i]

            label = '{} {:.2f}'.format(predicted_class, score)
            draw = ImageDraw.Draw(image)
            label_size = draw.textsize(label, font)

            top, left, bottom, right = box
            top = max(0, np.floor(top + 0.5).astype('int32'))
            left = max(0, np.floor(left + 0.5).astype('int32'))
            bottom = min(image.size[1], np.floor(bottom + 0.5).astype('int32'))
            right = min(image.size[0], np.floor(right + 0.5).astype('int32'))
            # print('*', label, (left, top), (right, bottom))  # if use python only (without ros), obj info will show in here

            if top - label_size[1] >= 0:
                text_origin = np.array([left, top - label_size[1]])
            else:
                text_origin = np.array([left, top + 1])

            # My kingdom for a good redistributable image drawing library.
            for i in range(thickness):
                draw.rectangle(
                    [left + i, top + i, right - i, bottom - i],
                    outline=self.colors[c])
            draw.rectangle(
                [tuple(text_origin), tuple(text_origin + label_size)],
                fill=self.colors[c])
            draw.text(text_origin, label, fill=(0, 0, 0), font=font)
            del draw
            ROI_info = [predicted_class, score, left, right, top ,bottom]
            ROI_array.append(ROI_info)

        end = timer()
        # print("Time cost = " + str(round((end - start)*1000, 2)) + " ms/frame")
        return image, ROI_array

    def close_session(self):
        self.sess.close()

def detect_video(yolo, video_path, output_path=""):
    
    rospy.Subscriber("/camera/image_color", rosimage, yolo.FLIR_Callback)
    
    while not rospy.is_shutdown():
        un_dst_img = cv.undistort(yolo.cv_image, yolo.mtx, yolo.dist, None, yolo.newcameramtx)
        un_dst_img = un_dst_img[yolo.dst_roi_y:yolo.dst_roi_y+yolo.dst_roi_h, \
                    yolo.dst_roi_x:yolo.dst_roi_x+yolo.dst_roi_w]
        image = Image.fromarray(un_dst_img)
        # image = Image.fromarray(yolo.cv_image)

        image, ROI_array_recive = yolo.detect_image(image)    # receieve ROI
        
        if ROI_array_recive != None:
            
            ROI_array_msg = ROI_array()
            
            for i in range(len(ROI_array_recive)):  
                ROI_msg = ROI()
                ROI_msg.object_name = str(ROI_array_recive[i][0])
                ROI_msg.score      = float(ROI_array_recive[i][1])
                ROI_msg.min_x      = int(ROI_array_recive[i][2])
                ROI_msg.Max_x      = int(ROI_array_recive[i][3])
                ROI_msg.min_y      = int(ROI_array_recive[i][4])
                ROI_msg.Max_y      = int(ROI_array_recive[i][5])
                ROI_array_msg.ROI_list.append(ROI_msg)   
            
            roi_array_pub.publish(ROI_array_msg) 

        else:
            
            print("no object now")
        
        result = np.asarray(image)

        bounding_point = [[405, 25], [1440, 25], [1440, 982], [405, 982]]
        line_color = [(255, 0, 0), (255, 0, 0), (255, 0, 0), (255, 0, 0)]
        line_thickness = 5

        rectangle_shift = [0, 0]

        for i in range(4):
            bounding_point[i][0] = bounding_point[i][0] + rectangle_shift[0]
            bounding_point[i][1] = bounding_point[i][1] + rectangle_shift[1]
            bounding_point[i] = tuple(bounding_point[i])

        cv.line(result, bounding_point[0], bounding_point[1], line_color[0], line_thickness)
        cv.line(result, bounding_point[1], bounding_point[2], line_color[1], line_thickness)
        cv.line(result, bounding_point[2], bounding_point[3], line_color[2], line_thickness)
        cv.line(result, bounding_point[3], bounding_point[0], line_color[3], line_thickness)
        
        cv.namedWindow("result", cv.WINDOW_NORMAL)
        cv.imshow("result", result)
        cv.waitKey(1)

    yolo.close_session()

def detect_img(yolo):
    while not rospy.is_shutdown():
        img = input('Input image filename:')
        try:
            image = Image.open(img)
        except:
            print('Open Error! Try again!')
            continue
        else:
            r_image = yolo.detect_image(image)
            r_image.show()
    yolo.close_session()

FLAGS = None

if __name__ == '__main__':
    try:
        
        rospy.init_node('yolo3', anonymous=True)
        roi_array_pub = rospy.Publisher("/object/ROI_array",ROI_array,queue_size=10)
        # class YOLO defines the default value, so suppress any default here
        parser = argparse.ArgumentParser(argument_default=argparse.SUPPRESS)
        '''
        Command line options
        '''

        parser.add_argument(
            '--model', type=str,
            help='path to model weight file, default ' + YOLO.get_defaults("model_path") 
        )

        parser.add_argument(
            '--anchors', type=str,
            help='path to anchor definitions, default ' + YOLO.get_defaults("anchors_path")
        )

        parser.add_argument(
            '--classes', type=str,
            help='path to class definitions, default ' + YOLO.get_defaults("classes_path")
        )

        parser.add_argument(
            '--gpu_num', type=int,
            help='Number of GPU to use, default ' + str(YOLO.get_defaults("gpu_num"))
        )

        parser.add_argument(
            '--image', default=False, action="store_true",
            help='Image detection mode, will ignore all positional arguments'
        )
        '''
        Command line positional arguments -- for video detection mode
        '''
        parser.add_argument(
            "--input", nargs='?', type=str,required=False,default='0',
            help = "Video input path"
        )

        parser.add_argument(
            "--output", nargs='?', type=str, default="",
            help = "[Optional] Video output path"
        )
        
        FLAGS = parser.parse_args(rospy.myargv()[1:])  # add rospy.myargv()[1:], or roslaunch will error (process will died)
        
        if FLAGS.image:
            """
            Image detection mode, disregard any remaining command line arguments
            """
            print("Image detection mode")
            if "input" in FLAGS:
                print(" Ignoring remaining command line arguments: " + FLAGS.input + "," + FLAGS.output)
            detect_img(YOLO(**vars(FLAGS)))
            
        elif "input" in FLAGS:
            detect_video( YOLO(**vars(FLAGS)), FLAGS.input, FLAGS.output)
        else:
            print("Must specify at least video_input_path.  See usage with --help.")
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo('enable yolo.py failed')
        pass
