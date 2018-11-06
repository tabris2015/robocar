#!/usr/bin/env python
from __future__ import print_function
import roslib
import rospkg
roslib.load_manifest('robocar')
import sys
import csv
import numpy as np
import rospy
import cv2
import message_filters
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image, Range, CompressedImage, Joy
from geometry_msgs.msg import Twist, TwistStamped
from cv_bridge import CvBridge, CvBridgeError

# from keras.preprocessing.image import load_img
# from keras.preprocessing.image import img_to_array

# from keras.models import model_from_json

# import pandas as pd
import threading

# import tensorflow as tf
import os
import sys
import glob
import shutil

# from keras.models import Sequential
# from keras.layers import Dense, Dropout, Flatten
# from keras.layers import Conv2D, MaxPooling2D
# from keras import backend as K
# from keras.callbacks import TensorBoard, ModelCheckpoint

# from keras.utils import plot_model
# import models
# from models import custom_loss
# import imutils


class CubosDetector:
    idx = 0
    deviation_from_cube = 0.0
    cubes = {}
    boundaries = [
	([17, 15, 100], [50, 56, 200]),     # red
	([86, 31, 4], [220, 88, 50]),       # blue
	([25, 146, 190], [62, 174, 250]),   # yellow
	([103, 86, 65], [145, 133, 128])    # gray
    ]

    dim = (224, 224)
    deviation = 0
    linear = 0
    angular_joy = 0

    def __init__(self, folder):
        rospack = rospkg.RosPack()
        pack_path = rospack.get_path('robocar')
        # model_path = pack_path + '/scripts/simple2'
        
        # get ros params
        self.img_topic = rospy.get_param('img_topic', default='/raspicam_node/image/compressed')
        print("-------------------")
        print(self.img_topic)
        print("-------------------")
        self.output_topic = rospy.get_param('output_topic', default='/cmd_vel')
        self.feedback_topic = rospy.get_param('feedback_topic', default='/real_twist')
        self.lowerBound=np.array([33,80,40])
        self.upperBound=np.array([102,255,255])
        self.bridge = CvBridge()

        # self.graph = tf.get_default_graph()

        print('creando subs y pubs...')
        # image subscriber for the predictor
        self.image_sub = rospy.Subscriber(self.img_topic, CompressedImage, self.imCallback, queue_size=1)
        
        self.feedback_sub = rospy.Subscriber(self.feedback_topic, Twist, queue_size=1)
        
        # float32 publisher for output 
        self.output_pub = rospy.Publisher(self.output_topic, Twist, queue_size=1)
        self.image_pub = rospy.Publisher("/out_image", Image, queue_size=1)

    #this callback executes when the two subscribers sync
    def imCallback(self, img):
        """ este calback lee la imagen de la camara, la preprocesa y obtiene 
        una prediccion para el comando de control del robot"""

        # lee la imagen y la preprocesa
        img = cv2.imdecode(np.fromstring(img.data, np.uint8),cv2.IMREAD_COLOR)
        
        imgHSV= cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        mask=cv2.inRange(imgHSV,self.lowerBound,self.upperBound)
        
        kernelOpen=np.ones((6,6))
        kernelClose=np.ones((20,20))
        maskOpen=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernelOpen)
        maskClose=cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)

        maskFinal = maskClose

        im2, conts, hierarchy = cv2.findContours(maskFinal.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        
        if not conts: 
            print("no conts!")
            return

        cv2.drawContours(img,conts,-1,(255,0,0),3)
        c = max(conts, key = cv2.contourArea)
        
        M = cv2.moments(c)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
    
        # draw the contour and center of the shape on the image
        cv2.drawContours(img, [c], -1, (0, 255, 0), 2)
        cv2.circle(img, (cX, cY), 7, (255, 255, 255), -1)

        area_txt = str(cv2.contourArea(c))
        
        self.deviation = int(cX - 160)

        deviation_txt = str(self.deviation)
        cv2.putText(img, area_txt, (cX - 20, cY - 20),
		        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.putText(img, deviation_txt, (cX - 20, cY + 20),
		        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    

        x,y,w,h = cv2.boundingRect(c)
        # draw the book contour (in green)
        cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        
        error = self.deviation * 0.001
        # calculo la salida
        ctrl_msg = Twist()
        ctrl_msg.angular.z = error
        self.output_pub.publish(ctrl_msg)


    def feedbackCallback(self, twist):
        # llega mensaje
        
        # calculo el error
        error = self.deviation * 0.1
        # calculo la salida
        ctrl_msg = Twist()
        ctrl_msg.angular.z = error
        self.output_pub.publish(ctrl_msg)
        # publico setpoint

        pass

def main(args):
    rospy.init_node('cubos_node', anonymous=True)
    stamper = CubosDetector(None)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
