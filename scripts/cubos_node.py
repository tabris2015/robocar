#!/usr/bin/env python
from __future__ import print_function
import roslib
import rospkg
roslib.load_manifest('robocar')
import sys
import csv
import operator
import numpy as np
import rospy
import cv2
import message_filters
from std_msgs.msg import String, Float32, Bool
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

    current_color = ''

    color_areas = {}
    color_deviations = {}
    color_last_center = {}
    
    cGreen = []
    cBlue = []
    cRed = []

    last_color_deviations = {}
    last_state = 0

    first = False
    def __init__(self, folder):
        rospack = rospkg.RosPack()
        pack_path = rospack.get_path('robocar')
        # model_path = pack_path + '/scripts/simple2'
        
        # get ros params
        self.img_topic = rospy.get_param('img_topic', default='/raspicam_node/image/compressed')
        print("-------------------")
        print(self.img_topic)
        print("-------------------")
        self.output_topic = rospy.get_param('output_topic', default='/follow_color_cmd_vel')
        self.feedback_topic = rospy.get_param('feedback_topic', default='/real_twist')
        self.lowerGreen=np.array([40,80,40])
        self.upperGreen=np.array([80,255,255])

        self.colors_dic = { "g":([40,80,40],[80,255,255]),
                            "b":([95,80,40],[120,255,255]),
                            "r1":([0,80,40],[10,255,255]),
                            "r2":([160,80,40],[180,255,255]),}
        self.bridge = CvBridge()

        # self.graph = tf.get_default_graph()

        print('creando subs y pubs...')
        # image subscriber for the predictor
        self.image_sub = rospy.Subscriber(self.img_topic, CompressedImage, self.imCallback, queue_size=1)
        
        # self.feedback_sub = rospy.Subscriber(self.feedback_topic, Twist, queue_size=1)
        
        # float32 publisher for output 
        self.output_pub = rospy.Publisher(self.output_topic, Twist, queue_size=1)
        self.image_pub = rospy.Publisher("/out_image", Image, queue_size=1)
        self.cube_pub = rospy.Publisher("/cube_detected", Bool, queue_size=1)
        self.cube_color_pub = rospy.Publisher("/cube_color", String, queue_size=1)

    #this callback executes when the two subscribers sync
    def imCallback(self, img):
        """ este calback lee la imagen de la camara, la preprocesa y obtiene 
        una prediccion para el comando de control del robot"""
        self.color_areas = {}
        self.color_deviations = {}
        kernelClose=np.ones((20,20))
        kernelOpen=np.ones((6,6))

        ctrl_msg = Twist()
        ctrl_msg.angular.z = 0
        ctrl_msg.linear.x = 0

        # lee la imagen y la preprocesa
        img = cv2.imdecode(np.fromstring(img.data, np.uint8),cv2.IMREAD_COLOR)
        ## para deteccion de linea
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        imgHSV= cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        # detectar los 3 colores
        # green
        maskGreen=cv2.inRange(imgHSV,np.array(self.colors_dic['g'][0]),np.array(self.colors_dic['g'][1]))
        
        maskOpen=cv2.morphologyEx(maskGreen,cv2.MORPH_OPEN,kernelOpen)
        maskClose=cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)

        maskGreenFinal = maskClose

        im2, contsGreen, hierarchy = cv2.findContours(maskGreenFinal.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        
        # Blue
        maskBlue=cv2.inRange(imgHSV,np.array(self.colors_dic['b'][0]),np.array(self.colors_dic['b'][1]))
        
        maskOpen=cv2.morphologyEx(maskBlue,cv2.MORPH_OPEN,kernelOpen)
        maskClose=cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)

        maskBlueFinal = maskClose

        im2, contsBlue, hierarchy = cv2.findContours(maskBlueFinal.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)

        # Blue
        maskRed1=cv2.inRange(imgHSV,np.array(self.colors_dic['r1'][0]),np.array(self.colors_dic['r1'][1]))
        maskRed2=cv2.inRange(imgHSV,np.array(self.colors_dic['r2'][0]),np.array(self.colors_dic['r2'][1]))
        
        maskRed=cv2.bitwise_or(maskRed1, maskRed2)
        
        maskOpen=cv2.morphologyEx(maskRed,cv2.MORPH_OPEN,kernelOpen)
        maskClose=cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)

        maskRedFinal = maskClose

        im2, contsRed, hierarchy = cv2.findContours(maskRedFinal.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)


        if contsGreen:
            cGreen = max(contsGreen, key = cv2.contourArea)
            area = int(cv2.contourArea(cGreen))
            if area > 220:
                self.color_areas['green'] = area                      # <-----------------
                M = cv2.moments(cGreen)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                cv2.circle(img, (cX, cY), 7, (255, 255, 255), -1)
                area_txt = str(area)
                self.color_deviations['green'] = int(160 - cX)          # <-----------------
                deviation_txt = str(self.color_deviations['green'])
                cv2.putText(img, area_txt, (cX - 20, cY - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(img, deviation_txt, (cX - 20, cY + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                # self.current_color = "green"
                self.color_last_center['green'] = (cX, cY)
                self.last_color_deviations['green'] = self.color_deviations['green']
            # error = self.color_deviations['green'] * 0.003
                # calculo la salida

        if contsBlue:
            cBlue = max(contsBlue, key = cv2.contourArea)
            area = int(cv2.contourArea(cBlue))
            if area > 220:
                self.color_areas['blue'] = area
                M = cv2.moments(cBlue)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                cv2.circle(img, (cX, cY), 7, (255, 255, 255), -1)
                area_txt = str(area)
                self.color_deviations['blue'] = int(160 - cX)           #<-----------
                deviation_txt = str(self.color_deviations['blue'])
                cv2.putText(img, area_txt, (cX - 20, cY - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(img, deviation_txt, (cX - 20, cY + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                # self.current_color = "blue"
                self.color_last_center['blue'] = (cX, cY)
                self.last_color_deviations['blue'] = self.color_deviations['blue']


        if contsRed:
            cRed = max(contsRed, key = cv2.contourArea)
            area = int(cv2.contourArea(cRed))
            if area > 220:
                self.color_areas['red'] = area
                M = cv2.moments(cRed)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                cv2.circle(img, (cX, cY), 7, (255, 255, 255), -1)
                area_txt = str(area)
                self.color_deviations['red'] = int(160 - cX)           #<-----------
                deviation_txt = str(self.color_deviations['red'])
                cv2.putText(img, area_txt, (cX - 20, cY - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(img, deviation_txt, (cX - 20, cY + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                # self.current_color = "red"
                self.color_last_center['red'] = (cX, cY)
                self.last_color_deviations['red'] = self.color_deviations['red']
        # print(len(self.color_areas.keys()), "objetos en escena")
        

        if self.color_areas:
            if self.first:
                self.cube_pub.publish(True)
                self.first = False

            col = max(self.color_areas.iteritems(), key=operator.itemgetter(1))[0]
            print("siguiendo: ", col)
            self.current_color = col
            error = self.color_deviations[col] * 0.003

            ctrl_msg.angular.z = error
            ctrl_msg.linear.x = 0.1
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        else:

            if (
                    (self.current_color == "green") and 
                    (abs(self.last_color_deviations["green"]) < 50) and 
                    (self.color_last_center["green"][1] > 200)
                ):
                print("conseguido cubo VERDE!")

                self.cube_color_pub.publish("G")

            elif (
                    (self.current_color == "blue") and 
                    (abs(self.last_color_deviations["blue"]) < 50) and
                    (self.color_last_center["blue"][1] > 200)
                ):
                print("conseguido cubo AZUL!")
                self.cube_color_pub.publish("B")

            elif (
                    (self.current_color == "red") and 
                    (abs(self.last_color_deviations["red"]) < 50) and
                    (self.color_last_center["red"][1] > 200)
                ):
                print("conseguido cubo ROJO!")
                self.cube_color_pub.publish("R")

            self.first = True

        self.output_pub.publish(ctrl_msg)
        ########################################################################
        ############ LINE FOLLOWING ############################################
        ########################################################################
        blur = cv2.GaussianBlur(gray,(5,5),0)
        ret,thresh = cv2.threshold(blur,60,255,cv2.THRESH_BINARY_INV)
        contours,hierarchy = cv2.findContours(thresh.copy(), 1, cv2.CHAIN_APPROX_NONE)
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.line(img,(cx,0),(cx,720),(255,0,0),1)
            cv2.line(img,(0,cy),(1280,cy),(255,0,0),1)
            cv2.drawContours(img, contours, -1, (0,255,0), 1)
            if cx >= 120:
                print("Turn Left!")
            if cx < 120 and cx > 50:
                print("On Track!")
            if cx <= 50:
                print("Turn Right")

            self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
                
        

    def feedbackCallback(self, twist):
        # llega mensaje
        
        # calculo el error
        error = self.deviation * 0.1
        # calculo la salida
        ctrl_msg = Twist()
        ctrl_msg.angular.z = error
        self.output_pub.publish(ctrl_msg)
        # publico setpoint

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
