#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('minesweeper')
import sys
import csv
import numpy as np
import rospy
import cv2
import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import Image, Range, CompressedImage, Joy
from geometry_msgs.msg import Twist, TwistStamped
from cv_bridge import CvBridge, CvBridgeError

from keras.preprocessing.image import load_img
from keras.preprocessing.image import img_to_array

from keras.models import model_from_json

import pandas as pd
import threading

import tensorflow as tf
import os
import sys
import glob
import shutil

from keras.models import Sequential
from keras.layers import Dense, Dropout, Flatten
from keras.layers import Conv2D, MaxPooling2D
from keras import backend as K
from keras.callbacks import TensorBoard, ModelCheckpoint

from keras.utils import plot_model
import models
from models import custom_loss
import imutils



class AutoPilot:
    idx = 0
    img_topic = '/camera/image/compressed'
    #img_topic = '/usb_cam/image_raw/compressed'
    twist_topic = '/robot/cmd_vel'
    sign_topic = '/robot/sign_event'
    joy_topic = 'joy'
    model_name = '/home/pepe/catkin_ws/src/minesweeper/scripts/simple2'
    dim = (224, 224)

    linear = 0
    angular_joy = 0

    def __init__(self, folder):
        
        ## cargar la red neuronal en la memoria 
        # load json and create model
        json_file = open(self.model_name + '.json', 'r')
        loaded_model_json = json_file.read()
        json_file.close()
        self.model = model_from_json(loaded_model_json)

        # load weights into new model
        self.model.load_weights(self.model_name + "_best.h5")
        print("Loaded model from disk") 
        self.model.compile(loss='mse', optimizer='adam', metrics=['accuracy'])
        self.model.summary()
        self.graph = tf.get_default_graph()

        print('creando subs y pubs...')
        self.image_sub = rospy.Subscriber(self.img_topic, CompressedImage, self.imCallback, queue_size=1)
        self.sign_sub = rospy.Subscriber(self.sign_topic, String, self.signCallback, queue_size=1)
        self.joy_sub = rospy.Subscriber(self.joy_topic, Joy, self.joyCallback, queue_size=1)

        self.twist_pub = rospy.Publisher(self.twist_topic, Twist, queue_size=1)
    
    #this callback executes when the two subscribers sync
    def imCallback(self, img):
        """ este calback lee la imagen de la camara, la preprocesa y obtiene 
        una prediccion para el comando de control del robot"""

        # lee la imagen y la preprocesa
        np_image = cv2.imdecode(np.fromstring(img.data, np.uint8),cv2.IMREAD_COLOR)
        np_image = cv2.resize(np_image, self.dim, interpolation = cv2.INTER_AREA)
        #print (np_image.shape)
        np_image = (2 * (np_image / 255.0 - 0.5))
        x = np_image.reshape((1,) + np_image.shape)  # this is a Numpy array with shape (1, h,w, c)
        #print (x.shape)
        # obtiene la prediccion de la red neuronal
        
        with self.graph.as_default():
            angular = self.model.predict(x, batch_size=1, verbose=0)
        angular = np.asscalar(angular.flatten())
        #print ("prediccion: ", angular)
        # crea el mensaje para el control del carro y publica 
        msg = Twist()
        # permite control manual en caso de emergencia
        if abs(self.angular_joy) < 0.1:
            msg.angular.z = angular
        else:
            msg.angular.z = self.angular_joy

        msg.linear.x = self.linear
        self.twist_pub.publish(msg)
    
    def signCallback(self, event):
        # este callback debe implementar la logica de control
        # para signos de transito, en principio, solo ALTO
        pass
    
    def joyCallback(self, joy):
        #control manual de velocidad con el joystick
        self.angular_joy = joy.axes[0]
        self.linear = (joy.axes[5] * (-0.5) + 0.5) + (joy.axes[2] - 1)
        #print(self.linear)
        
def main(args):
    rospy.init_node('autopilot', anonymous=True)
    stamper = AutoPilot(None)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
