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
from sensor_msgs.msg import Image, Range, CompressedImage
from geometry_msgs.msg import Twist, TwistStamped
from cv_bridge import CvBridge, CvBridgeError

class ImgStamp:
    idx = 0
    img_topic = '/camera/image/compressed'
    stamped_topic = '/robot/stamped_image/compressed'
    def __init__(self, folder):
        self.image_pub = rospy.Publisher(self.stamped_topic, CompressedImage)
        self.image_sub = rospy.Subscriber(self.img_topic, CompressedImage, self.callback, queue_size=1)

    
    #this callback executes when the two subscribers sync
    def callback(self, img):
        msg = CompressedImage()         
        msg.header.stamp = rospy.Time.now()
        msg.format = 'jpeg'
        msg.data = img.data
        self.image_pub.publish(msg)

        
def main(args):
    rospy.init_node('img_stamp', anonymous=True)
    stamper = ImgStamp(None)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
