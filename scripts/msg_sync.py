#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('robocar')
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

class RobocarLogger:
    idx = 0
    folder = 'dataset/img/'
    target_file = "dataset/target.csv"
    img_topic = '/robot/stamped_image/compressed'
    twist_topic = '/robot/stamped_cmd_vel'
    def __init__(self, folder):
        if(folder != None):
            self.folder = folder
        self.bridge = CvBridge()
        self.string_pub_ = rospy.Publisher('sync_msg', String, queue_size=2)
        # command subscriber
        if rospy.has_param('twist_topic'):
            self.twist_topic = rospy.get_param('twist_topic')
            rospy.loginfo('Parameter %s has value %s', rospy.resolve_name('twist_topic'), self.twist_topic)
            
        self.twist_sub_ = message_filters.Subscriber(self.twist_topic, TwistStamped)
        # image subscriber
        if rospy.has_param('/robot/img_topic'):
            self.img_topic = rospy.get_param('/robot/img_topic')
            rospy.loginfo('Parameter %s has value %s', rospy.resolve_name('img_topic'), self.img_topic)
            
        self.img_sub_ = message_filters.Subscriber(self.img_topic, CompressedImage)
        
        # sync topics
        self.ts = message_filters.ApproximateTimeSynchronizer([self.img_sub_, self.twist_sub_],5, 0.5)
        print('registering callback...')
        self.ts.registerCallback(self.callback)
        #self.grayscale = rospy.get_param("grayscale")
        
        print('opening file for targets...')
        # init table for targets and ids
        with open(self.target_file, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(['id', 'linear', 'angular', 'target'])

        print('topics: ', self.img_topic, ' -- ', self.twist_topic)
    
    def save_entry(self, image, twist):
        
        pass

    #this callback executes when the two subscribers sync
    def callback(self, image, twist):
        #self.string_pub_.publish('hola...')
        # for image
        #cv_image = self.bridge.imgmsg_to_cv2(image, 'mono8')
        
        # for compressed image
        print('messages synced!')
        np_image = cv2.imdecode(np.fromstring(image.data, np.uint8),cv2.IMREAD_COLOR)
        
        dim = (224, 224)
        np_image = cv2.resize(np_image, dim, interpolation = cv2.INTER_AREA)
        #if self.grayscale != None:
        #    np_image = cv2.imdecode(np_image,cv2.IMREAD_GRAYSCALE)

        filename = "dataset/" + str(self.idx) + ".bmp";
        print("saving file: ",filename, " size: ", np_image.shape)

        cv2.imwrite(filename, np_image)
        target_data = [self.idx, twist.twist.linear.x, twist.twist.angular.z, [twist.twist.linear.x, twist.twist.angular.z]]
        with open(self.target_file, 'a') as f:
            writer = csv.writer(f)
            writer.writerow(target_data)

        self.idx += 1

        print('data saved!')
        
def main(args):
    rospy.init_node('msg_sync', anonymous=True)
    logger = RobocarLogger(None)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
