#!/usr/bin/env python
from __future__ import print_function
import roslib
import rospkg
import rospy
roslib.load_manifest('robocar')

from sensor_msgs.msg import Range
from std_msgs.msg import Float32
import sys
import csv

class ObstacleDetector:
    idx = 0
    # model_name = '/home/pepe/catkin_ws/src/robocar/scripts/simple2'
    dim = (224, 224)

    linear = 0
    angular_joy = 0
    max_acc = 0.3

    kp = 4.0

    def __init__(self, folder):
        # get ros params
        rospy.loginfo("Obstacle node init")
        self.range_topic = rospy.get_param('range_topic', default='/laser')
        self.output_topic = rospy.get_param('output_topic', default='/obstacle_output')
        self.stop_distance = rospy.get_param('stop_distance', default=0.15)
        # input subscriber for the predictor
        self.range_sub = rospy.Subscriber(self.range_topic, Range, self.RangeCallback, queue_size=1)
        
        # float32 publisher for output 
        self.output_pub = rospy.Publisher(self.output_topic, Float32, queue_size=1)
    
    #this callback executes when the two subscribers sync
    def RangeCallback(self, msg):
        """ este calback recibe el rango del sensor y calcula la salida 
        correspondiente
        """
        error = msg.range - self.stop_distance

        error = error * 2 if error < 0 else error 

        acceleration = self.kp * error


        acceleration = self.max_acc if acceleration > self.max_acc else acceleration

        acceleration = -self.max_acc * 2 if acceleration < -self.max_acc else acceleration

        acceleration = 0 if (acceleration < (self.stop_distance + 0.03)) and (acceleration > (self.stop_distance - 0.01)) else acceleration

        out_msg = Float32()
        out_msg.data = acceleration * 0.75
        self.output_pub.publish(out_msg)
        
        # if distance is less than 15 cm
        
        
def main(args):
    rospy.init_node('neural_node', anonymous=True)
    stamper = ObstacleDetector(None)
    rospy.spin()
   

if __name__ == '__main__':
    main(sys.argv)
