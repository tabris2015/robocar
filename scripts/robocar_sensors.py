#!/usr/bin/env python

import rospy
import VL53L0X
from sensor_msgs.msg import Range

class Sensors(object):
    range_topic = "/robot/laser"
    frameid = "/infrared"
    def __init__(self):
        # objetos utiles
        self.laser = VL53L0X.VL53L0X()
        self.laser.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)
        timing = self.laser.get_timing()

        if (timing < 20000):
            timing = 20000
        rospy.loginfo("Timing %d ms" % (timing/1000))
        #ros pubs y subs
        self.pub = rospy.Publisher(self.range_topic, Range, queue_size=5)
        # create message
        self.msg = Range()
        self.msg.radiation_type = Range.INFRARED
        self.msg.header.frame_id =  self.frameid
        self.msg.field_of_view = 0.1  # fake
        self.msg.min_range = 0.0
        self.msg.max_range = 1.47

    def run(self, hz=10):
        rospy.init_node('laser_range', anonymous=True)
        rate = rospy.Rate(hz) # 10 H
        while not rospy.is_shutdown():
            # construct message
            self.msg.range = self.laser.get_distance() / 1000.0 # in meters
            self.msg.header.stamp = rospy.get_rostime()
            rospy.loginfo(self.msg.range)
            self.pub.publish(self.msg)
            rate.sleep()

        self.laser.stop_ranging()

if __name__ == '__main__':
    node = Sensors()
    node.run()