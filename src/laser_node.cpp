#include "VL53L0X.h"

#include <ros/ros.h>
#include <sensor_msgs/Range.h> // para el sensor
// libreria para el sensor laser
class LaserNode
{
  public:
    LaserNode();

    ros::Timer timer;

    ros::NodeHandle nh_;
    void timerCallback(const ros::TimerEvent &event);
    float GetInterval();
  private:
    // sensor
    VL53L0X sensor_;

    int freq_hz_;
    double l_scale_;
    double a_scale_;
    double l_offset_;

    ros::Publisher laser_pub_;
    // msgs
    sensor_msgs::Range laser_msg_;
};

LaserNode::LaserNode()
{
    nh_.param<int>("freq_hz", freq_hz_, 20);
    if (freq_hz_ <= 0)
    {
        ROS_WARN("Invalid frequency value, default: 20Hz");
        freq_hz_ = 20;
    }

    // init publisher
    laser_pub_ = nh_.advertise<sensor_msgs::Range>("laser", 1);
    
    // laser
    sensor_.initialize();
    sensor_.setTimeout(200);
    
    // init some fixed data 
    laser_msg_.radiation_type = 1;
    laser_msg_.header.frame_id = "laser";
    laser_msg_.field_of_view = 0.1;
    laser_msg_.min_range = 0.0;
    laser_msg_.max_range = 1.20;
}

float LaserNode::GetInterval()
{
    return 1.0 / freq_hz_;
}

void LaserNode::timerCallback(const ros::TimerEvent &event)
{
    uint16_t distance = sensor_.readRangeSingleMillimeters();

    if (!sensor_.timeoutOccurred())
    {
        // create message and publish
        laser_msg_.range = distance / 1000.0;
        laser_msg_.header.stamp = ros::Time::now();
        laser_pub_.publish(laser_msg_);
    }
    else
    {
        ROS_WARN("Laser sensor timeout");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_node");
    LaserNode laser_node;
    
    laser_node.timer = laser_node.nh_.createTimer(ros::Duration(laser_node.GetInterval()), 
                                                &LaserNode::timerCallback, 
                                                &laser_node);
    ros::spin();
}