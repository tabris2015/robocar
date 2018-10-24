#include <ros/ros.h>
#include <boost/thread.hpp>
#include <std_msgs/Float32.h>

#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Joy.h>

class Pilot
{

  private:
    // data
    bool manual_;
    // topics from param server
    std::string neural_topic_;
    std::string obstacle_topic_;
    std::string joy_twist_topic_;

    std::string output_topic_;

    int freq_hz_;
    // messages
    geometry_msgs::Twist twist_msg_;          // simple twist message from joystick
    geometry_msgs::Twist manual_twist_msg_;          // simple twist message from joystick
    // geometry_msgs::TwistStamped stamped; // stamped twist for sync

    // subs and pubs
    ros::Subscriber neural_sub_;
    ros::Subscriber obstacle_sub_;
    ros::Subscriber joy_twist_sub_;

    ros::Publisher twist_cmd_pub_;

    // subs callback
    // for neural steering
    void NeuralCallback(const std_msgs::Float32::ConstPtr &steering);

    // for obstacle node
    void ObstacleCallback(const std_msgs::Float32::ConstPtr &acceleration);

    // for joy teleop
    void JoyTwistCallback(const geometry_msgs::Twist::ConstPtr &joy_twist);

    // for find object 2d
    void ObjectCallback(const std_msgs::Float32::ConstPtr &steering);

  public:
    Pilot();

    ros::Timer timer;

    ros::NodeHandle nh_;
    void TimerCallback(const ros::TimerEvent &event);
    float GetInterval();
};
// constructor
Pilot::Pilot() : manual_(false)
{
    ROS_INFO("Iniciando nodos");

    nh_.param<int>("freq_hz", freq_hz_, 20);
    if (freq_hz_ <= 0)
    {
        ROS_WARN("Invalid frequency value, default: 20Hz");
        freq_hz_ = 20;
    }

    nh_.param<std::string>("neural_topic", neural_topic_, "/neural_output");
    nh_.param<std::string>("obstacle_topic", obstacle_topic_, "/obstacle_output");
    nh_.param<std::string>("joy_twist_topic", joy_twist_topic_, "/joy_cmd_vel");

    nh_.param<std::string>("output_topic", output_topic_, "/cmd_vel");
    
    // subscribe to nodes
    ROS_INFO("Creando subs y pubs");
    neural_sub_ = nh_.subscribe<std_msgs::Float32>(neural_topic_, 1, &Pilot::NeuralCallback, this);
    obstacle_sub_ = nh_.subscribe<std_msgs::Float32>(obstacle_topic_, 1, &Pilot::ObstacleCallback, this);
    joy_twist_sub_ = nh_.subscribe<geometry_msgs::Twist>(joy_twist_topic_, 1, &Pilot::JoyTwistCallback, this);

    twist_cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(output_topic_, 1);

}

float Pilot::GetInterval()
{
    return 1.0 / freq_hz_;
}

///////////////////////////////////////
// CALLBACKS //////////////////////////
///////////////////////////////////////

void Pilot::NeuralCallback(const std_msgs::Float32::ConstPtr &steering)
{
    // extract the command
    // put in the message
    // double angular = steering;
    twist_msg_.angular.z = steering->data;
}

void Pilot::ObstacleCallback(const std_msgs::Float32::ConstPtr &acceleration)
{
    // double linear = acceleration;
    twist_msg_.linear.x = acceleration->data;
}

void Pilot::JoyTwistCallback(const geometry_msgs::Twist::ConstPtr &joy_twist)
{
    manual_ = true;
    manual_twist_msg_.linear.x = joy_twist->linear.x;
    manual_twist_msg_.angular.z = joy_twist->angular.z;
}

void Pilot::TimerCallback(const ros::TimerEvent &event)
{
    if(manual_)
    {
        twist_cmd_pub_.publish(manual_twist_msg_);
        manual_ = false;
    }
    else
    {
        twist_cmd_pub_.publish(twist_msg_);
        manual_ = false;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pilot_node");
    Pilot teleop_joy;

    teleop_joy.timer = teleop_joy.nh_.createTimer(ros::Duration(teleop_joy.GetInterval()),
                                                    &Pilot::TimerCallback,
                                                    &teleop_joy);
    ros::spin();
}