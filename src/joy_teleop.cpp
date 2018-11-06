#include <ros/ros.h>
#include <boost/thread.hpp>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Joy.h>

class TeleopRobocar
{
  public:
    TeleopRobocar();

    ros::Timer timer;

    ros::NodeHandle nh_;
    void timerCallback(const ros::TimerEvent &event);
    float GetInterval();

  private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);

    int linear_; // axis id
    int brake_;
    int angular_;

    double l_scale_;
    double a_scale_;
    double l_offset_;

    std::string output_topic;

    int freq_hz_;

    ros::Publisher vel_pub_;
    ros::Publisher velStamped_pub_;
    ros::Subscriber joy_sub_;

    // for callbacks
    geometry_msgs::Twist twist;          // simple twist message from joystick
    geometry_msgs::TwistStamped stamped; // stamped twist for sync
};

TeleopRobocar::TeleopRobocar()
{
    nh_.param<int>("freq_hz", freq_hz_, 20);
    if (freq_hz_ <= 0)
    {
        ROS_WARN("Invalid frequency value, default: 20Hz");
        freq_hz_ = 20;
    }

    nh_.param<int>("axis_linear", linear_, 5);
    nh_.param<int>("axis_brake", brake_, 2);
    nh_.param<double>("scale_linear", l_scale_, -0.5);
    nh_.param<double>("offset_linear", l_offset_, 0.5);

    nh_.param<int>("axis_angular", angular_, 0);

    nh_.param<double>("scale_angular", a_scale_, -1.32);    // -0.34

    nh_.param<std::string>("output_topic", output_topic, "/cmd_vel");

    vel_pub_ = nh_.advertise<geometry_msgs::Twist>(output_topic, 1);
    
    velStamped_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/stamped_cmd_vel", 1);

    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &TeleopRobocar::joyCallback, this);
}

float TeleopRobocar::GetInterval()
{
    return 1.0 / freq_hz_;
}

void TeleopRobocar::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{

    twist.angular.z = joy->axes[angular_] * a_scale_;

    double acceleration = l_scale_ * joy->axes[linear_] + l_offset_;
    ROS_DEBUG("%f", acceleration);

    double reverse = l_scale_ * joy->axes[brake_] + l_offset_;
    ROS_DEBUG("%f", reverse);

    twist.linear.x = acceleration - reverse;

    ROS_DEBUG("%f", twist.linear.x);

    // vel_pub_.publish(twist);
}

void TeleopRobocar::timerCallback(const ros::TimerEvent &event)
{
    stamped.twist = twist;
    stamped.header.stamp = ros::Time::now();

    vel_pub_.publish(twist);
    velStamped_pub_.publish(stamped);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_teleop");
    TeleopRobocar teleop_joy;

    teleop_joy.timer = teleop_joy.nh_.createTimer(ros::Duration(teleop_joy.GetInterval()),
                                                    &TeleopRobocar::timerCallback,
                                                    &teleop_joy);
    ros::spin();
}