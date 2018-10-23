#include <ros/ros.h>
#include <boost/thread.hpp>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Joy.h>

class TeleopRobot
{
public:
  TeleopRobot();

  ros::Timer timer;
  
  ros::NodeHandle nh_;
  void timerCallback(const ros::TimerEvent& event);

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  int linear_;      // axis id
  int brake_;
  int angular_;

  double l_scale_;
  double a_scale_;
  double l_offset_;

  ros::Publisher vel_pub_;
  ros::Publisher velStamped_pub_;
  ros::Subscriber joy_sub_;

  // for callbacks
  geometry_msgs::Twist twist; // simple twist message from joystick
  geometry_msgs::TwistStamped stamped; // stamped twist for sync

};


TeleopRobot::TeleopRobot():
  linear_(5),
  brake_(2),
  angular_(0)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_brake", brake_, brake_);
  nh_.param("scale_linear", l_scale_, l_scale_);
  nh_.param("offset_linear", l_offset_, l_offset_);

  
  nh_.param("axis_angular", angular_, angular_);

  nh_.param("scale_angular", a_scale_, a_scale_);



  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  velStamped_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/stamped_cmd_vel", 1);
  
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopRobot::joyCallback, this);

 
}

void TeleopRobot::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

  twist.angular.z = joy->axes[angular_];
  double acceleration = joy->axes[linear_] * l_scale_ + l_offset_;
  double reverse = joy->axes[brake_] * l_scale_ + l_offset_;
  twist.linear.x = acceleration - reverse;
  
  vel_pub_.publish(twist);
}

void TeleopRobot::timerCallback(const ros::TimerEvent& event)
{
  stamped.twist = twist;
  stamped.header.stamp = ros::Time::now();

  velStamped_pub_.publish(stamped);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_robot");
  TeleopRobot teleop_turtle;
  // 50 hz timer for publishing twist stamped
  teleop_turtle.timer = teleop_turtle.nh_.createTimer(ros::Duration(0.02),&TeleopRobot::timerCallback, &teleop_turtle);

  ros::spin();
}