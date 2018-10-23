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


TeleopRobocar::TeleopRobocar():
  linear_(5),
  brake_(2),
  angular_(0)
{

  nh_.param<int>("axis_linear", linear_, 5);
  nh_.param<int>("axis_brake", brake_, 2);
  nh_.param<double>("scale_linear", l_scale_, -0.5);
  nh_.param<double>("offset_linear", l_offset_, 0.5);

  
  nh_.param<int>("axis_angular", angular_, 0);

  nh_.param<double>("scale_angular", a_scale_, -0.34);



  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  velStamped_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/stamped_cmd_vel", 1);
  
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &TeleopRobocar::joyCallback, this);

 
}

void TeleopRobocar::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
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

void TeleopRobocar::timerCallback(const ros::TimerEvent& event)
{
  stamped.twist = twist;
  stamped.header.stamp = ros::Time::now();

  vel_pub_.publish(twist);
  velStamped_pub_.publish(stamped);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_teleop");
  TeleopRobocar teleop_joy;
  // 20 hz timer for publishing twist stamped
  teleop_joy.timer = teleop_joy.nh_.createTimer(ros::Duration(0.05),&TeleopRobocar::timerCallback, &teleop_joy);
  //teleop_joy.timer = teleop_joy.nh_.createTimer(ros::Duration(0.02),&TeleopRobocar::timerCallback, &teleop_joy);


  ros::spin();
}