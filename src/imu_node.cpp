
#include <RTIMULib.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h> // para el control
#include <std_msgs/Float32.h>       // para el setpoint

// #include "motor_driver_i2c.h"

#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"

#include "geometry_msgs/Vector3.h"

#define MOTORS_ADD 0x0f
static const double G_TO_MPSS = 9.80665;

float error;
float last_error;
float output;
float setpoint;
float curr_heading;
float last_yaw;

float integral_term = 0;

float kp=0.8, ki=0.01, kd=0.3;

//
// MotorDriverI2c * motors_ptr;
// prototipos
void angleCallback(const std_msgs::Float32::ConstPtr &angle)
{
    // calcular error y salida
    setpoint = angle->data;
}
// 


int main(int argc, char **argv)
{
    // for transforming to ypr
    double roll, pitch, yaw;

    ros::init(argc, argv, "imu_node");
    ROS_INFO("imu node up!");
    ros::NodeHandle nh("~");

    std::string calibration_file_path;
    if(!nh.getParam("calibration_file_path", calibration_file_path))
    {
        ROS_ERROR("The calibration_file_path parameter must be set to use a "
                  "calibration file.");
        ROS_BREAK();
    }

    std::string calibration_file_name = "RTIMULib";
    if(!nh.getParam("calibration_file_name", calibration_file_name))
    {
        ROS_WARN_STREAM("No calibration_file_name provided - default: "
                        << calibration_file_name);
    }

    std::string frame_id = "imu_link";
    if(!nh.getParam("frame_id", frame_id))
    {
        ROS_WARN_STREAM("No frame_id provided - default: " << frame_id);
    }

    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 1);
    ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    
    ros::Subscriber setpoint_sub = nh.subscribe("/angle", 100, angleCallback);

    // Load the RTIMULib.ini config file
    // Settings also is the i2c bus handler
    RTIMUSettings *settings = new RTIMUSettings(calibration_file_path.c_str(),
                                                calibration_file_name.c_str());

    RTIMU *imu = RTIMU::createIMU(settings);

    if ((imu == NULL) || (imu->IMUType() == RTIMU_TYPE_NULL))
    {
        ROS_ERROR("No Imu found");
        ROS_BREAK();
    }

    // Initialise the imu object
    imu->IMUInit();

    // Set the Fusion coefficient
    imu->setSlerpPower(0.02);
    // Enable the sensors
    imu->setGyroEnable(true);
    imu->setAccelEnable(true);
    imu->setCompassEnable(true);

    sensor_msgs::Imu imu_msg;

    // end IMU init

    // MotorDriverI2c motors(settings, MOTORS_ADD);
    // motors_ptr = &motors;

    while (ros::ok())
    {
        if (imu->IMURead())
        {
            RTIMU_DATA imu_data = imu->getIMUData();

            imu_msg.header.stamp = ros::Time::now();
            imu_msg.header.frame_id = frame_id;

            imu_msg.orientation.x = imu_data.fusionQPose.x(); 
            imu_msg.orientation.y = imu_data.fusionQPose.y(); 
            imu_msg.orientation.z = imu_data.fusionQPose.z(); 
            imu_msg.orientation.w = imu_data.fusionQPose.scalar(); 

            imu_msg.angular_velocity.x = imu_data.gyro.x();
            imu_msg.angular_velocity.y = imu_data.gyro.y();
            imu_msg.angular_velocity.z = imu_data.gyro.z();

            imu_msg.linear_acceleration.x = imu_data.accel.x() * G_TO_MPSS;
            imu_msg.linear_acceleration.y = imu_data.accel.y() * G_TO_MPSS;
            imu_msg.linear_acceleration.z = imu_data.accel.z() * G_TO_MPSS;
            float yaw   = atan2(
                            2.0 * (
                                    imu_data.fusionQPose.z() * imu_data.fusionQPose.scalar() + 
                                    imu_data.fusionQPose.x() * imu_data.fusionPose.y()
                                ) , 
                                - 1.0 + 2.0 * (
                                    imu_data.fusionQPose.scalar() * imu_data.fusionQPose.scalar() + 
                                    imu_data.fusionQPose.x() * imu_data.fusionQPose.x()
                                ));

            error = setpoint - yaw;

            ROS_INFO_STREAM("yaw: " << yaw << " error: " << error);
            
            integral_term += error;

            float derivative_term = (yaw - last_yaw);
            
            last_yaw = yaw;
            
            float integral_final = integral_term * ki;
            
            if(integral_final > 10.0) integral_final = 10.0;
            if(integral_final < -10.0) integral_final = -10.0;
            

            output = kp * error + integral_final + kd * derivative_term;
            
            geometry_msgs::Twist twist;
            twist.angular.z = output;
            twist_pub.publish(twist);

            imu_pub.publish(imu_msg);
        }
        ros::spinOnce();
        ros::Duration(imu->IMUGetPollInterval() / 1000.0).sleep();
    }
    return 0;
}

