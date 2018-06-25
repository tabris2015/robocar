
// #define USE_MOTOR_DRIVER_I2C

#include <RTIMULib.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h> // para el control
#if defined(USE_MOTOR_DRIVER_I2C)
    // libreria para el motor i2c
    #include "motor_driver_i2c.h" 
    #define MOTORS_ADD 0x0f
#endif
#include <sensor_msgs/Range.h> // para el sensor
// libreria para el sensor laser
#include "VL53L0X.h"



static const double G_TO_MPSS = 9.80665;


#if defined(USE_MOTOR_DRIVER_I2C)
    // motores
    MotorDriverI2c * motors_ptr;
#endif // USE_MOTOR_DRIVER_I2C


// sensor
VL53L0X sensor;
// prototipos

void twistCallback(const geometry_msgs::Twist& msg);

// 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "i2c_sensors_node");
    ROS_INFO("i2c sensors up!");
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

    #if defined(USE_MOTOR_DRIVER_I2C)
    ros::Subscriber sub = nh.subscribe("/robot/cmd_vel", 100, twistCallback);
    MotorDriverI2c motors(settings, MOTORS_ADD);
    motors_ptr = &motors;
    #endif // 


    // laser 
    sensor.initialize();
	sensor.setTimeout(200);

    ros::Publisher laser_pub = nh.advertise<sensor_msgs::Range>("laser", 1);
    
    sensor_msgs::Range laser_msg;
  
    laser_msg.radiation_type = 1;
    laser_msg.header.frame_id =  "laser";
    laser_msg.field_of_view = 0.1 ;
    laser_msg.min_range = 0.0;
    laser_msg.max_range = 1.20;

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

            imu_pub.publish(imu_msg);
        }
        uint16_t distance = sensor.readRangeSingleMillimeters();
        if (!sensor.timeoutOccurred()) {
			// create message and publish
            laser_msg.range = distance / 1000.0;
            laser_msg.header.stamp = ros::Time::now();
            laser_pub.publish(laser_msg);
		} else {
			ROS_WARN("Laser sensor timeout");
		}
        ros::spinOnce();
        ros::Duration(imu->IMUGetPollInterval() / 1000.0).sleep();
    }
    return 0;
}


void twistCallback(const geometry_msgs::Twist& msg)
{
    
    #if defined(USE_MOTOR_DRIVER_I2C)
    motors_ptr->SetUnicycleVelocities(msg.linear.x, msg.angular.z);
    #endif // USE_MOTOR_DRIVER_I2C
    return;
}