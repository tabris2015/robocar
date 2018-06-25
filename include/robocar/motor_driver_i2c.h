
#ifndef MOTOR_DRIVER_I2C_H
#define MOTOR_DRIVER_I2C_H

#include "RTIMUSettings.h"

#define REQ_WRITE_BYTE   0x34

#define MAX_LINEAR 0.75

#define MAX_ANGULAR 0.75

class RTIMUSettings;


class MotorDriverI2c
{
private:
    /* data */
    RTIMUSettings * m_settings;
    unsigned char address_;
    char error_buffer_[50];
    unsigned char linear_index_;
    unsigned char angular_index_;
    unsigned char req_byte_;
    float linear_;
    float angular_;
    int linear_bytes_;
    int angular_bytes_;
    
    bool WriteToDriver(unsigned char * data, unsigned char size);
public:
    MotorDriverI2c(RTIMUSettings *settings, unsigned char address);

    bool SetUnicycleVelocities(float linear, float angular);
    
    ~MotorDriverI2c();
};


#endif // MOTOR_DRIVER_I2C_H

