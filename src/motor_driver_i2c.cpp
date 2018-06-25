
#include "motor_driver_i2c.h"


MotorDriverI2c::MotorDriverI2c(RTIMUSettings * settings, unsigned char address):
m_settings(settings), address_(address), linear_index_(0x4), angular_index_(0x5)
{
    req_byte_ = REQ_WRITE_BYTE;
}

bool MotorDriverI2c::SetUnicycleVelocities(float linear, float angular)
{
    // convert float data to int
    float aux_linear = linear;
    float aux_angular = angular;
    if(aux_linear > MAX_LINEAR) aux_linear = MAX_LINEAR;
    if(aux_linear < -MAX_LINEAR) aux_linear = -MAX_LINEAR;
    
    if(aux_angular > MAX_ANGULAR) aux_angular = MAX_ANGULAR;
    if(aux_angular < -MAX_ANGULAR) aux_angular = -MAX_ANGULAR;
    
    
    linear_bytes_ = (int)(linear * 65536);
    angular_bytes_ = (int)(angular * 65536);
    
    /// LINEAR
    // request write (1)
    m_settings->HALWrite(address_, req_byte_, 0, NULL, error_buffer_);
    // write register index (1)
    m_settings->HALWrite(address_, linear_index_, 0, NULL, error_buffer_);

    m_settings->HALWrite(address_, *((unsigned char *)(&linear_bytes_)), 3, (unsigned char *)(&linear_bytes_) + 1, error_buffer_);
    /// ANGULAR
    // request write (1)
    m_settings->HALWrite(address_, req_byte_, 0, NULL, error_buffer_);
    // write register index (1)
    m_settings->HALWrite(address_, angular_index_, 0, NULL, error_buffer_);

    m_settings->HALWrite(address_, *((unsigned char *)(&angular_bytes_)), 3, (unsigned char *)(&angular_bytes_) + 1, error_buffer_);
    return true;
}

bool MotorDriverI2c::WriteToDriver(unsigned char * data, unsigned char size)
{
    // programar la secuencia de escritura
    // request write (1)

    //m_settings->HALWrite(address_, req_byte_, 0, NULL, error_buffer_);
    
    /*
    bool result = m_settings->ifWrite(data, size);
    if (result < 0) {
        if (strlen(error_buffer_) > 0)
            HAL_ERROR2("%s write of data failed - %s\n", ifType, error_buffer_);
        return false;
    }
    else if (result != 1)
    {
        if (strlen(error_buffer_) > 0)
            HAL_ERROR2("%s write of data failed (nothing written) - %s\n", ifType, error_buffer_);
        return false;
    }
    */
   return true;
}

MotorDriverI2c::~MotorDriverI2c()
{
    m_settings->HALClose();
    
}