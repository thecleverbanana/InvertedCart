#ifndef MPU6050_WRAPPER_HPP
#define MPU6050_WRAPPER_HPP

#include <Wire.h>
#include <MPU6050_tockn.h>

class IMU {
public:
    void begin();
    void update();

    float getPitchAngle();     // phi（deg）  
    float getPitchRate();      // phi'（deg/s） 
    float getCartAccel();      // ẍ（m/s²）

    float getCartAccelCorrected(float theta, float dtheta, float ddtheta);
    float getCartAccelCalibrated(float theta_rad, float dtheta_rad, float ddtheta_rad); 
    void calibrateAccel();

    float getPitchAngleZeroed(); 
    void calibratePitchAngle();
    
    float getPitchRateZeroed();   
    void calibratePitchRate();  


private:
    MPU6050 mpu = MPU6050(Wire2);

    float imuBiasY = 0.0f; 
    float phiOffset = 0.0f;
    float dphiOffset = 0.0f;
    float l = 0.133f;
};

#endif
