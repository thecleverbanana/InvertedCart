#ifndef MPU6050_WRAPPER_HPP
#define MPU6050_WRAPPER_HPP

#include <Wire.h>
#include <MPU6050_tockn.h>

struct imu_data{
    float ActualXangle; // Real system X-axis angle
    float ActualYangle; // Real system Y-axis angle
    float ActualZangle; // Real system Z-axis angle
    

    float ActualXgyro; // Real system X-axis gyro rate
    float ActualYgyro; // Real system Y-axis gyro rate
    float ActualZgyro; // Real system Z-axis gyro rate

    float ActualXacc; // Real system X-axis acc rate
    float ActualYacc; // Real system Y-axis acc rate
    float ActualZacc; // Real system z-axis acc rate
};

class IMU {
public:
    IMU(TwoWire &wire, char X_mappedFrom, char Y_mappedFrom, char Z_mappedFrom,
    bool X_flipped, bool Y_flipped,bool Z_flipped);
    void begin();
    void update();
    void calibrateIMU();
    imu_data getIMUData();
    void setFilterAlpha(float alpha);

private:
    TwoWire* _wire;
    MPU6050 mpu;

    // Axis mapping: user defines in constructor what axis X, Y, Z map to
    char axisMap[3];  // 0: X, 1: Y, 2: Z -> stores 'X', 'Y', 'Z'

    // Axis flipping: +1 or -1 to apply sign flip
    float axisFlip[3];

    // --- Accel biases ---
    float accelBiasX = 0.0f;
    float accelBiasY = 0.0f;
    float accelBiasZ = 0.0f;

    // --- Gyro biases ---
    float gyroBiasX = 0.0f;
    float gyroBiasY = 0.0f;
    float gyroBiasZ = 0.0f;

    // --- Angle zero offsets ---
    float phiOffset = 0.0f;  // Pitch zero offset
    float rollOffset = 0.0f; // Roll zero offset
    float yawOffset = 0.0f;  // Yaw zero offset

    // --- Pendulum parameter ---
    float l = 0.133f;


    // --- Angle (degrees) ---
    float getAngleXDeg();  // Pitch angle 
    float getAngleYDeg();  // Roll angle   
    float getAngleZDeg();  // Yaw angle

    // --- Gyroscope rate (deg/s) ---
    float getGyroRateXDegPerSec();  // Pitch rate
    float getGyroRateYDegPerSec();  // Roll rate
    float getGyroRateZDegPerSec();  // Yaw rate

    // --- Acceleration (m/s^2) ---
    float getAccelX();
    float getAccelY();  // Cart direction
    float getAccelZ();


    // --- Angle (degrees), bias-compensated ---
    float getAngleXDegBiasCompensated();  // Pitch angle
    float getAngleYDegBiasCompensated();  // Roll angle
    float getAngleZDegBiasCompensated();  // Yaw angle

    // --- Gyroscope rate (deg/s), bias-compensated ---
    float getGyroRateXDegPerSecBiasCompensated();  // Pitch rate
    float getGyroRateYDegPerSecBiasCompensated();  // Roll rate
    float getGyroRateZDegPerSecBiasCompensated();  // Yaw rate

    // --- Acceleration (m/s^2), bias-compensated ---
    float getAccelXBiasCompensated();
    float getAccelYBiasCompensated();  // Cart direction
    float getAccelZBiasCompensated();

    // Mapping helper function
    float applyMapping(char target, float x, float y, float z, float flip);

    // Filters
    float applyLowPassFilter(float previous, float current, float alpha);

    float LowPassFilterAccel(float rawValue, int axisIndex);
    float LowPassFilterGyro(float rawValue, int axisIndex);
    float LowPassFilterAngle(float rawValue, int axisIndex);

    float prevAccel[3] = {0.0f, 0.0f, 0.0f};
    float prevGyro[3] = {0.0f, 0.0f, 0.0f};
    float prevAngle[3] = {0.0f, 0.0f, 0.0f};

    float filterAlpha = 0.9f; 
};

#endif
