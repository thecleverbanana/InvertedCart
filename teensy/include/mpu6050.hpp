#ifndef MPU6050_WRAPPER_HPP
#define MPU6050_WRAPPER_HPP

#include <Wire.h>
#include <MPU6050_tockn.h>

struct IMU_DATA{
    float Xangle; // X-axis angle
    float Yangle; // Y-axis angle
    float Zangle; // Z-axis angle
    

    float Xgyro; // X-axis gyro rate
    float Ygyro; // Y-axis gyro rate
    float Zgyro; // Z-axis gyro rate

    float Xacc; // X-axis acc rate
    float Yacc; // Y-axis acc rate
    float Zacc; // Z-axis acc rate
};

class IMU {
public:
    IMU(TwoWire &wire, float rollInitialize, float pitchInitialize, float yawInitialize,
        const char* name, float rx, float ry, float rz);
    void begin();
    void update();

    void calibrateIMU();
    void getIMUData();

    void setLowPassFilterAlpha(float alpha);
    void serialPlotter(IMU_DATA data, const char* objectName);

private:
    TwoWire* _wire;
    MPU6050 mpu;
    const char* objectName;

    // --- Rotate IMU to real world axis ----
    float rotationMatrixInitialize[3][3];

    // --- Get Raw data and Change to World Axis (degrees) ---
    IMU_DATA rawData;
    void getRawData();

    //Bias Data
    IMU_DATA bias;

    // --- Get Compenstaed data (degrees) ---
    IMU_DATA compensatedData;
    void getCompensatedData();

    IMU_DATA filteredData;
    void getFilteredData();

    IMU_DATA radianData;
    void getRadianData();

    // Filters
        //Low Pass filter
        float applyLowPassFilter(float previous, float current, float alpha);

        float LowPassFilterAccel(float rawValue, int axisIndex);
        float LowPassFilterGyro(float rawValue, int axisIndex);
        float LowPassFilterAngle(float rawValue, int axisIndex);

        float prevAccel[3] = {0.0f, 0.0f, 0.0f};
        float prevGyro[3] = {0.0f, 0.0f, 0.0f};
        float prevAngle[3] = {0.0f, 0.0f, 0.0f};

        float LowPassfilterAlpha = 0.9f; 

        //Complimentary filter
        float estimatedAngle[3] = {0.0f, 0.0f, 0.0f};
        
        float applyComplementaryFilter(float previousAngle, float gyroRate, float accelAngle, float dt, float alpha);

        float ComplimentaryfilterAlpha = 0.9f;

    // ---- Rotatory center for each axis ---
    IMU_DATA linearAccCorrectionData;
    float r_x = 0.0f;
    float r_y = 0.0f;
    float r_z = 0.0f;

    float r[3] = {r_x, r_y, r_z};
    

    void getActualAcceleration();
 
};

#endif
