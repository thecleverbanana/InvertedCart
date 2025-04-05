#include "mpu6050.hpp"

void IMU::begin() {
    Wire2.begin();
    mpu.begin();
    mpu.calcGyroOffsets(true); 
}

void IMU::update() {
    mpu.update();
}

float IMU::getPitchAngle() {
    return mpu.getAngleX();  
}

float IMU::getPitchRate() {
    return mpu.getGyroX(); 
}

float IMU::getCartAccel() {
    return mpu.getAccY();  
}

float IMU::getCartAccelCorrected(float theta, float dtheta, float ddtheta) {  
    float rawAccel = getCartAccel(); 

    float correction = -l * ddtheta * cos(theta) + l * dtheta * dtheta * sin(theta);
    return rawAccel - correction;
}


float IMU::getCartAccelCalibrated(float theta_rad, float dtheta_rad, float ddtheta_rad) {
    return getCartAccelCorrected(theta_rad, dtheta_rad, ddtheta_rad) - imuBiasY;
}


void IMU::calibrateAccel() {
    const int samples = 200;
    float readings[samples];

    Serial.println("\nCalibrating corrected X-axis acceleration... Please keep system still.");
    delay(100); 

    float prev_dtheta = 0.0f;
    const float dt = 0.005f;

    for (int i = 0; i < samples; ++i) {
        update();

        float theta_deg   = getPitchAngleZeroed();    
        float dtheta_deg  = getPitchRateZeroed();     
        float ddtheta_deg = (dtheta_deg - prev_dtheta) / dt;
        prev_dtheta = dtheta_deg;

        float theta_rad   = radians(theta_deg);
        float dtheta_rad  = radians(dtheta_deg);
        float ddtheta_rad = radians(ddtheta_deg);

        readings[i] = getCartAccelCorrected(theta_rad, dtheta_rad, ddtheta_rad);
        delay(5);
    }

    float sum = 0.0f;
    float maxVal = readings[0], minVal = readings[0];
    for (int i = 0; i < samples; ++i) {
        sum += readings[i];
        if (readings[i] > maxVal) maxVal = readings[i];
        if (readings[i] < minVal) minVal = readings[i];
    }

    sum -= maxVal + minVal;
    imuBiasY = sum / (samples - 2); 

    Serial.printf("Calibration complete. imuBiasY = %.5f m/s²\n", imuBiasY);
}


float IMU::getPitchAngleZeroed() {
    return getPitchAngle() - phiOffset;
}

void IMU::calibratePitchAngle() {
    update(); 
    phiOffset = getPitchAngle();
    Serial.printf("Pitch angle zero calibrated: %.2f° → now zeroed\n", phiOffset);
}

float IMU::getPitchRateZeroed() {
    return getPitchRate() - dphiOffset;
}

void IMU::calibratePitchRate() {
    const int samples = 100;
    float sum = 0.0f;

    Serial.println("Calibrating pitch angular rate bias...");
    for (int i = 0; i < samples; i++) {
        update();
        sum += getPitchRate();  // gyroX
        delay(5);
    }

    dphiOffset = sum / samples;
    Serial.printf("Pitch rate bias: %.4f deg/s\n", dphiOffset);
}
