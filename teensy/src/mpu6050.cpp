#include "mpu6050.hpp"

IMU::IMU(TwoWire &wire,
    char X_mappedFrom, char Y_mappedFrom, char Z_mappedFrom,
    bool X_flipped, bool Y_flipped, bool Z_flipped)
: _wire(&wire), mpu(wire)
{
axisMap[0] = X_mappedFrom;
axisMap[1] = Y_mappedFrom;
axisMap[2] = Z_mappedFrom;

axisFlip[0] = X_flipped ? -1.0f : 1.0f;
axisFlip[1] = Y_flipped ? -1.0f : 1.0f;
axisFlip[2] = Z_flipped ? -1.0f : 1.0f;
}


void IMU::begin() {
    _wire->begin();
    mpu.begin();
    mpu.calcGyroOffsets(true);
}

void IMU::update() {
    mpu.update();
}

imu_data IMU::getIMUData() {
    imu_data data;

    // --- Bias-compensated values ---
    float x_angle_raw = getAngleXDegBiasCompensated();
    float y_angle_raw = getAngleYDegBiasCompensated();
    float z_angle_raw = getAngleZDegBiasCompensated(); 

    float x_gyro_raw = getGyroRateXDegPerSecBiasCompensated();
    float y_gyro_raw = getGyroRateYDegPerSecBiasCompensated(); 
    float z_gyro_raw = getGyroRateZDegPerSecBiasCompensated();

    float x_acc_raw = getAccelXBiasCompensated();
    float y_acc_raw = getAccelYBiasCompensated();
    float z_acc_raw = getAccelZBiasCompensated();

    // --- Apply filtering ---
    float x_angle = LowPassFilterAngle(x_angle_raw, 0);
    float y_angle = LowPassFilterAngle(y_angle_raw, 1);
    float z_angle = LowPassFilterAngle(z_angle_raw, 2);
    
    float x_gyro = LowPassFilterGyro(x_gyro_raw, 0);
    float y_gyro = LowPassFilterGyro(y_gyro_raw, 1);
    float z_gyro = LowPassFilterGyro(z_gyro_raw, 2);

    float x_acc = LowPassFilterAccel(x_acc_raw, 0);
    float y_acc = LowPassFilterAccel(y_acc_raw, 1);
    float z_acc = LowPassFilterAccel(z_acc_raw, 2);

    // --- Change to Radian ---
    float x_angle_radian = radians(x_angle);
    float y_angle_radian = radians(y_angle);
    float z_angle_radian = radians(z_angle);

    float x_gyro_radian = radians(x_gyro);
    float y_gyro_radian = radians(y_gyro);
    float z_gyro_radian = radians(z_gyro);
    

    // --- Apply mapping and flipping ---
    data.ActualXangle = applyMapping(axisMap[0], x_angle_radian, y_angle_radian, z_angle_radian, axisFlip[0]);
    data.ActualYangle = applyMapping(axisMap[1], x_angle_radian, y_angle_radian, z_angle_radian, axisFlip[1]);
    data.ActualZangle = applyMapping(axisMap[2], x_angle_radian, y_angle_radian, z_angle_radian, axisFlip[2]);

    data.ActualXgyro = applyMapping(axisMap[0], x_gyro_radian, y_gyro_radian, z_gyro_radian, axisFlip[0]);
    data.ActualYgyro = applyMapping(axisMap[1], x_gyro_radian, y_gyro_radian, z_gyro_radian, axisFlip[1]);
    data.ActualZgyro = applyMapping(axisMap[2], x_gyro_radian, y_gyro_radian, z_gyro_radian, axisFlip[2]);

    data.ActualXacc = applyMapping(axisMap[0], x_acc, y_acc, z_acc, axisFlip[0]);
    data.ActualYacc = applyMapping(axisMap[1], x_acc, y_acc, z_acc, axisFlip[1]);
    data.ActualZacc = applyMapping(axisMap[2], x_acc, y_acc, z_acc, axisFlip[2]);

    return data;
}


void IMU::setFilterAlpha(float alpha) {
    filterAlpha = constrain(alpha, 0.0f, 1.0f);
}

// Angle (degrees):
float IMU::getAngleXDeg() {
    return mpu.getAngleX();  // X-axis angle (pitch)
}

float IMU::getAngleYDeg() {
    return mpu.getAngleY();  // Y-axis angle (roll)
}

float IMU::getAngleZDeg() {
    return mpu.getAngleZ();  // Z-axis angle (yaw)
}

// Gyro rate (degrees per second):

float IMU::getGyroRateXDegPerSec() {
    return mpu.getGyroX();  // X-axis gyro rate
}

float IMU::getGyroRateYDegPerSec() {
    return mpu.getGyroY();  // Y-axis gyro rate
}

float IMU::getGyroRateZDegPerSec() {
    return mpu.getGyroZ();  // Z-axis gyro rate
}

// Acceleration (m/s²):

float IMU::getAccelX() {
    return mpu.getAccX();  // X-axis acceleration
}

float IMU::getAccelY() {
    return mpu.getAccY();  // Y-axis acceleration (cart direction)
}

float IMU::getAccelZ() {
    return mpu.getAccZ();  // Z-axis acceleration
}


void IMU::calibrateIMU() {
    const int samples = 200;
    
    float accelSumX = 0.0f, accelSumY = 0.0f, accelSumZ = 0.0f;
    float gyroSumX = 0.0f, gyroSumY = 0.0f, gyroSumZ = 0.0f;

    Serial.println("\nCalibrating IMU... Please keep system still.");
    delay(300);

    for (int i = 0; i < samples; ++i) {
        update();

        accelSumX += getAccelX();
        accelSumY += getAccelY();
        accelSumZ += getAccelZ();

        gyroSumX += getGyroRateXDegPerSec();
        gyroSumY += getGyroRateYDegPerSec();
        gyroSumZ += getGyroRateZDegPerSec();

        delay(5);
    }

    // Compute averages
    accelBiasX = accelSumX / samples;
    accelBiasY = accelSumY / samples;
    accelBiasZ = (accelSumZ / samples) - 9.80665f;  // Subtract gravity for Z-axis

    gyroBiasX = gyroSumX / samples;
    gyroBiasY = gyroSumY / samples;
    gyroBiasZ = gyroSumZ / samples;

    phiOffset = getAngleXDeg();
    rollOffset = getAngleYDeg();
    yawOffset = getAngleZDeg();

    // Report results
    Serial.println("Calibration complete:");
    Serial.printf("Accel bias: X: %.5f m/s², Y: %.5f m/s², Z: %.5f m/s²\n", 
                accelBiasX, accelBiasY, accelBiasZ);
    Serial.printf("Gyro bias: X: %.5f deg/s, Y: %.5f deg/s, Z: %.5f deg/s\n", 
                gyroBiasX, gyroBiasY, gyroBiasZ);
    Serial.printf("Initial angle offsets: Pitch (X): %.2f°, Roll (Y): %.2f°, Yaw (Z): %.2f°\n",
                phiOffset, rollOffset, yawOffset);
}

// --- Angle (degrees), bias-compensated ---
float IMU::getAngleXDegBiasCompensated() {
    return mpu.getAngleX() - phiOffset;  // Pitch
}

float IMU::getAngleYDegBiasCompensated() {
    return mpu.getAngleY() - rollOffset;  // Roll
}

float IMU::getAngleZDegBiasCompensated() {
    return mpu.getAngleZ() - yawOffset;  // Yaw
}

// --- Gyro rate (degrees per second), bias-compensated ---
float IMU::getGyroRateXDegPerSecBiasCompensated() {
    return mpu.getGyroX() - gyroBiasX;  // Pitch rate
}

float IMU::getGyroRateYDegPerSecBiasCompensated() {
    return mpu.getGyroY() - gyroBiasY;  // Roll rate
}

float IMU::getGyroRateZDegPerSecBiasCompensated() {
    return mpu.getGyroZ() - gyroBiasZ;  // Yaw rate
}

// --- Acceleration (m/s²), bias-compensated ---
float IMU::getAccelXBiasCompensated() {
    return mpu.getAccX() - accelBiasX;
}

float IMU::getAccelYBiasCompensated() {
    return mpu.getAccY() - accelBiasY;
}

float IMU::getAccelZBiasCompensated() {
    return mpu.getAccZ() - accelBiasZ;
}

float IMU::applyMapping(char target, float x, float y, float z, float flip) {
    float value = 0.0f;
    switch (target) {
        case 'X': value = x; break;
        case 'Y': value = y; break;
        case 'Z': value = z; break;
        default: value = 0.0f; break;
    }
    return flip * value;
}

float IMU::applyLowPassFilter(float previous, float current, float alpha) {
    return alpha * previous + (1.0f - alpha) * current;
}

float IMU::LowPassFilterAccel(float rawValue, int axisIndex) {
    prevAccel[axisIndex] = applyLowPassFilter(prevAccel[axisIndex], rawValue, filterAlpha);
    return prevAccel[axisIndex];
}

float IMU::LowPassFilterGyro(float rawValue, int axisIndex) {
    prevGyro[axisIndex] = applyLowPassFilter(prevGyro[axisIndex], rawValue, filterAlpha);
    return prevGyro[axisIndex];
}

float IMU::LowPassFilterAngle(float rawValue, int axisIndex) {
    prevAngle[axisIndex] = applyLowPassFilter(prevAngle[axisIndex], rawValue, filterAlpha);
    return prevAngle[axisIndex];
}    