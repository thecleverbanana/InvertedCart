#include "mpu6050.hpp"

IMU::IMU(TwoWire &wire, float rollInitialize, float pitchInitialize, float yawInitialize)
: _wire(&wire), mpu(wire)
{
    cr = cos(rollInitialize);
    sr = sin(rollInitialize);
    cp = cos(pitchInitialize);
    sp = sin(pitchInitialize);
    cy = cos(yawInitialize);
    sy = sin(yawInitialize);

    rotationMatrixInitialize[0][0] = cy * cp;
    rotationMatrixInitialize[0][1] = cy * sp * sr - sy * cr;
    rotationMatrixInitialize[0][2] = cy * sp * cr + sy * sr;

    rotationMatrixInitialize[1][0] = sy * cp;
    rotationMatrixInitialize[1][1] = sy * sp * sr + cy * cr;
    rotationMatrixInitialize[1][2] = sy * sp * cr - cy * sr;

    rotationMatrixInitialize[2][0] = -sp;
    rotationMatrixInitialize[2][1] = cp * sr;
    rotationMatrixInitialize[2][2] = cp * cr;
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

    // --- Apply rotation matrix ---
    float sensorAngles[3] = {x_angle_radian, y_angle_radian, z_angle_radian};
    float sensorGyros[3]  = {x_gyro_radian, y_gyro_radian, z_gyro_radian};
    float sensorAccs[3]   = {x_acc, y_acc, z_acc};

    // Transform angles
    data.ActualXangle = 0;
    data.ActualYangle = 0;
    data.ActualZangle = 0;
    for (int i = 0; i < 3; ++i) {
        data.ActualXangle += rotationMatrixInitialize[0][i] * sensorAngles[i];
        data.ActualYangle += rotationMatrixInitialize[1][i] * sensorAngles[i];
        data.ActualZangle += rotationMatrixInitialize[2][i] * sensorAngles[i];
    }

    // Transform gyros
    data.ActualXgyro = 0;
    data.ActualYgyro = 0;
    data.ActualZgyro = 0;
    for (int i = 0; i < 3; ++i) {
        data.ActualXgyro += rotationMatrixInitialize[0][i] * sensorGyros[i];
        data.ActualYgyro += rotationMatrixInitialize[1][i] * sensorGyros[i];
        data.ActualZgyro += rotationMatrixInitialize[2][i] * sensorGyros[i];
    }

    // Transform accelerations
    data.ActualXacc = 0;
    data.ActualYacc = 0;
    data.ActualZacc = 0;
    for (int i = 0; i < 3; ++i) {
        data.ActualXacc += rotationMatrixInitialize[0][i] * sensorAccs[i];
        data.ActualYacc += rotationMatrixInitialize[1][i] * sensorAccs[i];
        data.ActualZacc += rotationMatrixInitialize[2][i] * sensorAccs[i];
    }

    return data;
}



void IMU::setLowPassFilterAlpha(float alpha) {
    LowPassfilterAlpha = constrain(alpha, 0.0f, 1.0f);
}


void IMU::serialPlotter(imu_data data) {
    Serial.print("ActualXangle:");
    Serial.print(data.ActualXangle);
    Serial.print(",");

    Serial.print("ActualYangle:");
    Serial.print(data.ActualYangle);
    Serial.print(",");

    Serial.print("ActualZangle:");
    Serial.print(data.ActualZangle);
    Serial.print(",");

    Serial.print("ActualXgyro:");
    Serial.print(data.ActualXgyro);
    Serial.print(",");

    Serial.print("ActualYgyro:");
    Serial.print(data.ActualYgyro);
    Serial.print(",");

    Serial.print("ActualZgyro:");
    Serial.print(data.ActualZgyro);
    Serial.print(",");

    Serial.print("ActualXacc:");
    Serial.print(data.ActualXacc);
    Serial.print(",");

    Serial.print("ActualYacc:");
    Serial.print(data.ActualYacc);
    Serial.print(",");

    Serial.print("ActualZacc:");
    Serial.print(data.ActualZacc);
    Serial.print(",");
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
    // Serial.print("ObjectXacc:");
    // Serial.print(mpu.getAccX());
    // Serial.print(",");
    return mpu.getAccX();  // X-axis acceleration
}

float IMU::getAccelY() {
    // Serial.print("ObjectYacc:");
    // Serial.print(mpu.getAccY());
    // Serial.print(",");
    return mpu.getAccY();  // Y-axis acceleration (cart direction)
}

float IMU::getAccelZ() {
    // Serial.print("ObjectZacc:");
    // Serial.print(mpu.getAccZ());
    // Serial.print(",");
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
    accelBiasZ = (accelSumZ / samples); //- 9.80665f;  // Subtract gravity for Z-axis

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

float IMU::applyLowPassFilter(float previous, float current, float alpha) {
    return alpha * previous + (1.0f - alpha) * current;
}

float IMU::LowPassFilterAccel(float rawValue, int axisIndex) {
    prevAccel[axisIndex] = applyLowPassFilter(prevAccel[axisIndex], rawValue, LowPassfilterAlpha);
    return prevAccel[axisIndex];
}

float IMU::LowPassFilterGyro(float rawValue, int axisIndex) {
    prevGyro[axisIndex] = applyLowPassFilter(prevGyro[axisIndex], rawValue, LowPassfilterAlpha);
    return prevGyro[axisIndex];
}

float IMU::LowPassFilterAngle(float rawValue, int axisIndex) {
    prevAngle[axisIndex] = applyLowPassFilter(prevAngle[axisIndex], rawValue, LowPassfilterAlpha);
    return prevAngle[axisIndex];
}    

float IMU::applyComplementaryFilter(float previousAngle, float gyroRate, float accelAngle, float dt, float alpha) {
    float gyroAngle = previousAngle + gyroRate * dt;
    return alpha * gyroAngle + (1.0f - alpha) * accelAngle;
}