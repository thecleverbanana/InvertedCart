#include "mpu6050.hpp"

IMU::IMU(TwoWire &wire, float rollInitialize, float pitchInitialize, float yawInitialize, 
    const char* name, float rx, float ry, float rz)
: _wire(&wire), mpu(wire), objectName(name), r_x(rx), r_y(ry), r_z(rz)
{
    r[0] = r_x;
    r[1] = r_y;
    r[2] = r_z;
    
    float cr = cos(rollInitialize);
    float sr = sin(rollInitialize);
    float cp = cos(pitchInitialize);
    float sp = sin(pitchInitialize);
    float cy = cos(yawInitialize);
    float sy = sin(yawInitialize);

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

void IMU::calibrateIMU() {
    const int samples = 200;

    float accelSum[3] = {0.0f, 0.0f, 0.0f};
    float gyroSum[3]  = {0.0f, 0.0f, 0.0f};
    float angleSum[3] = {0.0f, 0.0f, 0.0f};

    Serial.println("\nCalibrating IMU... Please keep system still.");
    delay(300);

    for (int i = 0; i < samples; ++i) {
        update();

        getRawData();

        accelSum[0] += rawData.Xacc;
        accelSum[1] += rawData.Yacc;
        accelSum[2] += rawData.Zacc;

        gyroSum[0] += rawData.Xgyro;
        gyroSum[1] += rawData.Ygyro;
        gyroSum[2] += rawData.Zgyro;

        angleSum[0] += rawData.Xangle;
        angleSum[1] += rawData.Yangle;
        angleSum[2] += rawData.Zangle;

        delay(5);
    }

    // --- store in bias struct ---
    bias.Xacc = accelSum[0] / samples;
    bias.Yacc = accelSum[1] / samples;
    bias.Zacc = accelSum[2] / samples - 9.80665f; //subtract gravity

    bias.Xgyro = gyroSum[0] / samples;
    bias.Ygyro = gyroSum[1] / samples;
    bias.Zgyro = gyroSum[2] / samples;

    bias.Xangle = angleSum[0] / samples;
    bias.Yangle = angleSum[1] / samples;
    bias.Zangle = angleSum[2] / samples;

    // --- serial output ---
    Serial.println("Calibration complete:");
    Serial.printf("Accel bias: X: %.5f m/s², Y: %.5f m/s², Z: %.5f m/s²\n", 
                  bias.Xacc, bias.Yacc, bias.Zacc);
    Serial.printf("Gyro bias: X: %.5f deg/s, Y: %.5f deg/s, Z: %.5f deg/s\n", 
                  bias.Xgyro, bias.Ygyro, bias.Zgyro);
    Serial.printf("Initial angle offsets: X: %.2f°, Y: %.2f°, Z: %.2f°\n",
                  bias.Xangle, bias.Yangle, bias.Zangle);
}

IMU_DATA IMU::getIMUData() {
    getRawData();           // Step 1: Read sensor and apply rotation matrix
    getCompensatedData();   // Step 2: Remove bias
    getFilteredData();      // Step 3: Apply filtering
    getRadianData();        // Step 4: Convert to radians
    getActualAcceleration(); //step 5: Get linear correction data

    return linearAccCorrectionData;
}

void IMU::setLowPassFilterAlpha(float alpha) {
    LowPassfilterAlpha = constrain(alpha, 0.0f, 1.0f);
}

void IMU::serialPlotter(IMU_DATA data) {
    Serial.print(">");

    Serial.print(objectName); Serial.print("Xangle:");
    Serial.print(data.Xangle); Serial.print(",");

    Serial.print(objectName); Serial.print("Yangle:");
    Serial.print(data.Yangle); Serial.print(",");

    Serial.print(objectName); Serial.print("Zangle:");
    Serial.print(data.Zangle); Serial.print(",");

    Serial.print(objectName); Serial.print("Xgyro:");
    Serial.print(data.Xgyro); Serial.print(",");

    Serial.print(objectName); Serial.print("Ygyro:");
    Serial.print(data.Ygyro); Serial.print(",");

    Serial.print(objectName); Serial.print("Zgyro:");
    Serial.print(data.Zgyro); Serial.print(",");

    Serial.print(objectName); Serial.print("Xacc:");
    Serial.print(data.Xacc); Serial.print(",");

    Serial.print(objectName); Serial.print("Yacc:");
    Serial.print(data.Yacc); Serial.print(",");

    Serial.print(objectName); Serial.print("Zacc:");
    Serial.print(data.Zacc);

    Serial.println();
}

void IMU::getRawData() {
    // 1. Raw data
    float sensorAngles[3] = { mpu.getAngleX(), mpu.getAngleY(), mpu.getAngleZ() };
    float sensorGyros[3]  = { mpu.getGyroX(), mpu.getGyroY(), mpu.getGyroZ() };
    float sensorAccs[3]   = { mpu.getAccX(), mpu.getAccY(), mpu.getAccZ() };

    // 2. Change Axis to World Axis
    float worldAngles[3] = {0.0f, 0.0f, 0.0f};
    float worldGyros[3]  = {0.0f, 0.0f, 0.0f};
    float worldAccs[3]   = {0.0f, 0.0f, 0.0f};

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            worldAngles[i] += rotationMatrixInitialize[i][j] * sensorAngles[j];
            worldGyros[i]  += rotationMatrixInitialize[i][j] * sensorGyros[j];
            worldAccs[i]   += rotationMatrixInitialize[i][j] * sensorAccs[j];
        }
    }

    // 3. write to struct
    rawData.Xangle = worldAngles[0];
    rawData.Yangle = worldAngles[1];
    rawData.Zangle = worldAngles[2];

    rawData.Xgyro = worldGyros[0];
    rawData.Ygyro = worldGyros[1];
    rawData.Zgyro = worldGyros[2];

    rawData.Xacc = worldAccs[0];
    rawData.Yacc = worldAccs[1];
    rawData.Zacc = worldAccs[2];
}

void IMU::getCompensatedData() {
    // Acceleration compensated
    compensatedData.Xacc = rawData.Xacc - bias.Xacc;
    compensatedData.Yacc = rawData.Yacc - bias.Yacc;
    compensatedData.Zacc = rawData.Zacc - bias.Zacc;

    // Gyroscope compensated
    compensatedData.Xgyro = rawData.Xgyro - bias.Xgyro;
    compensatedData.Ygyro = rawData.Ygyro - bias.Ygyro;
    compensatedData.Zgyro = rawData.Zgyro - bias.Zgyro;

    // Angle compensated
    compensatedData.Xangle = rawData.Xangle - bias.Xangle;
    compensatedData.Yangle = rawData.Yangle - bias.Yangle;
    compensatedData.Zangle = rawData.Zangle - bias.Zangle;
}

void IMU::getFilteredData() {
    // --- Apply Low Pass Filter to Acceleration ---
    filteredData.Xacc = LowPassFilterAccel(compensatedData.Xacc, 0);
    filteredData.Yacc = LowPassFilterAccel(compensatedData.Yacc, 1);
    filteredData.Zacc = LowPassFilterAccel(compensatedData.Zacc, 2);

    // --- Apply Low Pass Filter to Gyroscope ---
    filteredData.Xgyro = LowPassFilterGyro(compensatedData.Xgyro, 0);
    filteredData.Ygyro = LowPassFilterGyro(compensatedData.Ygyro, 1);
    filteredData.Zgyro = LowPassFilterGyro(compensatedData.Zgyro, 2);

    // --- Apply Low Pass Filter to Angles ---
    filteredData.Xangle = LowPassFilterAngle(compensatedData.Xangle, 0);
    filteredData.Yangle = LowPassFilterAngle(compensatedData.Yangle, 1);
    filteredData.Zangle = LowPassFilterAngle(compensatedData.Zangle, 2);
}

void IMU::getRadianData() {
     // --- Acceleration ---
     radianData.Xacc = filteredData.Xacc;
     radianData.Yacc = filteredData.Yacc;
     radianData.Zacc = filteredData.Zacc;
 
     // --- Gyroscope: deg/s -> rad/s ---
     radianData.Xgyro = radians(filteredData.Xgyro);
     radianData.Ygyro = radians(filteredData.Ygyro);
     radianData.Zgyro = radians(filteredData.Zgyro);
 
     // --- Angle: deg -> rad ---
     radianData.Xangle = radians(filteredData.Xangle);
     radianData.Yangle = radians(filteredData.Yangle);
     radianData.Zangle = radians(filteredData.Zangle);
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


void IMU::getActualAcceleration() {    
    // Get angular velocity (rad/s)
    float omega[3] = { radianData.Xgyro, radianData.Ygyro, radianData.Zgyro };

    // Step 1: omega × r
    float omegaCrossR[3] = {
        omega[1] * r[2] - omega[2] * r[1],
        omega[2] * r[0] - omega[0] * r[2],
        omega[0] * r[1] - omega[1] * r[0]
    };

    // Step 2: omega × (omega × r)
    float centrifugal[3] = {
        omega[1] * omegaCrossR[2] - omega[2] * omegaCrossR[1],
        omega[2] * omegaCrossR[0] - omega[0] * omegaCrossR[2],
        omega[0] * omegaCrossR[1] - omega[1] * omegaCrossR[0]
    };

    // Step 3: Subtract centrifugal from measured acceleration (radianData)
    linearAccCorrectionData.Xacc = radianData.Xacc - centrifugal[0];
    linearAccCorrectionData.Yacc = radianData.Yacc - centrifugal[1];
    linearAccCorrectionData.Zacc = radianData.Zacc - centrifugal[2];

    // Optional: Copy other useful data if you like
    linearAccCorrectionData.Xgyro = radianData.Xgyro;
    linearAccCorrectionData.Ygyro = radianData.Ygyro;
    linearAccCorrectionData.Zgyro = radianData.Zgyro;

    linearAccCorrectionData.Xangle = radianData.Xangle;
    linearAccCorrectionData.Yangle = radianData.Yangle;
    linearAccCorrectionData.Zangle = radianData.Zangle;
}

