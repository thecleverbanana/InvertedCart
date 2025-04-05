#include <Arduino.h>
#include "motor.hpp"
#include "control.hpp"
#include "mpu6050.hpp" 

// Global Objects
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
Motor leftMotor(&can1, 1, 1);
Motor rightMotor(&can1, 2, -1);
Controller controller(&leftMotor, &rightMotor, 0.005); 

float leftInitialAngle = 0.0f;
float rightInitialAngle = 0.0f;
const float wheelRadius = 0.06f;
float dx_filtered = 0.0f;
float dtheta_filtered = 0.0f;
static float prev_dtheta = 0.0f;


IMU imu; // MPU6050

void setup() {
  Serial.begin(115200);
  while (!Serial);

  can1.begin();
  can1.setBaudRate(1000000);

  // initialize the motor to get the intial angle
  leftInitialAngle = leftMotor.initialization();
  rightInitialAngle = rightMotor.initialization();

  // //clear multiturn and enable the save
  // leftMotor.clearMultiTurn();
  // delay(100);
  // leftMotor.enableMultiTurnSave();
  // delay(100);
  // leftMotor.softRestart();
  // delay(1000);
  // leftMotor.verifyClearedZero();

  // set the initial torque
  leftMotor.setTorque(0.0f);
  rightMotor.setTorque(0.0f);

  Serial.println("Motors initialized with 0.0 N/m torque");

  //IMU initialization
  imu.begin();
  delay(100);
  imu.calibrateAccel();
  imu.calibratePitchAngle();
  imu.calibratePitchRate();
  Serial.println("MPU6050 Initialized");

  //Initialize intial state value for controller
}

void stopMotors() {
  leftMotor.setTorque(0.0f);
  rightMotor.setTorque(0.0f);
  Serial.println("Motors stopped.");
}

void loop() {
  // Get C values for estimation
  imu.update();

  float x = 0.0f;
  float theta  = imu.getPitchAngleZeroed();      
  float dtheta = imu.getPitchRateZeroed(); 

  float ddtheta = (dtheta - prev_dtheta) / 0.1;
  prev_dtheta = dtheta;

  float theta_rad = radians(theta);
  float dtheta_rad = radians(dtheta);
  float ddtheta_rad = radians(ddtheta);

  float dx = imu.getCartAccelCalibrated(theta_rad, dtheta_rad, ddtheta_rad);

  MotorStatus status;
  if (leftMotor.getStatus(status)) {
      float deltaAngle = status.angle_deg - leftInitialAngle;  // deg
      x = -wheelRadius * radians(deltaAngle);       //add negative to align with the direction     
  }

  float dx_filtered = 0.9 * dx_filtered + (1 - 0.9) * dx;
  float dtheta_filtered = 0.9 * dtheta_filtered + 0.1 * dtheta;
  float dtheta_filtered_rad = radians(dtheta_filtered);
 
  //Serial.printf("x = %7.4f m,   dx = %6.3f m/s²,   theta = %6.2f°,   dtheta = %6.2f°/s\n", x, dx_filtered, theta, dtheta_filtered);
  
  //Apply Control
  float u = controller.updateLQR(x, dx_filtered, 0.0, dtheta_filtered);
  Serial.printf("u derived by LQR = %7.4f tau", u);
  

  // Serial.printf("x = %7.4f m,   dx = %6.3f m/s²,  dtheta = %6.2f\n", x, dx_filtered, dtheta_filtered_rad);
  // float u = controller.updateLQG(x, dx_filtered, 0.00, dtheta_filtered_rad);

  // leftMotor.setTorque(u);
  // rightMotor.setTorque(u); 

  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 's') { 
      stopMotors();
      while (1);
    }
  }

  //dynamically update control frequency
  delay(5);
  // unsigned long now = millis();
  // static unsigned long last_time = 0;
  // float dt_dynamic = (now - last_time) / 1000.0f;
  // last_time = now;
  // controller.setDt(dt_dynamic);

  controller.setDt(0.005f);
}
