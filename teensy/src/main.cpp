#include <Arduino.h>
#include "motor.hpp"
#include "control.hpp"
#include "mpu6050.hpp" 
#include "utils.hpp"

//frequency
float dt = 0.003f; // s
const unsigned long control_period_us = 3000;  // 3ms = 0.003s = 333Hz
unsigned long next_time = micros();
// Global Objects
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

Motor leftMotor(&can1, 1, 1);
Motor rightMotor(&can1, 2, -1);

Controller controller(&leftMotor, &rightMotor, dt); 

IMU imu_board(Wire2, PI, 0.0f, PI/2.0f, "Board_IMU", 0.02, 0.12, 0.2);
IMU imu_motor(Wire1, PI/2.0f, 0.0f, 0.0f, "Motor_IMU", 0.02, 0.015, 0.2);


//Global Variable
float leftInitialAngle = 0.0f;
float rightInitialAngle = 0.0f;
const float wheelRadius = 0.06f;
//IMU data
IMU_DATA imu_board_data, imu_motor_data;
float prev_Xacc = 0.0f;
float prev_Xvel = 0.0f;
float Xacc = 0.0f;

//For control
float x[4];   //Real Data
float x_hat[4];   //Predict Data

//Debug
Vectorx4 x_test = { 
  0.0f,   // x position (meters)
  0.0f,   // x velocity (m/s)
  0.1f,   // theta (radians)
  0.0f    // theta_dot (rad/s)
};

float u_test = 0;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial);

  // Initialize Can Bus communication
  can1.begin();
  can1.setBaudRate(1000000);

  // initialize the motor to get the intial angle
  leftMotor.clearMultiTurn();
  delay(100);
  leftMotor.softRestart();
  delay(1000);
  rightMotor.clearMultiTurn();
  delay(100);
  rightMotor.softRestart();
  delay(1000);

  leftInitialAngle = leftMotor.initialization();
  rightInitialAngle = rightMotor.initialization();

  // set the initial torque
  leftMotor.setTorque(0.0f);
  rightMotor.setTorque(0.0f);

  // Serial.println("Motors initialized with 0.0 N/m torque");

  //IMU initialization
  Wire1.setClock(1000000);  // 1 MHz
  Wire2.setClock(1000000);  // 1 MHz

  imu_board.begin();
  delay(100);
  imu_motor.begin();
  delay(100);
  imu_board.calibrateIMU();
  delay(200);
  imu_motor.calibrateIMU();
  delay(200);
  Serial.println("MPU6050 Initialized");

  //Initialize intial state x_hat[] value for controller
  imu_board.update();
  imu_motor.update();

  imu_board_data = imu_board.getIMUData();
  imu_motor_data = imu_motor.getIMUData();


  //Initialize x_hat for LQG
  // x_hat[0] = -radians(leftMotor.getCurrentDeltaAngle())*wheelRadius;
  // Xacc = imu_motor_data.Xacc;
  // x_hat[1] = computeVelocity(prev_Xacc,Xacc,dt,prev_Xvel);
  // prev_Xacc = Xacc;
  // prev_Xvel = x_hat[1];
  // x_hat[2] = imu_motor_data.Yangle;
  // x_hat[3] = imu_motor_data.Ygyro;

  //debug x_hat
  x_hat[0] = 0.0f;
  x_hat[1] = 0.0f;
  x_hat[2] = 0.1f;
  x_hat[3] = 0.1f;

  controller.initializeState(x_hat);
}

void loop() {
  if (micros() >= next_time) {
    next_time += control_period_us;

    unsigned long start_time = micros();

    // --- start ---
    imu_board.update();
    imu_motor.update();

    imu_board_data = imu_board.getIMUData();
    imu_motor_data = imu_motor.getIMUData();

    // debug
    x[0] = 0.0f;
    x[1] = 0.0f;
    x[2] = 0.0f;
    x[3] = 0.0f;

    float u = controller.updateEKF_LQG(x[0], x[1], x[2], x[3]);
    // leftMotor.setTorque(u / 2);
    // rightMotor.setTorque(u / 2);

    if (Serial.available()) {
      char cmd = Serial.read();
      if (cmd == 's') {
        leftMotor.stop();
        rightMotor.stop();
        while (1);
      }
    }

    // --- finish control ---

    unsigned long elapsed = micros() - start_time;

    // frequency print
    static unsigned long last_debug = 0;
    if (millis() - last_debug >= 500) {
      Serial.print("Loop time (us): ");
      Serial.print(elapsed);
      Serial.print(" | Frequency (Hz): ");
      Serial.println(1e6 / (float)elapsed);
      last_debug = millis();
    }
  }
}

