#include <Arduino.h>
#include "motor.hpp"
#include "control.hpp"
#include "mpu6050.hpp" 
#include "utils.hpp"

// Global Objects
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
Motor leftMotor(&can1, 1, 1);
Motor rightMotor(&can1, 2, -1);
Controller controller(&leftMotor, &rightMotor, 0.005); 

float leftInitialAngle = 0.0f;
float rightInitialAngle = 0.0f;
const float wheelRadius = 0.06f;

// IMU imu_board(
//   Wire2,
//   'Y', 'X', 'Z',    // axis mapping
//   true, true, true // flip
// );
IMU imu_board(Wire2, PI, 0.0f, PI / 2.0f);
// IMU imu_motor(Wire1, PI / 2.0f, 0.0f, 0.0f);

//IMU data
imu_data imu_board_data, imu_motor_data;

//For control
float x[4];   //Real Data
float x_hat[4];   //Predict Data

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial);

  // Initialize Can Bus communication
  can1.begin();
  can1.setBaudRate(1000000);

  // initialize the motor to get the intial angle
  leftInitialAngle = leftMotor.initialization();
  rightInitialAngle = rightMotor.initialization();

  // //clear multiturn and enable the save
  leftMotor.clearMultiTurn();
  delay(100);
  leftMotor.enableMultiTurnSave();
  delay(100);
  leftMotor.softRestart();
  delay(100);
  leftMotor.verifyClearedZero();

  // set the initial torque
  leftMotor.setTorque(0.0f);
  rightMotor.setTorque(0.0f);

  Serial.println("Motors initialized with 0.0 N/m torque");

  //IMU initialization
  Wire1.setClock(1000000);  // 1 MHz
  Wire2.setClock(1000000);  // 1 MHz

  imu_board.begin();
  delay(100);
  // imu_motor.begin();
  delay(100);
  imu_board.calibrateIMU();
  delay(200);
  // imu_motor.calibrateIMU();
  delay(200);
  Serial.println("MPU6050 Initialized");

  //Initialize intial state x_hat[] value for controller
  imu_board.update();
  // imu_motor.update();

  imu_board_data = imu_board.getIMUData();
  // imu_motor_data = imu_motor.getIMUData();

  MotorStatus status;
  if (leftMotor.getStatus(status)) {
      float deltaAngle = status.angle_deg - leftInitialAngle;  // deg
      x_hat[0] = wheelRadius * radians(deltaAngle);       //add negative to align with the direction     
  }

  x_hat[1] = imu_motor_data.ActualXacc;
  // x_hat[1] = imu_board_data.ActualXacc;
  x_hat[2] = imu_board_data.ActualYangle;
  x_hat[3] = imu_board_data.ActualYgyro;
  
  controller.initializeState(x_hat);
}

void stopMotors() {
  leftMotor.setTorque(0.0f);
  rightMotor.setTorque(0.0f);
  Serial.println("Motors stopped.");
}

void loop() {
  // Get measured values for estimation
  imu_board.update();
  // imu_motor.update();

  imu_board_data = imu_board.getIMUData();
  // imu_motor_data = imu_motor.getIMUData();

  Serial.print(">");
  imu_board.serialPlotter(imu_board_data);
  Serial.println(); // Writes \r\n

  MotorStatus status;
  if (leftMotor.getStatus(status)) {
      float deltaAngle = status.angle_deg - leftInitialAngle;  // deg
      x[0] = -wheelRadius * radians(deltaAngle);       //add negative to align with the direction     
  }

  x[1] = imu_motor_data.ActualXacc;
  // x[1] = imu_board_data.ActualXacc;
  x[2] = imu_board_data.ActualYangle;
  x[3] = imu_board_data.ActualYgyro;


  // Serial.printf("x = %7.4f m,   dx = %6.3f m/s²,   theta = %6.2frad,   dtheta = %6.2frad/s\n", x[0], x[1], x[2], x[3]);
  
  //Apply Control
  // float u = controller.updateLQG(x[0], x[1], x[2], x[3]);

  // leftMotor.setTorque(-u);
  // rightMotor.setTorque(-u); 

  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 's') { 
      stopMotors();
      while (1);
    }
  }

  //dynamically update control frequency
  // unsigned long now = millis();
  // static unsigned long last_time = 0;
  // float dt_dynamic = (now - last_time) / 1000.0f;
  // last_time = now;
  // controller.setDt(dt_dynamic);
  delay(5);
}
