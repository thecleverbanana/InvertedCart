#include <Arduino.h>
#include "motor.hpp"
#include "control.hpp"
#include "mpu6050.hpp" 
#include "utils.hpp"

//frequency
float dt = 0.001f; // s
// Global Objects
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
Motor leftMotor(&can1, 1, 1);
Motor rightMotor(&can1, 2, -1);
Controller controller(&leftMotor, &rightMotor, dt); 
IMU imu_board(Wire2, PI, 0.0f, PI/2.0f, "Board_IMU", 0.02, 0.12, 0.2);
IMU imu_motor(Wire1, PI/2.0f, 0.0f, 0.0f, "Motor_IMU", 0.02, 0.015, 0.2);
// IMU imu_motor(Wire1, PI / 2.0f, 0.0f, 0.0f);


//Global Variable
float leftInitialAngle = 0.0f;
float rightInitialAngle = 0.0f;
const float wheelRadius = 0.06f;
//IMU data
IMU_DATA imu_board_data, imu_motor_data;

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

  Serial.println("Motors initialized with 0.0 N/m torque");

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
  x_hat[0] = -radians(leftMotor.getCurrentDeltaAngle())*wheelRadius;
  x_hat[1] = imu_motor_data.Xacc;
  x_hat[2] = imu_motor_data.Yangle;
  x_hat[3] = imu_board_data.Ygyro;
  
  controller.initializeState(x_hat);
}

void loop() {
  // Get measured values for estimation
  imu_board.update();
  imu_motor.update();

  imu_board_data = imu_board.getIMUData();
  imu_motor_data = imu_motor.getIMUData();

  // imu_board.serialPlotter(imu_board_data);
  // imu_motor.serialPlotter(imu_motor_data);

  x[0] = -radians(leftMotor.getCurrentDeltaAngle())*wheelRadius;
  x[0] = fusedPositionEstimate(x[0], x[1], dt);
  x[1] = imu_motor_data.Xacc;
  x[2] = imu_motor_data.Yangle;
  x[3] = imu_board_data.Ygyro;

  // Serial.print(">");
  // Serial.printf("x: %7.4f m", x[0]);Serial.printf(",");
  // Serial.printf("dx: %6.3f m", x[1]);Serial.printf(",");
  // Serial.printf("theta: %7.4f m", x[2]);Serial.printf(",");
  // Serial.printf("dtheta: %6.3f m", x[3]);Serial.printf(",");
  // Serial.println();

  //Apply Control
  // float u = controller.updateLQR(x[0], x[1], x[2], x[3]);
  float u = controller.updateLQG(x[0], x[1], x[2], x[3]);
  // float u = controller.updateLQGArchieve(x[0], x[1], x[2], x[3]);

  leftMotor.setTorque(u/2);
  rightMotor.setTorque(u/2); 

  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 's') { 
      leftMotor.stop();
      rightMotor.stop();
      while (1);
    }
  }

  //dynamically update control frequency
  // unsigned long now = millis();
  // static unsigned long last_time = 0;
  // float dt_dynamic = (now - last_time) / 1000.0f;
  // last_time = now;
  // controller.setDt(dt_dynamic);
  delay(dt*1000); //ms
}
