#include <Arduino.h>
#include "motor.hpp"
#include "control.hpp"
#include "mpu6050.hpp" 
#include "utils.hpp"

//frequency
float dt = 0.005f; // s
const unsigned long control_period_us = 5000;
unsigned long next_time = 0;

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
float z_k[4];   //Real Data

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  // while (!Serial);

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

  //Initialize intial state z_k[] value for controller
  imu_board.update();
  imu_motor.update();

  imu_board_data = imu_board.getIMUData();
  imu_motor_data = imu_motor.getIMUData();

  //Initialize x_hat for LQG
  z_k[0] = -radians(leftMotor.getCurrentDeltaAngle())*wheelRadius;
  Xacc = imu_motor_data.Xacc;
  z_k[1] = computeVelocity(prev_Xacc,Xacc,dt,prev_Xvel); prev_Xacc = Xacc;prev_Xvel = z_k[1];
  z_k[2] = imu_motor_data.Yangle;
  z_k[3] = imu_motor_data.Ygyro;

  controller.initializeState(z_k);
}

void loop() {
  unsigned long now = micros();

  if((long)(now - next_time) >= 0){
    next_time += control_period_us;  // Schedule next loop
    unsigned long start_time = micros();  // Start timing

    // --- Start control logic ---
    imu_board.update();
    imu_motor.update();

    imu_board_data = imu_board.getIMUData();
    imu_motor_data = imu_motor.getIMUData();

    Xacc = imu_motor_data.Xacc;
    z_k[0] = fusedPositionEstimate((-radians(leftMotor.getCurrentDeltaAngle()) * wheelRadius),Xacc,dt);
    z_k[1] = computeVelocity(prev_Xacc, Xacc, dt, prev_Xvel); prev_Xacc = Xacc; prev_Xvel = z_k[1];
    z_k[2] = imu_motor_data.Yangle;
    z_k[3] = imu_motor_data.Ygyro;

    float u = controller.updateEKF_LQG(z_k[0], z_k[1], z_k[2], z_k[3]);
    float* x_hat = controller.get_xhat();

    // --- Debug print ---
    // Serial.print(">");
    // Serial.print("x_hat:");     Serial.print(x_hat[0]); Serial.print(",");
    // Serial.print("dx_hat:");    Serial.print(x_hat[1]); Serial.print(",");
    // Serial.print("theta_hat:"); Serial.print(x_hat[2]); Serial.print(",");
    // Serial.print("dtheta_hat:");Serial.print(x_hat[3]); Serial.print(",");
    // Serial.print("tau:");       Serial.print(u);
    // Serial.print("\r\n"); 

    // Serial.print(">");
    // Serial.print("zx_hat:");     Serial.print(z_k[0]); Serial.print(",");
    // Serial.print("dzx_hat:");    Serial.print(z_k[1]); Serial.print(",");
    // Serial.print("ztheta_hat:"); Serial.print(z_k[2]); Serial.print(",");
    // Serial.print("dztheta_hat:");Serial.print(z_k[3]); Serial.print(",");
    // // Serial.print("tau:");       Serial.print(u);
    // Serial.print("\r\n"); 


    // --- Apply torque  ---
    leftMotor.setTorque(u / 2);
    rightMotor.setTorque(u / 2);


    unsigned long now = micros();
    unsigned long loop_time_us = now - start_time;
    float loop_freq_hz = 1e6f / (float)loop_time_us;
    Serial.print("Control Frequency: ");
    Serial.println(loop_freq_hz);

    // Catch up if fall behind too much
    if ((long)(micros() - next_time) > 5 * control_period_us) {
      next_time = micros() + control_period_us;
    }
  }
}



