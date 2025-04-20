#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "motor.hpp"
#include "utils.hpp"
#include <BasicLinearAlgebra.h>
#include <cmath>

using namespace BLA;

using Vectorx4 = Matrix<4,1>;
using Matrix4x4 = Matrix<4,4>;

class Controller {
public:
    Controller(Motor* left, Motor* right, float dt);
    void initializeState(const float x_hat_init[4]);
    void setDt(float new_dt);
    float* get_xhat();
    float updateLQR(float x_meas, float dx_meas, float theta_meas, float dtheta_meas); //LQR Control (With feedback) and return u
    float updateLQG(float x_meas, float dx_meas, float theta_meas, float dtheta_meas);  // LQG return u
    float updateEKF_LQG(float x_meas, float dx_meas, float theta_meas, float dtheta_meas);  // LQG return u


private:
    Motor* motorLeft;
    Motor* motorRight;
    //Control frequency;
    float dt = 0.005f;

    float lowPassFilter(float new_value, float prev_filtered_value, float alpha = 0.7f);
    float applySmoothBoost(float u);
    float applyAggressiveBoost(float u);
 
    // For Extended Kalman Filter
    static Vectorx4 f_func(const Vectorx4& x, float tau);
    static Matrix4x4 A_func(const Vectorx4& x, float tau);
    static Matrix4x4 W_func(const Vectorx4& x, float tau);
    static Vectorx4 h_func(const Vectorx4& x);
    static Matrix4x4 H_func(const Vectorx4& x);
    static Matrix4x4 V_func(const Vectorx4& x);
  
    //Global Variable
    Vectorx4 x_hat = {0.0f, 0.0f, 0.0f, 0.0f};
    float u_prev = 0.0f;
    float u_filtered; 
    const float deadzone_threshold = 0.2f;
    const float max_boost = 0.05f;
    const float TORQUE_LIMIT = 2.0f;

    Matrix4x4 P = {
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
    };
    
     //A Matrix(discrete)
     Matrix4x4 A = {
        1.0f, 0.005000000000000001f, -1.020946706937547e-05f, -1.70153338817461e-08f,
        0.0f, 1.0f, -0.0040840535720604145f, -1.0209467069375469e-05f,
        0.0f, 0.0f, 1.0003919380176636f, 0.005000653212962653f,
        0.0f, 0.0f, 0.1567854472900883f, 1.0003919380176636f
    };
    
    
    //B Matrix(discrete)
    Vectorx4 B = {
        -0.00021005924102969215f,
        -0.08402488025869838f,
        0.0017394731908484315f,
        0.6958347238219198f
    };

    // LQR gain(K)(discrete)
    Vectorx4 K = {
        0.10045949f,
        10.08750993f,
        12.59321803f,
        2.54164549f
    };
              
    
    // LQG observer gain (L)(Discrete)
    Matrix4x4 L = {
        0.9901957837493872f, 0.004276628219945077f, -9.590081735450662e-06f, -6.184615346918055e-08f,
        0.00012235540025459888f, 0.8541021397161006f, -0.0037876014884929425f, -2.0569629756478816e-05f,
        -5.944112796015694e-08f, -2.3178505204012516e-05f, 0.9164361537533032f, 0.005631771695017522f,
        -1.8276992328721056e-08f, -7.09565788661099e-06f, 0.14467674291694385f, 0.9167570418246586f
    };
    
    // Process noise covariance matrix (Q)
    Matrix4x4 Q = {
        1.0f, 0.0f,    0.0f,    0.0f,
        0.0f,  10.0f,   0.0f,    0.0f,
        0.0f,  0.0f,    1.0f,   0.0f,
        0.0f,  0.0f,    0.0f,   1.0f
    };

    // Measurement noise covariance matrix (R)
    Matrix4x4 R = {
        1e-3f, 0.0f,  0.0f,  0.0f,
        0.0f,    1e-4f, 0.0f,  0.0f,
        0.0f,    0.0f,   1e-4f,  0.0f,
        0.0f,    0.0f,   0.0f,    1e-4f
    };

    // Kalman Gain matrix (K_k)
    Matrix4x4 K_k = {
        0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f
    };

    
};

#endif
