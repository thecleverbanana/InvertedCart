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
    float updateLQR(float x_meas, float dx_meas, float theta_meas, float dtheta_meas); //LQR Control (With feedback) and return u
    float updateLQG(float x_meas, float dx_meas, float theta_meas, float dtheta_meas);  // LQG return u
    float updateEKF_LQG(float x_meas, float dx_meas, float theta_meas, float dtheta_meas);  // LQG return u
    float updateLQGArchieve(float x_meas, float dx_meas, float theta_meas, float dtheta_meas);
    void debugFunc(Vectorx4& x_test, float u_test);


private:
    Motor* motorLeft;
    Motor* motorRight;
    //Control frequency;
    float dt = 0.005f;

    float lowPassFilter(float new_value, float prev_filtered_value, float alpha = 0.9f);
    float applySmoothBoost(float u);
    float applyAggressiveBoost(float u);
 
    // For Extended Kalman Filter
    static Vectorx4 f_func(const Vectorx4& x, float tau);
    static Matrix4x4 A_func(const Vectorx4& x, float tau);
    static Matrix4x4 W_func(const Vectorx4& x, float tau);
    static Vectorx4 h_func(const Vectorx4& x);
    static Matrix4x4 H_func(const Vectorx4& x);
    static Matrix4x4 V_func(const Vectorx4& x);
    void ekf_estimation(Vectorx4& x_prev, Matrix4x4& P_prev, float u_prev, const Matrix4x4& Q, float dt, Vectorx4& x_pred, Matrix4x4& P_pred);
    void ekf_correction(Vectorx4& x_pred, Matrix4x4& P_pred, Vectorx4& z_k, const Matrix4x4& R);

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
        0.0f, 0.0f, 0.0f, 10.0f
    };
    
     //A Matrix(discrete)
     Matrix4x4 A = {
        1.0f, 0.001f, -4.0835307596351804e-07f, -1.3611754973143657e-10f,
        0.0f, 1.0f, -0.0008167082857744718f, -4.0835307596351804e-07f,
        0.0f, 0.0f, 1.0000156765376695f, 0.0010000052255070954f,
        0.0f, 0.0f, 0.03135315725696304f, 1.0000156765376695f
    };
    
    //B Matrix(discrete)
    Vectorx4 B = {
        -8.402255994743346e-06f,
        -0.01680452145978618f,
        6.957456478504361e-05f,
        0.13914949313170924f
    };

    // LQR gain(K)(discrete)
    Vectorx4 K = {
        0.38283111f,
        1.26933848f,
        3.63235362f,
        1.38874116f
    };
    
    // LQG observer gain (L)(Discrete)
    Matrix4x4 L = {
        0.99019514f, 0.00495145f, -9.3755585e-06f, -2.9694885e-08f,
        4.7635137e-07f, 0.99019527f, -0.0037443736f, -1.4096054e-05f,
        -7.5620600e-08f, -3.0798161e-05f, 0.91643614f, 0.0056317714f,
        -2.2977942e-08f, -9.3770137e-06f, 0.14467674f, 0.91675705f
    };
        
    // Process noise covariance matrix (Q)
    Matrix4x4 Q = {
        0.01f, 0.0f,    0.0f,    0.0f,
        0.0f,  0.01f,   0.0f,    0.0f,
        0.0f,  0.0f,    0.01f,   0.0f,
        0.0f,  0.0f,    0.0f,    0.01f
    };

    // Measurement noise covariance matrix (R)
    Matrix4x4 R = {
        0.000000001f, 0.0f,  0.0f,  0.0f,
        0.0f,    0.000000001f, 0.0f,  0.0f,
        0.0f,    0.0f,   0.000000001f,  0.0f,
        0.0f,    0.0f,   0.0f,    0.00000001f
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
