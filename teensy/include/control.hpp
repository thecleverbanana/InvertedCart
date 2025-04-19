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
    float dt = 0.003f;

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
        1.0f, 0.003f, -3.6752545028216145e-06f, -3.675219934100914e-09f,
        0.0f, 1.0f, -0.0024502272833919086f, -3.675254502821614e-06f,
        0.0f, 0.0f, 1.0001410917880882f, 0.0030001410904610063f,
        0.0f, 0.0f, 0.09406340387331616f, 1.0001410917880882f
    };
    
    
    //B Matrix(discrete)
    Vectorx4 B = {
        -7.562064488632212e-05f,
        -0.05041401895990994f,
        0.0006261841713932136f,
        0.4174659305900801f
    };

    // LQR gain(K)(discrete)
    Vectorx4 K = {
        0.8841099680413509f,
        1.790325725994268f,
        4.295728247270997f,
        1.494470010838359f
    };
    
    // LQG observer gain (L)(Discrete)
    Matrix4x4 L = {
        0.9901953691435345f, 0.0025659766908494117f, -3.4523806813966764e-06f, -1.336854477408982e-08f,
        7.34131649354905e-05f, 0.8541020287786182f, -0.002272406659160465f, -7.40848073926294e-06f,
        -2.1404152941015696e-08f, -1.3909536985404544e-05f, 0.916208070313337f, 0.0033789361648707047f,
        -3.9514084130858505e-09f, -2.5565386396473353e-06f, 0.08679971559349156f, 0.9163236832633308f
    };
    
    // Process noise covariance matrix (Q)
    Matrix4x4 Q = {
        1.0f, 0.0f,    0.0f,    0.0f,
        0.0f,  1.0f,   0.0f,    0.0f,
        0.0f,  0.0f,    1.0f,   0.0f,
        0.0f,  0.0f,    0.0f,    1.0f
    };

    // Measurement noise covariance matrix (R)
    Matrix4x4 R = {
        1e-06f, 0.0f,  0.0f,  0.0f,
        0.0f,    1e-06f, 0.0f,  0.0f,
        0.0f,    0.0f,   1e-06f,  0.0f,
        0.0f,    0.0f,   0.0f,    1e-06f
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
