#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "motor.hpp"

class Controller {
public:
    Controller(Motor* left, Motor* right, float dt);
    float updateLQR(float x_meas, float dx_meas, float theta_meas, float dtheta_meas); //LQR Control (With feedback) and return u
    float updateLQG(float x_meas, float dx_meas, float theta_meas, float dtheta_meas);  // LQG return u
    float updateLQGNonLinear(float x_meas, float dx_meas, float theta_meas, float dtheta_meas);  // LQG return u
    void setDt(float new_dt);
    void initializeState(const float x_hat_init[4]);
    float applySmoothBoost(float u);
       
private:
    Motor* motorLeft;
    Motor* motorRight;

    //Non-linear solution function
    float xdd_solution(float pos, float vel, float theta, float dtheta, float tau) {
        float cos_theta = cosf(theta);
        float sin_theta = sinf(theta);
        float cos_theta_squared = cos_theta * cos_theta;
    
        float denominator = 5.082572504e-5f - 3.90615696e-6f * cos_theta_squared;
    
        float term1 = -0.000118584f * tau * cos_theta / denominator;
        float term2 = -0.000669876f * tau / denominator;
        float term3 = -3.83193997776e-5f * sin_theta * cos_theta / denominator;
        float term4 = 1.3239429264e-6f * sin_theta * dtheta * dtheta / denominator;
    
        return term1 + term2 + term3 + term4;
    }

    float thetadd_solution(float pos, float vel, float theta, float dtheta, float tau) {
        float cos_theta = cosf(theta);
        float sin_theta = sinf(theta);
        float cos_theta_squared = cos_theta * cos_theta;
    
        float denominator = 5.082572504e-5f - 3.90615696e-6f * cos_theta_squared;
    
        float term1 = 0.0019764f * tau * cos_theta / denominator;
        float term2 = 0.0045524f * tau / denominator;
        float term3 = -3.90615696e-6f * sin_theta * cos_theta * dtheta * dtheta / denominator;
        float term4 = 0.00147106890936f * sin_theta / denominator;
    
        return term1 + term2 + term3 + term4;
    }
    
    //A Matrix
    float A[4][4] = {
        { 0.0f, 1.0f, 0.0f, 0.0f },
        { 0.0f, 0.0f, -0.81670402f, 0.0f },
        { 0.0f, 0.0f, 0.0f, 1.0f },
        { 0.0f, 0.0f, 31.35299342f, 0.0f }
    };

    //B Matrix
    float B[4] = { 
        0.0f,
        -16.80450252f, 
        0.0f, 
        139.14876601f };

    // LQR gain(K)
    float K[4] = {
        0.31622777f,
        0.850273f,
        3.15043909f,
        1.53313583f
    };

    // LQG observer gain (L)
    float L[4][4] = {
        { 40.1399430f, 0.124587778f, -0.000167889867f, -0.0458903804f },
        { 5.98021335f, 5.71268719f, -0.0137972870f, -3.66294922f },
        { -0.0329064140f, -0.0563389220f, 0.619861936f, 140.747489f },
        { -0.00734246086f, -0.0122098307f, 0.114895909f, 169.733597f }
    };
    
    //Control frequency;
    float dt = 0.005;

    //Global Variable
    float x_hat[4] = {0.0f, 0.0f, 0.0f, 0.0f};  

    const float deadzone_threshold = 0.2f;
    const float max_boost = 0.05f;
    const float TORQUE_LIMIT = 1.0f;
};

#endif
