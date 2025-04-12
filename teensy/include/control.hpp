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
    float applyAggressiveBoost(float u);
    float updateLQGArchieve(float x_meas, float dx_meas, float theta_meas, float dtheta_meas);
       
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
    
    //A Matrix(discrete)
    float A[4][4] = {
        { 1.0f, 0.010000000000000002f, -4.084587123407103e-05f, -1.3613867636970994e-07f },
        { 0.0f, 1.0f, -0.00817130853584296f, -4.0845871234071025e-05f },
        { 0.0f, 0.0f, 1.001568059301474f, 0.01000522631813993f },
        { 0.0f, 0.0f, 0.3136937949349954f, 1.001568059301474f }
    };
    
    //B Matrix(discrete)
    float B[4] = {
        -0.0008402724823072886f,
        -0.16806396872079393f,
        0.006959256294742129f,
        1.3922148958083924f
    };

    // LQR gain(K)(discrete)
    float K[4] = {
        0.5076895145840308f,
        1.6829907936997723f,
        4.740232721129529f,
        1.8351590935176787f
    };
    
    // LQG observer gain (L)(Discrete)
    float L[4][4] = {
        { 0.9990019950159471f, 0.0009990029900769537f, -4.043610932762708e-07f, -1.3641422407706974e-10f },
        { 9.950564185421027e-10f, 0.9990019950851031f, -0.0008087084885336889f, -4.071144193767245e-07f },
        { -3.949115103141783e-11f, -7.913878536090421e-08f, 0.9902106639517242f, 0.0009932805545049721f },
        { -2.4771302172628164e-12f, -4.977245281460048e-09f, 0.031048824026740078f, 0.99021085095184f }
    };
    
    //Control frequency;
    float dt = 0.005f;

    //Global Variable
    float x_hat[4] = {0.0f, 0.0f, 0.0f, 0.0f};  

    const float deadzone_threshold = 0.2f;
    const float max_boost = 0.05f;
    const float TORQUE_LIMIT = 1.0f;
};

#endif
