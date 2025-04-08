#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "motor.hpp"

class Controller {
public:
    Controller(Motor* left, Motor* right, float dt);
    void nonlinearDynamics(const float x_hat[], float tau, float dx_hat[]);
    float updateLQR(float x_meas, float dx_meas, float theta_meas, float dtheta_meas); //LQR Control (With feedback) and return u
    float updateLQG(float x_meas, float dx_meas, float theta_meas, float dtheta_meas);  // LQG return u
    void setDt(float new_dt);
    void initializeState(const float x_hat_init[4]);
       
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
        0.4472136f,
        1.04725721f,
        3.20271011f,
        1.150277f
    };

    // LQG observer gain (L)
    float L[4][4] = {
        { 3.21614944f, 0.149442707f, -0.00000537035564f, -0.0276865438f },
        { 5.37993746f, 0.567016431f, -0.0000699907853f, -0.328123735f },
        { -0.105258971f, -0.0381060942f, 0.000697209258f, 2.86961089f },
        { -0.4429847f, -0.145832771f, 0.0023425395f, 14.3134533f }
    };
            
    //Control frequency;
    float dt = 0.005;

    //Global Variable
    float x_hat[4] = {0.0f, 0.0f, 0.0f, 0.0f};  

    const float staticFrictionThreshold = 0.15f;
    const float staticFrictionBoost = 0.05f;
    const float TORQUE_LIMIT = 1.0f;
};

#endif
