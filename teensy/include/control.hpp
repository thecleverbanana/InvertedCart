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
    float lowPassFilter(float new_value, float prev_filtered_value, float alpha = 0.9f);
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
        { 1.0f, 0.001f, -4.0835307596351804e-07f, -1.3611754973143657e-10f },
        { 0.0f, 1.0f, -0.0008167082857744718f, -4.0835307596351804e-07f },
        { 0.0f, 0.0f, 1.0000156765376695f, 0.0010000052255070954f },
        { 0.0f, 0.0f, 0.03135315725696304f, 1.0000156765376695f }
    };
    
    //B Matrix(discrete)
    float B[4] = {
        -8.402255994743346e-06f,
        -0.01680452145978618f,
        6.957456478504361e-05f,
        0.13914949313170924f
    };

    // LQR gain(K)(discrete)
    float K[4] = {
        0.40463767f,
        1.34215075f,
        3.83143639f,
        1.46719294f
    };
    
    // LQG observer gain (L)(Discrete)
    float L[4][4] = {
        { 0.9901951361173478f,     0.0009902903411694102f,   -3.750025412712118e-07f,  -2.376859497282838e-10f },
        { 9.520465457924206e-08f,  0.9901951415017708f,     -0.0007487858594328596f,  -5.639928032586026e-07f },
        { -3.0257512020851732e-09f, -6.161114743338966e-06f,  0.9160940367957012f,      0.0011262909522156796f },
        { -1.840569314394952e-10f, -3.7553167240272156e-07f,  0.028932183774827494f,    0.9161068879605732f }
    };
    
    
    //Control frequency;
    float dt = 0.005f;

    //Global Variable
    float x_hat[4] = {0.0f, 0.0f, 0.0f, 0.0f}; 
    float u = 0.0f;
    float u_filtered = u; 

    const float deadzone_threshold = 0.2f;
    const float max_boost = 0.05f;
    const float TORQUE_LIMIT = 2.0f;
};

#endif
