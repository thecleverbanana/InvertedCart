#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "motor.hpp"

class Controller {
public:
    Controller(Motor* left, Motor* right, float dt);
    void computeDerivative(const float x_hat[], float u_prev, const float y[], float dx_hat[]);
    float updateLQR(float x_meas, float dx_meas, float theta_meas, float dtheta_meas); //LQR Control (With feedback) and return u
    float updateLQG(float x_meas, float dx_meas, float theta_meas, float dtheta_meas);  // LQG return u
    void setDt(float new_dt);
       
private:
    Motor* motorLeft;
    Motor* motorRight;

    //A Matrix
    float A[4][4] = {
        {  0.0f,           1.0f,            0.0f,           0.0f },
        { -13.61173363f,   0.0f,           -0.81670402f,    0.0f },
        {  0.0f,           0.0f,            0.0f,           1.0f },
        { 749.41211761f,   0.0f,           44.96472706f,    0.0f }
    };

    //B Matrix
    float B[4] = {
        0.0f,
       -16.80450252f,
        0.0f,
       419.223808f
    };

    // LQR gain(K)
    float K[4] = {
        886.37834069f,
        218.24558569f,
         46.8581451f,
          9.37081155f
    };

    // LQG observer gain (L)
    float L[4][4] = {
        {  0.489828325f,   0.0292953722f,  0.0f,            3.13333700f },
        {  0.0292953722f,  3.02996405f,    0.0f,           -0.339121813f },
        { -3.27900841f,   -0.0575885289f,  0.0f,            1.49475998f },
        {  3.13333700f,   -0.339121813f,   0.0f,           69.5040400f }
    };    
    
    //Control frequency;
    float dt = 0.005;

    float x_hat[4] = {0.0f, 0.0f, 0.0f, 0.0f};  
    float u_prev = 0.0f;

    const float TORQUE_LIMIT = 1.0f;
};

#endif
