#include "control.hpp"
#include <cmath>

Controller::Controller(Motor* left, Motor* right, float dt): motorLeft(left), motorRight(right) {}

void Controller::nonlinearDynamics(const float x_hat[], float tau, float dx_hat[]) {
    float pos = x_hat[0];
    float vel = x_hat[1];
    float theta = x_hat[2];
    float dtheta = x_hat[3];

    float x_ddot = xdd_solution(pos, vel, theta, dtheta, tau);
    float theta_ddot = thetadd_solution(pos, vel, theta, dtheta, tau);

    dx_hat[0] = vel;
    dx_hat[1] = x_ddot;
    dx_hat[2] = dtheta;
    dx_hat[3] = theta_ddot;
}


float Controller::updateLQR(float x_meas, float dx_meas, float theta_meas, float dtheta_meas){
    float x[4] = {x_meas, dx_meas, theta_meas, dtheta_meas};

    float u = 0;
    for(int i = 0; i < 4; ++i){
        u -= K[i] * x[i];
    }
        // Torque Limit
        // if (u > TORQUE_LIMIT) u = TORQUE_LIMIT;
        // if (u < -TORQUE_LIMIT) u = -TORQUE_LIMIT;

        return u;
}

float Controller::updateLQG(float x_meas, float dx_meas, float theta_meas, float dtheta_meas) {
    // Measurement
    float y[4] = { x_meas, dx_meas, theta_meas, dtheta_meas };

    // --- 1. Compute control input based on current estimate ---
    float tau = 0.0f;
    for (int i = 0; i < 4; ++i) {
        tau -= K[i] * x_hat[i];
    }
    if (abs(tau) < staticFrictionThreshold) {
        float boost = staticFrictionBoost * ((tau > 0) - (tau < 0));
        tau += boost;
    }
        
    tau = constrain(tau, -TORQUE_LIMIT, TORQUE_LIMIT);

    // --- 2. Observer prediction step ---
    float k1[4];
    nonlinearDynamics(x_hat, tau, k1);

    float x_temp[4];
    for (int i = 0; i < 4; ++i) {
        x_temp[i] = x_hat[i] + dt * k1[i];
    }

    // --- 3. Observer correction step ---
    for (int i = 0; i < 4; ++i) {
        float innovation = y[i] - x_temp[i];
        float correction = 0.0f;
        for (int j = 0; j < 4; ++j) {
            correction += L[i][j] * innovation;
        }
        x_hat[i] = x_temp[i] + correction * dt;
    }

    // --- 4. Debug print ---
    Serial.printf("x̂ = [%.3fm, %.3fm/s, %.3frad, %.3frad/s], τ = %.3f\n", x_hat[0], x_hat[1], x_hat[2], x_hat[3], tau);

    return tau;
}

void Controller:: setDt(float new_dt){
    dt = new_dt;
}

void Controller::initializeState(const float x_hat_init[4]) {
    for (int i = 0; i < 4; ++i) {
        this->x_hat[i] = x_hat_init[i];
    }
}

    