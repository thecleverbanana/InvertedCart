#include "control.hpp"

Controller::Controller(Motor* left, Motor* right, float dt): motorLeft(left), motorRight(right) {}

void Controller::computeDerivative(const float x_hat[], float u_prev, const float y[], float dx_hat[]) {
    for (int i = 0; i < 4; ++i) {
        // Predictive term: A * x̂ + B * u_prev
        float predict = 0.0f;
        for (int j = 0; j < 4; ++j) {
            predict += A[i][j] * x_hat[j];
        }
        predict += B[i] * u_prev;

        // Correction term: L * (y - ŷ)
        float correction = 0.0f;
        for (int j = 0; j < 4; ++j) {
            correction += L[i][j] * (y[j] - x_hat[j]);
        }

        dx_hat[i] = predict + correction;
    }
}

float Controller::updateLQR(float x_meas, float dx_meas, float theta_meas, float dtheta_meas){
    float x[4] = {x_meas, dx_meas, theta_meas, dtheta_meas};

    float u = 0;
    for(int i = 0; i < 4; ++i){
        u -= K[i] * x[i];
    }
        // Torque Limit
        if (u > TORQUE_LIMIT) u = TORQUE_LIMIT;
        if (u < -TORQUE_LIMIT) u = -TORQUE_LIMIT;

        return u;
}

float Controller::updateLQG(float x_meas, float dx_meas, float theta_meas, float dtheta_meas) {
    float y[4] = { x_meas, dx_meas, theta_meas, dtheta_meas };

    float k1[4], k2[4], k3[4], k4[4], x_temp[4];

    computeDerivative(x_hat, u_prev, y, k1);

    for (int i = 0; i < 4; ++i) x_temp[i] = x_hat[i] + 0.5f * dt * k1[i];
    computeDerivative(x_temp, u_prev, y, k2);

    for (int i = 0; i < 4; ++i) x_temp[i] = x_hat[i] + 0.5f * dt * k2[i];
    computeDerivative(x_temp, u_prev, y, k3);

    for (int i = 0; i < 4; ++i) x_temp[i] = x_hat[i] + dt * k3[i];
    computeDerivative(x_temp, u_prev, y, k4);

    for (int i = 0; i < 4; ++i) {
        x_hat[i] += (dt / 6.0f) * (k1[i] + 2.0f * k2[i] + 2.0f * k3[i] + k4[i]);
    }

    // ====== Control: u = -K * x̂ ======
    float u = 0.0f;
    for (int i = 0; i < 4; ++i) {
        u -= K[i] * x_hat[i];
    }

    if (u > TORQUE_LIMIT) u = TORQUE_LIMIT;
    if (u < -TORQUE_LIMIT) u = -TORQUE_LIMIT;

    Serial.printf("x̂ = [%.3f, %.3f, %.3f, %.3f], u = %.3f\n", x_hat[0], x_hat[1], x_hat[2], x_hat[3], u);

    u_prev = u;
    return u;
}

void Controller:: setDt(float new_dt){
    dt = new_dt;
}
    