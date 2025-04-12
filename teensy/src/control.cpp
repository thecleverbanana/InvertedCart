#include "control.hpp"
#include <cmath>

Controller::Controller(Motor* left, Motor* right, float dt): motorLeft(left), motorRight(right) {}


float Controller::updateLQR(float x_meas, float dx_meas, float theta_meas, float dtheta_meas){
    float x[4] = {x_meas, dx_meas, theta_meas, dtheta_meas};

    float u = 0;
    for(int i = 0; i < 4; ++i){
        u -= K[i] * x[i];
    }
        // Torque Limit
        // if (u > TORQUE_LIMIT) u = TORQUE_LIMIT;
        // if (u < -TORQUE_LIMIT) u = -TORQUE_LIMIT;

        Serial.print(">");
        Serial.printf("tau: %6.3f Nm", u);
        Serial.println();

        return u;
}

float Controller::updateLQG(float x_meas, float dx_meas, float theta_meas, float dtheta_meas) {
    float y[4] = {x_meas, dx_meas, theta_meas, dtheta_meas};
    // compute u, u = -k_r * x_hat
    float u = 0.0f;

    for(int i=0; i<4; i++){
        u -= K[i]*x_hat[i];
    }


    // === 2. Observer update ===
    // Compute state estimate prediction: x_hat_pred = A * x_hat + B * u
    float x_hat_pred[4] = {0.0f};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            x_hat_pred[i] += A[i][j] * x_hat[j];
        }
        x_hat_pred[i] += B[i] * u;
    }

    // Measurement innovation: y - x_hat_pred (since C = Identity)
    float innovation[4];
    for (int i = 0; i < 4; i++) {
        innovation[i] = y[i] - x_hat_pred[i];
    }

    // Correct state estimate: x_hat_next = x_hat_pred + L * innovation
    float x_hat_next[4] = {0.0f};
    for (int i = 0; i < 4; i++) {
        x_hat_next[i] = x_hat_pred[i];
        for (int j = 0; j < 4; j++) {
            x_hat_next[i] += L[i][j] * innovation[j];
        }
    }

    // Update state estimate
    for (int i = 0; i < 4; i++) {
        x_hat[i] = x_hat_next[i];
    }

    // === 3. Recompute control input with updated estimate: u = -K * x_hat ===
    u = 0.0f;
    for (int i = 0; i < 4; i++) {
        u -= K[i] * x_hat[i];
    }

    u = applySmoothBoost(u);    //apply boost

    // --- Debug print ---
    Serial.print(">");
    Serial.printf("x_hat: %7.4f m", x_hat[0]); Serial.printf(",");
    Serial.printf("dx_hat: %6.3f m/s", x_hat[1]);Serial.printf(",");
    Serial.printf("theta_hat: %7.4f rad", x_hat[2]);Serial.printf(",");
    Serial.printf("dtheta_hat: %6.3f rad/s", x_hat[3]); Serial.printf(",");
    Serial.printf("tau: %6.3f Nm", u);
    Serial.println();

    return u;
}

void Controller:: setDt(float new_dt){
    dt = new_dt;
}

void Controller::initializeState(const float x_hat_init[4]) {
    for (int i = 0; i < 4; ++i) {
        this->x_hat[i] = x_hat_init[i];
    }
    
    Serial.print("Initialized x_hat: ");
    Serial.printf("x: %.4f, ", x_hat[0]);
    Serial.printf("dx: %.4f, ", x_hat[1]);
    Serial.printf("theta: %.4f, ", x_hat[2]);
    Serial.printf("dtheta: %.4f\n", x_hat[3]);
}

float Controller::applySmoothBoost(float u) {
    if (u > 0 && u < deadzone_threshold) {
        float scale = u / deadzone_threshold; 
        return u + (1.0f - scale) * max_boost;
    } else if (u < 0 && u > -deadzone_threshold) {
        float scale = -u / deadzone_threshold; 
        return u - (1.0f - scale) * max_boost;
    }
    return u;
}

float Controller::applyAggressiveBoost(float u) {
    // Parameters for tuning
    const float boost_intensity = 1.5f; // boost factor multiplier
    const float nonlinearity = 2.0f;    // 1.0f = linear, >1.0f = more aggressive

    if (u > 0 && u < deadzone_threshold) {
        float scale = powf(u / deadzone_threshold, nonlinearity); 
        return u + (1.0f - scale) * max_boost * boost_intensity;
    } else if (u < 0 && u > -deadzone_threshold) {
        float scale = powf(-u / deadzone_threshold, nonlinearity); 
        return u - (1.0f - scale) * max_boost * boost_intensity;
    }
    return u;
}

float Controller::updateLQGArchieve(float x_meas, float dx_meas, float theta_meas, float dtheta_meas) {
    float y[4] = {x_meas, dx_meas, theta_meas, dtheta_meas};
    // compute u, u = -k_r * x_hat
    float u = 0.0f;

    for(int i=0; i<4; i++){
        u -= K[i]*x_hat[i];
    }

    // --- 4. Debug print ---
    Serial.print(">");
    Serial.printf("x_hat: %7.4f m", x_hat[0]); Serial.printf(",");
    Serial.printf("dx_hat: %6.3f m/s", x_hat[1]);Serial.printf(",");
    Serial.printf("theta_hat: %7.4f rad", x_hat[2]);Serial.printf(",");
    Serial.printf("dtheta_hat: %6.3f rad/s", x_hat[3]); Serial.printf(",");
    Serial.printf("tau: %6.3f Nm", u);
    Serial.println();

    // compute dx_hat, dx_hat = A*x_hat+B*u+k_f * (y-y_hat) 
    // use linearized system here
    float dx_hat[4];
    for (int i = 0; i < 4; i++) {
        dx_hat[i] = 0.0f; 
        for (int j = 0; j < 4; j++) {
            dx_hat[i] += A[i][j] * x_hat[j];    //A*x_hat
        }
    }

    for (int i = 0; i < 4; i++) {
        dx_hat[i] += B[i]*u;    //B*u
    }

    float yMinusyhat[4];
    for(int i = 0; i < 4; i++){
        yMinusyhat[i] = y[i] - x_hat[i]; // Here assume x_hat is full state observable with identity matrix C
        for(int j = 0; j < 4; j++){
            dx_hat[i] += L[i][j] * yMinusyhat[j];
        }
    }

    // compute x_hat+1, x_hat+1 = x_hat+dx_hat*dt
     for(int i=0; i<4; i++){
        x_hat[i] = x_hat[i]+dt*dx_hat[i];
     }

    // Apply LQR Full State
    for(int i=0; i<4; i++){
        u -= K[i]*x_hat[i]; 
    }

    u = applySmoothBoost(u);    //apply boost

    return u;
}
    