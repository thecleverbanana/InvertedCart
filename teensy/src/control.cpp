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

float Controller::updateLQGNonLinear(float x_meas, float dx_meas, float theta_meas, float dtheta_meas) {
    float y[4] = {x_meas, dx_meas, theta_meas, dtheta_meas};
    // compute u, u = -k_r * x_hat
    float u = 0.0f;

    for(int i=0; i<4; i++){
        u -= K[i]*x_hat[i];
    }


    // compute dx_hat, dx_hat = [dx, ddx, dtheta, ddtheta]
    // use linearized system here
    float dx_hat[4];
    dx_hat[0] = x_hat[1];
    dx_hat[1] = xdd_solution(x_hat[0], x_hat[1], x_hat[2], x_hat[3], u);
    dx_hat[2] = x_hat[3];
    dx_hat[3] = thetadd_solution(x_hat[0], x_hat[1], x_hat[2], x_hat[3], u);

    // Compute measurement error
    float y_hat[4] = {x_hat[0], x_hat[1], x_hat[2], x_hat[3]}; // y_hat = C * x_hat
    float y_error[4];
    for (int i = 0; i < 4; i++) {
        y_error[i] = y[i] - y_hat[i];
    }

    // Observer correction term: dx_hat += L * (y - y_hat)
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            dx_hat[i] += L[i][j] * y_error[j];
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

    