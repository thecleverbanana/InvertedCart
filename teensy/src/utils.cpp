#include "utils.hpp"

float fusedPositionEstimate(float encoderPosition, float accelX, float dt) {
    static float accelVelocity = 0.0f;
    static float accelPosition = 0.0f;

    const float alpha = 0.98f;

    // 1. Acc Integration
    accelVelocity += accelX * dt;
    accelPosition += accelVelocity * dt;

    // 2. fusion filter
    float fusedPosition = alpha * encoderPosition + (1.0f - alpha) * accelPosition;

    return fusedPosition;
}

Matrix4x4 makeIdentity() {
    Matrix4x4 I = Zeros<4,4>();
    for (int i = 0; i < 4; ++i) {
        I(i,i) = 1.0f;
    }
    return I;
}


// Function to compute x-velocity from x-acceleration
float computeVelocity(const float prev_acc, const float curr_acc,float dt, float prev_velocity) {
    // Trapezoidal rule: v[n] = v[n-1] + (dt/2)*(a[n] + a[n-1])
    float velocity = prev_velocity + (dt / 2.0f) * (curr_acc + prev_acc);

    return velocity;
}