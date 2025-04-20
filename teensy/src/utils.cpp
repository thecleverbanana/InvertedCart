#include "utils.hpp"

float fusedPositionEstimate(float encoderPosition, float accelX, float dt) {
    static float accelVelocity = 0.0f;
    static float accelPosition = 0.0f;

    const float alpha = 0.98f;

    // Deadband to remove noise
    if (fabs(accelX) < 0.05f) accelX = 0.0f;

    // Leaky integration
    accelVelocity += accelX * dt;
    accelVelocity *= 0.98f;

    accelPosition += accelVelocity * dt;
    accelPosition *= 0.995f;

    // Sensor fusion
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

bool invert4x4(const Matrix4x4& input, Matrix4x4& output) {
    // Create an augmented matrix [input | I]
    float aug[4][8];

    // Initialize the augmented matrix
    for (int i = 0; i < 4; ++i){
        for (int j = 0; j < 4; ++j){
            aug[i][j] = input(i, j);               // Left: original matrix
            aug[i][j + 4] = (i == j) ? 1.0f : 0.0f; // Right: identity matrix
        }
    }

    // Perform Gauss-Jordan elimination
    for (int i = 0; i < 4; ++i){
        float pivot = aug[i][i];
        if (fabs(pivot) < 1e-6f) return false; // Singular matrix check

        // Normalize the pivot row
        for (int j = 0; j < 8; ++j){
            aug[i][j] /= pivot;
        }

        // Eliminate other rows
        for (int k = 0; k < 4; ++k){
            if (k == i) continue;
            float factor = aug[k][i];
            for (int j = 0; j < 8; ++j){
                aug[k][j] -= factor * aug[i][j];
            }
        }
    }

    // Copy the right half into output
    for (int i = 0; i < 4; ++i){
        for (int j = 0; j < 4; ++j){
            output(i, j) = aug[i][j + 4];
        }
    }

    return true;
}

// Function to compute x-velocity from x-acceleration
float computeVelocity(const float prev_acc, const float curr_acc, float dt, float prev_velocity) {
    float avg_acc = 0.5f * (prev_acc + curr_acc);

    if (fabs(avg_acc) < 0.05f) avg_acc = 0.0f;

    float velocity = prev_velocity + avg_acc * dt;
    velocity *= 0.98f; // decay

    return velocity;
}
