#include "utils.hpp"
#include <cmath> // for std::sqrt, etc.
#include <stdexcept> // for exception handling (if needed)
#include <Arduino.h>

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

// =====================
// Matrix operations
// =====================

Matrix4x4 mat_mul(const Matrix4x4& A, const Matrix4x4& B) {
    Matrix4x4 result = { 0.0f };
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            for (int k = 0; k < 4; ++k)
                result[i][j] += A[i][k] * B[k][j];
    return result;
}

Matrix4x4 mat_transpose(const Matrix4x4& A) {
    Matrix4x4 result = { 0.0f };
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            result[i][j] = A[j][i];
    return result;
}

Matrix4x4 mat_identity() {
    Matrix4x4 I = { 0.0f };
    for (int i = 0; i < 4; ++i)
        I[i][i] = 1.0f;
    return I;
}

Matrix4x4 mat_add(const Matrix4x4& A, const Matrix4x4& B) {
    Matrix4x4 result = { 0.0f };
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            result[i][j] = A[i][j] + B[i][j];
    return result;
}

Matrix4x4 mat_scalar_mul(const Matrix4x4& A, float scalar) {
    Matrix4x4 result = { 0.0f };
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            result[i][j] = A[i][j] * scalar;
    return result;
}

State state_add(const State& a, const State& b) {
    State result = { 0.0f };
    for (int i = 0; i < 4; ++i)
        result[i] = a[i] + b[i];
    return result;
}

State state_scalar_mul(const State& a, float scalar) {
    State result = { 0.0f };
    for (int i = 0; i < 4; ++i)
        result[i] = a[i] * scalar;
    return result;
}

// =====================
// 4x4 Matrix inversion
// (Gauss-Jordan elimination)
// =====================

Matrix4x4 inverse_matrix(const Matrix4x4& A) {
    Matrix4x4 inv = mat_identity(); // Initialize inverse as identity matrix
    Matrix4x4 m = A;                // Copy input matrix to work on

    const int N = 4;                // Matrix size (4x4)
    const float epsilon = 1e-8f;    // Threshold for singularity check

    for (int i = 0; i < N; ++i) {
        // Step 1: Partial Pivoting
        // Find the row with the largest absolute value in column i (from row i to N-1)
        int maxRow = i;
        float maxVal = std::abs(m[i][i]);
        for (int k = i + 1; k < N; ++k) {
            float val = std::abs(m[k][i]);
            if (val > maxVal) {
                maxVal = val;
                maxRow = k;
            }
        }

        // Step 2: Check for singularity
        if (maxVal < epsilon) {
            Serial.printf("Matrix inversion failed: singular matrix\n");
            return inv; // Return partially computed inverse (could throw exception instead)
        }

        // Step 3: Swap rows if necessary
        if (maxRow != i) {
            for (int j = 0; j < N; ++j) {
                std::swap(m[i][j], m[maxRow][j]);
                std::swap(inv[i][j], inv[maxRow][j]);
            }
        }

        // Step 4: Normalize the pivot row
        float diag = m[i][i]; // Pivot element after swapping
        for (int j = 0; j < N; ++j) {
            m[i][j] /= diag;
            inv[i][j] /= diag;
        }

        // Step 5: Eliminate other rows (above and below pivot)
        for (int k = 0; k < N; ++k) {
            if (k == i) continue; // Skip the pivot row
            float factor = m[k][i];
            for (int j = 0; j < N; ++j) {
                m[k][j] -= factor * m[i][j];
                inv[k][j] -= factor * inv[i][j];
            }
        }
    }

    return inv; // Return the computed inverse
}

