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
    Matrix4x4 inv = mat_identity();
    Matrix4x4 m = A;

    const int N = 4;

    // Forward elimination
    for (int i = 0; i < N; ++i) {
        float diag = m[i][i];
        if (std::abs(diag) < 1e-8f) {
            Serial.printf("Matrix inversion failed: singular matrix");
        }

        for (int j = 0; j < N; ++j) {
            m[i][j] /= diag;
            inv[i][j] /= diag;
        }

        for (int k = 0; k < N; ++k) {
            if (k == i) continue;
            float factor = m[k][i];
            for (int j = 0; j < N; ++j) {
                m[k][j] -= factor * m[i][j];
                inv[k][j] -= factor * inv[i][j];
            }
        }
    }

    return inv;
}

