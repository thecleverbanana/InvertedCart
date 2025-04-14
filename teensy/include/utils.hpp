#ifndef UTILS_HPP
#define UTILS_HPP
#include <array>
#include <cmath>

using State = std::array<float, 4>;
using Matrix4x4 = std::array<std::array<float, 4>, 4>;

// Estimates the fused position using complementary filter.
// Combines encoder-based position estimation with acceleration-based integration
// to improve accuracy and reduce noise.
//
// @param encoderPosition  Position estimated from the encoder (in meters).
// @param accelX           Linear acceleration in the X direction (in m/s^2).
// @param dt               Time step duration (in seconds).
// @return                 Fused position estimate (in meters).
float fusedPositionEstimate(float encoderPosition, float accelX, float dt);

/// @brief Matrix multiplication: C = A * B
/// @param A Matrix A (size 4x4)
/// @param B Matrix B (size 4x4)
/// @return Result matrix C = A * B
Matrix4x4 mat_mul(const Matrix4x4& A, const Matrix4x4& B);

/// @brief Matrix transpose: B = A^T
/// @param A Input matrix (size 4x4)
/// @return Transposed matrix B = A^T
Matrix4x4 mat_transpose(const Matrix4x4& A);

/// @brief Identity matrix generator: I = eye(4)
/// @return Identity matrix of size 4x4
Matrix4x4 mat_identity();

/// @brief Matrix addition: C = A + B
/// @param A Matrix A (size 4x4)
/// @param B Matrix B (size 4x4)
/// @return Sum matrix C = A + B
Matrix4x4 mat_add(const Matrix4x4& A, const Matrix4x4& B);

/// @brief Scalar-matrix multiplication: B = scalar * A
/// @param A Matrix A (size 4x4)
/// @param scalar Scalar multiplier
/// @return Scaled matrix B = scalar * A
Matrix4x4 mat_scalar_mul(const Matrix4x4& A, float scalar);

/// @brief State vector addition: c = a + b
/// @param a Vector a (size 4)
/// @param b Vector b (size 4)
/// @return Sum vector c = a + b
State state_add(const State& a, const State& b);

/// @brief Scalar-state multiplication: b = scalar * a
/// @param a Vector a (size 4)
/// @param scalar Scalar multiplier
/// @return Scaled vector b = scalar * a
State state_scalar_mul(const State& a, float scalar);

/// @brief Matrix inversion: B = A^-1
/// Computes the inverse of a 4x4 matrix using Gauss-Jordan elimination.
/// 
/// @param A Input matrix (size 4x4).
/// @return Inverse matrix B = A^-1.
/// 
/// @note If the matrix is singular (non-invertible), returns identity matrix as a fallback.
/// @warning No exception is thrown; ensure matrix is invertible for reliable results.
Matrix4x4 inverse_matrix(const Matrix4x4& A);

#endif