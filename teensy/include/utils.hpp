#ifndef UTILS_HPP
#define UTILS_HPP
#include <array>
#include <cmath>
#include <Arduino.h>
#include <BasicLinearAlgebra.h>

using namespace BLA;

using Vectorx4 = Matrix<4,1>;
using Matrix4x4 = Matrix<4,4>;

// Estimates the fused position using complementary filter.
// Combines encoder-based position estimation with acceleration-based integration
// to improve accuracy and reduce noise.
//
// @param encoderPosition  Position estimated from the encoder (in meters).
// @param accelX           Linear acceleration in the X direction (in m/s^2).
// @param dt               Time step duration (in seconds).
// @return                 Fused position estimate (in meters).
float fusedPositionEstimate(float encoderPosition, float accelX, float dt);

Matrix4x4 makeIdentity();

float computeVelocity(const float prev_acc, const float curr_acc,float dt, float prev_velocity);

#endif