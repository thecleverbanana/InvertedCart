#ifndef UTILS_HPP
#define UTILS_HPP

// Estimates the fused position using complementary filter.
// Combines encoder-based position estimation with acceleration-based integration
// to improve accuracy and reduce noise.
//
// @param encoderPosition  Position estimated from the encoder (in meters).
// @param accelX           Linear acceleration in the X direction (in m/s^2).
// @param dt               Time step duration (in seconds).
// @return                 Fused position estimate (in meters).
float fusedPositionEstimate(float encoderPosition, float accelX, float dt);




#endif