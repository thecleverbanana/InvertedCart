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



