#include "control.hpp"

Controller::Controller(Motor* left, Motor* right, float dt): motorLeft(left), motorRight(right),dt(dt) {}

void Controller::initializeState(const float x_hat_init[4]) {
    // Convert input array to Vectorx4
    Vectorx4 x_init_vec;
    for (int i = 0; i < 4; ++i) {
        x_init_vec(i,0) = x_hat_init[i];
    }

    // Initialize state estimate
    x_hat = x_init_vec;

    // Control input correction: u = -K * x_hat_init
    float u = -(~K * x_init_vec)(0,0);

    // Initialize filtered control input
    u_filtered = u;

    // Debug print
    Serial.print("Initialized x_hat and u_filtered: ");
    Serial.printf("x: %.4f, ", x_hat(0,0));
    Serial.printf("dx: %.4f, ", x_hat(1,0));
    Serial.printf("theta: %.4f, ", x_hat(2,0));
    Serial.printf("dtheta: %.4f\n", x_hat(3,0));
    Serial.printf("u_filtered: %.4f\n", u_filtered);
}

void Controller:: setDt(float new_dt){
    dt = new_dt;
}

float* Controller::get_xhat(){
    static float xhat_out[4];
    xhat_out[0] = x_hat(0,0);
    xhat_out[1] = x_hat(1,0);
    xhat_out[2] = x_hat(2,0);
    xhat_out[3] = x_hat(3,0);
    return xhat_out;
};

float Controller::updateLQR(float x_meas, float dx_meas, float theta_meas, float dtheta_meas){
    Vectorx4 x = { x_meas, dx_meas, theta_meas, dtheta_meas };

    float u = -(~K * x)(0,0);

    // Torque Limit
    if (u > TORQUE_LIMIT) u = TORQUE_LIMIT;
    if (u < -TORQUE_LIMIT) u = -TORQUE_LIMIT;

    return u;
}

float Controller::updateLQG(float x_meas, float dx_meas, float theta_meas, float dtheta_meas) {
    // === 1. Measurement vector (y) ===
    Vectorx4 y = { x_meas, dx_meas, theta_meas, dtheta_meas };

    // === 2. Control law: u = -K^T * x_hat ===
    float u = -(~K * x_hat)(0,0); // Transpose K (4x1 -> 1x4) * x_hat (4x1) = (1x1), extract scalar

    u_filtered = lowPassFilter(u,u_filtered);    //apply filter
    u = u_filtered;

    // Torque Limit
    if (u > TORQUE_LIMIT) u = TORQUE_LIMIT;
    if (u < -TORQUE_LIMIT) u = -TORQUE_LIMIT;

    // === 3. Observer update ===
    // Predict next state: x_hat_pred = A * x_hat + B * u
    Vectorx4 x_hat_pred = A * x_hat + B * u;

    // Measurement innovation: y - x_hat_pred (since C = Identity)
    Vectorx4 innovation = y - x_hat_pred;

    // Corrected state estimate: x_hat = x_hat_pred + L * innovation
    x_hat= x_hat_pred + L * innovation;
    
    return u;
}

float Controller::updateEKF_LQG(float x_meas, float dx_meas, float theta_meas, float dtheta_meas){
    Matrix4x4 I = makeIdentity();

    // x_k_priori = x_k-1 + f_func * dt
    float u = -(~K * x_hat)(0,0); 

    // === Estimation ===
    // 1. State prediction using Euler integration
    Vectorx4 x_pred = x_hat+(f_func(x_hat,u)*dt);
    Vectorx4 f_out = f_func(x_hat, u);
    
    // 2. Jacobians
    Matrix4x4 A_cont = A_func(x_hat, u);
    Matrix4x4 A_k = I + A_cont * dt;
    Matrix4x4 W_k = W_func(x_hat, u);

    // 3. Covariance prediction
    Matrix4x4 P_pred = A_k * P * (~A_k) + W_k * Q * (~W_k) * dt * dt;

    // ===== EKF Correction =====
    // 1. Jacobians
    Matrix4x4 V_k = V_func(x_pred);
    Matrix4x4 H_k = H_func(x_pred);

    //2. Kalman Gain
    // Matrix4x4 S_k = computePreciseSk(H_k, P_pred, V_k, R);
    Matrix4x4 S_k = P_pred + R;
    Matrix4x4 S_k_copy = S_k;
    Matrix4x4 S_inv;
    if (!invert4x4(S_k_copy, S_inv)) {
        // Serial.println("Matrix inversion failed!");
    } 
    K_k = P_pred * S_inv;

    //3. State update
    Vectorx4 z_k = {x_meas, dx_meas, theta_meas, dtheta_meas};
    Vectorx4 h_pred = h_func(x_pred);
    Vectorx4 y_k = z_k - h_pred;   
    x_hat = x_pred + (K_k * y_k);

    //4. Covariance update    
    P = (I - K_k * H_k) * P_pred;

    // u = -(~K * x_hat)(0,0);
    // Torque Limit
    if (u > TORQUE_LIMIT) u = TORQUE_LIMIT;
    if (u < -TORQUE_LIMIT) u = -TORQUE_LIMIT;

    return u;
}

float Controller::lowPassFilter(float new_value, float prev_filtered_value, float alpha = 0.7f) {
    return alpha * prev_filtered_value + (1.0f - alpha) * new_value;
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

Vectorx4 Controller::f_func(const Vectorx4& x, float tau) {
    float pos = x(0,0);
    float vel = x(1,0);
    float theta = x(2,0);
    float dtheta = x(3,0);

    float cos_theta = std::cos(theta);
    float sin_theta = std::sin(theta);
    float cos_theta_squared = cos_theta * cos_theta;

    float denominator = 5.082572504e-5f - 3.90615696e-6f * cos_theta_squared;

    // xdd (linear acceleration)
    float xdd = (
        -0.000118584f * tau * cos_theta
        - 0.000669876f * tau
        - 3.83193997776e-5f * sin_theta * cos_theta
        + 1.3239429264e-6f * sin_theta * dtheta * dtheta
    ) / denominator;

    // thetadd (angular acceleration)
    float thetadd = (
        0.0019764f * tau * cos_theta
        + 0.0045524f * tau
        - 3.90615696e-6f * sin_theta * cos_theta * dtheta * dtheta
        + 0.00147106890936f * sin_theta
    ) / denominator;

    return Vectorx4{ vel, xdd, dtheta, thetadd };
}

Matrix4x4 Controller::A_func(const Vectorx4& x, float tau) {
    float pos = x(0,0);
    float vel = x(1,0);
    float theta = x(2,0);
    float dtheta = x(3,0);

    float cos_theta = std::cos(theta);
    float sin_theta = std::sin(theta);
    float cos_theta_sq = cos_theta * cos_theta;

    float denom1 = std::pow(1.0f - 0.0768539348317381f * cos_theta_sq, 2);
    float denom2 = 5.082572504e-5f - 3.90615696e-6f * cos_theta_sq;

    // Prepare matrix (initialize to zero)
    Matrix4x4 A = Zeros<4,4>();

    A(0,1) = 1.0f;

    A(1,2) =
        0.358623393996421f * tau * sin_theta * cos_theta_sq / denom1 +
        2.02584838322831f * tau * sin_theta * cos_theta / denom1 +
        0.000118584f * tau * sin_theta / denom2 +
        0.115886065608755f * sin_theta * sin_theta * cos_theta_sq / denom1 -
        0.00400388674461244f * sin_theta * sin_theta * cos_theta * dtheta * dtheta / denom1 +
        3.83193997776e-5f * sin_theta * sin_theta / denom2 -
        3.83193997776e-5f * cos_theta_sq / denom2 +
        1.3239429264e-6f * cos_theta * dtheta * dtheta / denom2;

    A(1,3) = 2.6478858528e-6f * sin_theta * dtheta / denom2;

    A(2,3) = 1.0f;

    A(3,2) =
        -5.97705656660701f * tau * sin_theta * cos_theta_sq / denom1 -
        13.7674318527736f * tau * sin_theta * cos_theta / denom1 -
        0.0019764f * tau * sin_theta / denom2 +
        0.0118130545982421f * sin_theta * sin_theta * cos_theta_sq * dtheta * dtheta / denom1 -
        4.44882720330986f * sin_theta * sin_theta * cos_theta / denom1 +
        3.90615696e-6f * sin_theta * sin_theta * dtheta * dtheta / denom2 -
        3.90615696e-6f * cos_theta_sq * dtheta * dtheta / denom2 +
        0.00147106890936f * cos_theta / denom2;

    A(3,3) = -7.81231392e-6f * sin_theta * cos_theta * dtheta / denom2;

    return A;
}

Matrix4x4 Controller::W_func(const Vectorx4& x, float tau) {
    // Initialize W as zero matrix
    Matrix4x4 W = Zeros<4,4>();

    // Set diagonal elements for process noise mapping
    W(1,1) = 1.0f; // velocity noise from acceleration
    W(3,3) = 1.0f; // angular velocity noise from angular acceleration

    return W;
    // return makeIdentity();
}

Vectorx4 Controller::h_func(const Vectorx4& x) {
    return x;
}

Matrix4x4 Controller::H_func(const Vectorx4& x) {
    return makeIdentity();
}

Matrix4x4 Controller::V_func(const Vectorx4& x) {
    return makeIdentity();
}


    