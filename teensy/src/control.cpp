#include "control.hpp"
#include <cmath>

Controller::Controller(Motor* left, Motor* right, float dt): motorLeft(left), motorRight(right) {}

void Controller::initializeState(const float x_hat_init[4]) {
    for (int i = 0; i < 4; ++i) {
        this->x_hat[i] = x_hat_init[i];
    }

    for (int i = 0; i < 4; ++i) {
        u -= x_hat_init[i]*K[i];
    }

    u_filtered = u;
    Serial.print("Initialized x_hat and u_filtered: ");
    Serial.printf("x: %.4f, ", x_hat[0]);
    Serial.printf("dx: %.4f, ", x_hat[1]);
    Serial.printf("theta: %.4f, ", x_hat[2]);
    Serial.printf("dtheta: %.4f\n", x_hat[3]);
    Serial.printf("u_filtered: %.4f\n", u_filtered);
}

void Controller:: setDt(float new_dt){
    dt = new_dt;
}

float Controller::updateLQR(float x_meas, float dx_meas, float theta_meas, float dtheta_meas){
    float x[4] = {x_meas, dx_meas, theta_meas, dtheta_meas};

    float u = 0;
    for(int i = 0; i < 4; ++i){
        u -= K[i] * x[i];
    }
        // Torque Limit
        if (u > TORQUE_LIMIT) u = TORQUE_LIMIT;
        if (u < -TORQUE_LIMIT) u = -TORQUE_LIMIT;

        Serial.print(">");
        Serial.printf("tau: %6.3f Nm", u);
        Serial.println();

        return u;
}

float Controller::updateLQG(float x_meas, float dx_meas, float theta_meas, float dtheta_meas) {
    float y[4] = {x_meas, dx_meas, theta_meas, dtheta_meas};
    // === 1. u based on previous x estimation  ===
    float u = 0.0f;

    for(int i=0; i<4; i++){
        u -= K[i]*x_hat[i];
    }

    u_filtered = lowPassFilter(u,u_filtered);    //apply filter
    u = u_filtered;

    // Torque Limit
    if (u > TORQUE_LIMIT) u = TORQUE_LIMIT;
    if (u < -TORQUE_LIMIT) u = -TORQUE_LIMIT;

    // === 2. Observer update ===
    // Compute state estimate prediction: x_hat_pred = A * x_hat + B * u
    float x_hat_pred[4] = {0.0f};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            x_hat_pred[i] += A[i][j] * x_hat[j];
        }
        x_hat_pred[i] += B[i] * u;
    }

    // Measurement innovation: y - x_hat_pred (since C = Identity)
    float innovation[4];
    for (int i = 0; i < 4; i++) {
        innovation[i] = y[i] - x_hat_pred[i];
    }

    // Correct state estimate: x_hat_next = x_hat_pred + L * innovation
    float x_hat_next[4] = {0.0f};
    for (int i = 0; i < 4; i++) {
        x_hat_next[i] = x_hat_pred[i];
        for (int j = 0; j < 4; j++) {
            x_hat_next[i] += L[i][j] * innovation[j];
        }
    }

    // Update state estimate
    for (int i = 0; i < 4; i++) {
        x_hat[i] = x_hat_next[i];
    } 

    // --- Debug print ---
    // Serial.print(">");
    // Serial.printf("x_hat: %7.4f m", x_hat[0]); Serial.printf(",");
    // Serial.printf("dx_hat: %6.3f m/s", x_hat[1]);Serial.printf(",");
    // Serial.printf("theta_hat: %7.4f rad", x_hat[2]);Serial.printf(",");
    // Serial.printf("dtheta_hat: %6.3f rad/s", x_hat[3]); Serial.printf(",");
    // Serial.printf("tau: %6.3f Nm", u);
    // Serial.println();

    return u;
}

float Controller::updateEKF_LQG(float x_meas, float dx_meas, float theta_meas, float dtheta_meas){
     // Current measurement as state
     State z_k = {x_meas, dx_meas, theta_meas, dtheta_meas};

     // ===== EKF Estimation (Prediction Step) =====
     State x_pred;
     Matrix4x4 P_pred;
     ekf_estimation(x_hat, P, u_prev, Q, dt, f_func, A_func, W_func, x_pred, P_pred);
 
     // ===== EKF Correction (Update Step) =====
     ekf_correction(x_pred, P_pred, z_k, R, h_func, H_func, V_func);

    // Control input u = -K * x_hat
    float u_current = 0.0f;
    for (int i = 0; i < 4; ++i) {
        u_current -= K[i] * x_hat[i]; // K: your controller gain, assumed K[4]
    }
    
    u_prev = u_current;

    // --- Debug print ---
     Serial.print(">");
     Serial.printf("x_hat: %7.4f m", x_hat[0]); Serial.printf(",");
     Serial.printf("dx_hat: %6.3f m/s", x_hat[1]);Serial.printf(",");
     Serial.printf("theta_hat: %7.4f rad", x_hat[2]);Serial.printf(",");
     Serial.printf("dtheta_hat: %6.3f rad/s", x_hat[3]);Serial.printf(",");
     Serial.printf("P: %6.3f rad/s", P);
     Serial.println();

    // Return control input
     return u_current;
}

float Controller::lowPassFilter(float new_value, float prev_filtered_value, float alpha = 0.9f) {
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

State Controller::f_func(const State& x, float tau) {
    float pos = x[0];
    float vel = x[1];
    float theta = x[2];
    float dtheta = x[3];

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

    return {vel, xdd, dtheta, thetadd};
}

Matrix4x4 Controller::A_func(const State& x, float tau) {
    float pos = x[0];
    float vel = x[1];
    float theta = x[2];
    float dtheta = x[3];

    float cos_theta = std::cos(theta);
    float sin_theta = std::sin(theta);
    float cos_theta_sq = cos_theta * cos_theta;

    float denom1 = std::pow(1.0f - 0.0768539348317381f * cos_theta_sq, 2);
    float denom2 = 5.082572504e-5f - 3.90615696e-6f * cos_theta_sq;

    // Prepare matrix 
    Matrix4x4 A = { 0 };

    A[0][1] = 1.0f;

    A[1][2] =
        0.358623393996421f * tau * sin_theta * cos_theta_sq / denom1 +
        2.02584838322831f * tau * sin_theta * cos_theta / denom1 +
        0.000118584f * tau * sin_theta / denom2 +
        0.115886065608755f * sin_theta * sin_theta * cos_theta_sq / denom1 -
        0.00400388674461244f * sin_theta * sin_theta * cos_theta * dtheta * dtheta / denom1 +
        3.83193997776e-5f * sin_theta * sin_theta / denom2 -
        3.83193997776e-5f * cos_theta_sq / denom2 +
        1.3239429264e-6f * cos_theta * dtheta * dtheta / denom2;

    A[1][3] = 2.6478858528e-6f * sin_theta * dtheta / denom2;

    A[2][3] = 1.0f;

    A[3][2] =
        -5.97705656660701f * tau * sin_theta * cos_theta_sq / denom1 -
        13.7674318527736f * tau * sin_theta * cos_theta / denom1 -
        0.0019764f * tau * sin_theta / denom2 +
        0.0118130545982421f * sin_theta * sin_theta * cos_theta_sq * dtheta * dtheta / denom1 -
        4.44882720330986f * sin_theta * sin_theta * cos_theta / denom1 +
        3.90615696e-6f * sin_theta * sin_theta * dtheta * dtheta / denom2 -
        3.90615696e-6f * cos_theta_sq * dtheta * dtheta / denom2 +
        0.00147106890936f * cos_theta / denom2;

    A[3][3] = -7.81231392e-6f * sin_theta * cos_theta * dtheta / denom2;

    return A;
}

Matrix4x4 Controller::W_func(const State& x, float tau) {
    Matrix4x4 W = { 0.0f };

    W[1][1] = 1.0f; // velocity noise from acceleration
    W[3][3] = 1.0f; // angular velocity noise from angular acceleration

    return W;
}

State Controller::h_func(const State& x) {
    return x;
}

Matrix4x4 Controller::H_func(const State& x) {
    Matrix4x4 H = { 0.0f };

    for (int i = 0; i < 4; ++i) {
        H[i][i] = 1.0f;
    }

    return H;
}

Matrix4x4 Controller::V_func(const State& x) {
    Matrix4x4 V = { 0.0f };

    for (int i = 0; i < 4; ++i) {
        V[i][i] = 1.0f;
    }

    return V;
}

void Controller::ekf_estimation(
    const State& x_prev, const Matrix4x4& P_prev, float u_prev,
    const Matrix4x4& Q, float dt,
    State (*f_func)(const State&, float),
    Matrix4x4 (*A_func)(const State&, float),
    Matrix4x4 (*W_func)(const State&, float),
    State& x_pred, Matrix4x4& P_pred)
    {
        // 1.  (Euler integration)
        State dx_pred = f_func(x_prev, u_prev);
        for (int i = 0; i < 4; ++i)
            x_pred[i] = x_prev[i] + dt * dx_pred[i];

        // 2. Jacobians
        Matrix4x4 A_cont = A_func(x_prev, u_prev);
        Matrix4x4 A_k = mat_add(mat_identity(), mat_scalar_mul(A_cont, dt)); //Discrete-time Jacobian
        Matrix4x4 W_k = W_func(x_prev, u_prev);

        // 3. P covariance
        Matrix4x4 temp = mat_mul(A_k, mat_mul(P_prev, mat_transpose(A_k)));
        Matrix4x4 noise = mat_scalar_mul(mat_mul(W_k, mat_mul(Q, mat_transpose(W_k))), dt * dt);
        P_pred = mat_add(temp, noise);
    }

void Controller::ekf_correction(
    const State& x_pred, const Matrix4x4& P_pred, const State& z_k,
    const Matrix4x4& R,
    State (*h_func)(const State&),
    Matrix4x4 (*H_func)(const State&),
    Matrix4x4 (*V_func)(const State&))
{
    Matrix4x4 H_k = H_func(x_pred);
    Matrix4x4 V_k = V_func(x_pred);

    // S_k = H * P * H^T + V * R * V^T
    Matrix4x4 S_k = mat_add(mat_mul(H_k, mat_mul(P_pred, mat_transpose(H_k))),
                            mat_mul(V_k, mat_mul(R, mat_transpose(V_k))));

    Matrix4x4 S_inv = inverse_matrix(S_k); 

    // Kalman gain K_k = P * H^T * S^-1
    this->K_k = mat_mul(P_pred, mat_mul(mat_transpose(H_k), S_inv));

    // y_k = z_k - h(x_pred)
    State y_k;
    State h_x = h_func(x_pred);
    for (int i = 0; i < 4; ++i){
        y_k[i] = z_k[i] - h_x[i];
    }

    // x_upd = x_pred + K * y_k
    for (int i = 0; i < 4; ++i) {
        this->x_hat[i] = x_pred[i];
        for (int j = 0; j < 4; ++j){
            this->x_hat[i] += K_k[i][j] * y_k[j];
        }
    }

    // P_upd = (I - K * H) * P
    Matrix4x4 I = mat_identity();
    Matrix4x4 KH = mat_mul(this->K_k, H_k);
    Matrix4x4 temp = { 0.0f };

    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            temp[i][j] = I[i][j] - KH[i][j];
        }
    }

    this->P = mat_mul(temp, P_pred);
}


float Controller::updateLQGArchieve(float x_meas, float dx_meas, float theta_meas, float dtheta_meas) {
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
    