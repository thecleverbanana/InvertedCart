#include "control.hpp"

Controller::Controller(Motor* left, Motor* right, float dt): motorLeft(left), motorRight(right) {}

void Controller::initializeState(const float x_hat_init[4]) {
    // Convert input array to Vectorx4
    Vectorx4 x_init_vec;
    for (int i = 0; i < 4; ++i) {
        x_init_vec(i,0) = x_hat_init[i];
    }

    // Initialize state estimate
    this->x_hat = x_init_vec;

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

float Controller::updateLQR(float x_meas, float dx_meas, float theta_meas, float dtheta_meas){
    Vectorx4 x = { x_meas, dx_meas, theta_meas, dtheta_meas };

    float u = -(~K * x)(0,0);

    // Torque Limit
    if (u > TORQUE_LIMIT) u = TORQUE_LIMIT;
    if (u < -TORQUE_LIMIT) u = -TORQUE_LIMIT;

    Serial.print(">");
    Serial.printf("tau: %6.3f Nm", u);
    Serial.println();

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

    // --- Debug print ---
    Serial.print(">");
    Serial.print("x_hat:");     Serial.print(x_hat(0,0)); Serial.print(",");
    Serial.print("dx_hat:");    Serial.print(x_hat(1,0)); Serial.print(",");
    Serial.print("theta_hat:"); Serial.print(x_hat(2,0)); Serial.print(",");
    Serial.print("dtheta_hat:");Serial.print(x_hat(3,0)); Serial.print(",");
    Serial.print("tau:");       Serial.print(u);
    Serial.print("\r\n"); 


    return u;
}

float Controller::updateEKF_LQG(float x_meas, float dx_meas, float theta_meas, float dtheta_meas){
    // x_k_priori = x_k-1 + f_func * dt
    float u = -(~K * this->x_hat)(0,0); 
    Vectorx4 x_pred = this->x_hat+f_func(this->x_hat,u)*dt;
    Vectorx4 f_out = f_func(this->x_hat, u);
    // Serial.print(">");
    // Serial.printf("x_f_func: %7.4f m, ", f_out(0,0));          
    // Serial.printf("dx_f_func: %6.3f m/s, ", f_out(1,0));      
    // Serial.printf("theta_f_func: %7.4f rad, ", f_out(2,0));    
    // Serial.printf("dtheta_f_func: %6.3f rad/s, ", f_out(3,0)); 
    // Serial.println();  

    //P_k_priori = JacobianA @ P_k-1 @ Jacobian A.T + W @ Qn @ W.T
    Matrix4x4 I = makeIdentity();
    Matrix4x4 A_k = I+A_func(this->x_hat,u)*dt;
    Matrix4x4 W_k = W_func(this->x_hat, u);
    Matrix4x4 P_pred = A_k * this->P * (~A_k) + W_k * this->Q * (~W_k);
    // Serial.print("> P matrix: ");
    // Serial.printf("P00:%7.4f,", P_pred(0,0));
    // Serial.printf("P01:%7.4f,", P_pred(0,1));
    // Serial.printf("P02:%7.4f,", P_pred(0,2));
    // Serial.printf("P03:%7.4f,", P_pred(0,3));
    // Serial.printf("P10:%7.4f,", P_pred(1,0));
    // Serial.printf("P11:%7.4f,", P_pred(1,1));
    // Serial.printf("P12:%7.4f,", P_pred(1,2));
    // Serial.printf("P13:%7.4f,", P_pred(1,3));
    // Serial.printf("P20:%7.4f,", P_pred(2,0));
    // Serial.printf("P21:%7.4f,", P_pred(2,1));
    // Serial.printf("P22:%7.4f,", P_pred(2,2));
    // Serial.printf("P23:%7.4f,", P_pred(2,3));
    // Serial.printf("P30:%7.4f,", P_pred(3,0));
    // Serial.printf("P31:%7.4f,", P_pred(3,1));
    // Serial.printf("P32:%7.4f,", P_pred(3,2));
    // Serial.printf("P33:%7.4f",   P_pred(3,3));
    // Serial.println();


    //Kalman Gain
    Matrix4x4 V_k = V_func(x_pred);
    Matrix4x4 H_k = H_func(x_pred);
    Matrix4x4 S_k = H_k * P_pred * (~H_k) + V_k * R * (~V_k);
    Matrix4x4 S_inv = Invert(S_k);
    this->K_k = P_pred * (~H_k) * S_inv;

    //Correction based on new Kalman Gain
    // Current measurement as state
    Vectorx4 z_k = {x_meas, dx_meas, theta_meas, dtheta_meas};
    Serial.print(">");
    Serial.printf("x_z_k: %7.4f m, ", z_k(0,0));          // Numerical ∂h₀/∂xⱼ
    Serial.printf("dx_z_k: %6.3f m/s, ", z_k(1,0));       // Numerical ∂h₁/∂xⱼ
    Serial.printf("theta_z_k: %7.4f rad, ", z_k(2,0));    // Numerical ∂h₂/∂xⱼ
    Serial.printf("dtheta_z_k: %6.3f rad/s, ", z_k(3,0)); // Numerical ∂h₃/∂xⱼ
    Serial.println(); 
    Vectorx4 h_pred = h_func(x_pred);
    Vectorx4 y_k = z_k - h_pred;   
    Serial.print(">");
    Serial.printf("x_y_k: %7.4f m, ", y_k(0,0));          // Numerical ∂h₀/∂xⱼ
    Serial.printf("dx_y_k: %6.3f m/s, ", y_k(1,0));       // Numerical ∂h₁/∂xⱼ
    Serial.printf("theta_y_k: %7.4f rad, ", y_k(2,0));    // Numerical ∂h₂/∂xⱼ
    Serial.printf("dtheta_y_k: %6.3f rad/s, ", y_k(3,0)); // Numerical ∂h₃/∂xⱼ
    Serial.println();  
    this->x_hat = x_pred + (this->K_k * y_k);

    // Serial.print(">");
    // Serial.printf("x_hat: %7.4f m, ", x_hat(0,0));          // Numerical ∂h₀/∂xⱼ
    // Serial.printf("dx_hat: %6.3f m/s, ", x_hat(1,0));       // Numerical ∂h₁/∂xⱼ
    // Serial.printf("theta_hat: %7.4f rad, ", x_hat(2,0));    // Numerical ∂h₂/∂xⱼ
    // Serial.printf("dtheta_hat: %6.3f rad/s, ", x_hat(3,0)); // Numerical ∂h₃/∂xⱼ
    // Serial.println();

    u = -(~K * this->x_hat)(0,0);
    
    //Update covariance matrix
    this->P = (I - this->K_k * H_k) * P_pred;
    // Serial.print("> P matrix: ");
    // Serial.printf("P00:%7.4f,", this->P(0,0));
    // Serial.printf("P01:%7.4f,", this->P(0,1));
    // Serial.printf("P02:%7.4f,", this->P(0,2));
    // Serial.printf("P03:%7.4f,", this->P(0,3));
    // Serial.printf("P10:%7.4f,", this->P(1,0));
    // Serial.printf("P11:%7.4f,", this->P(1,1));
    // Serial.printf("P12:%7.4f,", this->P(1,2));
    // Serial.printf("P13:%7.4f,", this->P(1,3));
    // Serial.printf("P20:%7.4f,", this->P(2,0));
    // Serial.printf("P21:%7.4f,", this->P(2,1));
    // Serial.printf("P22:%7.4f,", this->P(2,2));
    // Serial.printf("P23:%7.4f,", this->P(2,3));
    // Serial.printf("P30:%7.4f,", this->P(3,0));
    // Serial.printf("P31:%7.4f,", this->P(3,1));
    // Serial.printf("P32:%7.4f,", this->P(3,2));
    // Serial.printf("P33:%7.4f",  this->P(3,3));
    // Serial.println();

    return u;
}

void Controller::debugFunc(Vectorx4& x_test, float u_test) {
    static int current_col = 0;
    const float eps = 1e-5f;

    // Step 1: Compute baseline h(x)
    Vectorx4 h_x = h_func(x_test);

    // Step 2: Perturb current state dimension
    Vectorx4 x_perturbed = x_test;
    x_perturbed(current_col, 0) += eps;

    Vectorx4 h_perturbed = h_func(x_perturbed);

    // Step 3: Estimate derivative numerically
    Vectorx4 diff = (h_perturbed - h_x) * (1.0f / eps);

    // Step 4: Compute analytical H_func()
    Matrix4x4 H_analytical = H_func(x_test);

    // Step 5: Strict serial print format (你的要求格式)
    Serial.print(">");
    Serial.printf("x_hat: %7.4f m, ", diff(0,0));          // Numerical ∂h₀/∂xⱼ
    Serial.printf("dx_hat: %6.3f m/s, ", diff(1,0));       // Numerical ∂h₁/∂xⱼ
    Serial.printf("theta_hat: %7.4f rad, ", diff(2,0));    // Numerical ∂h₂/∂xⱼ
    Serial.printf("dtheta_hat: %6.3f rad/s, ", diff(3,0)); // Numerical ∂h₃/∂xⱼ
    Serial.println();

    // Step 6: Advance to next column
    current_col = (current_col + 1) % 4;
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
        -13.7674318527736f * tau * sin_theta * cos_theta / denom1 -
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
    // // Initialize W as zero matrix
    // Matrix4x4 W = Zeros<4,4>();

    // // Set diagonal elements for process noise mapping
    // W(1,1) = 1.0f; // velocity noise from acceleration
    // W(3,3) = 1.0f; // angular velocity noise from angular acceleration

    // return W;
    return makeIdentity();
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

void Controller::ekf_estimation(Vectorx4& x_prev, Matrix4x4& P_prev, float u_prev, const Matrix4x4& Q, float dt, Vectorx4& x_pred, Matrix4x4& P_pred) {
    // 1. State prediction (Euler integration)
    Vectorx4 dx_pred = f_func(x_prev, u_prev);
    x_pred = x_prev + dx_pred * dt;

    // 2. Jacobians
    Matrix4x4 A_cont = A_func(x_prev, u_prev);
    Matrix4x4 A_k = makeIdentity() + A_cont * dt;  // Discrete-time Jacobian

    Matrix4x4 W_k = W_func(x_prev, u_prev);

    // 3. Covariance prediction
    Matrix4x4 temp = A_k * P_prev * (~A_k);
    Matrix4x4 noise = W_k * Q * (~W_k) * (dt * dt);

    P_pred = temp + noise;
}

void Controller::ekf_correction(Vectorx4& x_pred, Matrix4x4& P_pred, Vectorx4& z_k, const Matrix4x4& R) {
    Matrix4x4 H_k = H_func(x_pred);
    Matrix4x4 V_k = V_func(x_pred);

    // S_k = H * P * H^T + V * R * V^T
    Matrix4x4 S_k = H_k * P_pred * (~H_k) + V_k * R * (~V_k);

    Matrix4x4 S_inv = Invert(S_k);

    // K_k = P * H^T * S^-1
    this->K_k = P_pred * (~H_k) * S_inv;
    Serial.print(">");
    Serial.printf("K_k_1_1: %7.4f m, ", K_k(0,0));
    Serial.printf("K_k_2_2_: %6.3f m/s, ", K_k(1,1));
    Serial.printf("K_k_3_3_: %7.4f rad, ", K_k(2,2));
    Serial.printf("K_k_4_4_: %6.3f rad/s, ", K_k(3,3));
    Serial.println();

    // y_k = z_k - h(x_pred)
    Vectorx4 y_k = z_k - h_func(x_pred);
    Serial.print(">");
    Serial.printf("x_y_k: %7.4f m, ", y_k(0,0));
    Serial.printf("dx_y_k: %6.3f m/s, ", y_k(1,0));
    Serial.printf("theta_y_k: %7.4f rad, ", y_k(2,0));
    Serial.printf("dtheta_y_k: %6.3f rad/s, ", y_k(3,0));
    Serial.println();

    // x_hat = x_pred + K * y_k
    this->x_hat = x_pred + this->K_k * y_k;

    // P = (I - K * H) * P_pred
    Matrix4x4 I = makeIdentity();
    this->P = (I - this->K_k * H_k) * P_pred;
}
    
float Controller::updateLQGArchieve(float x_meas, float dx_meas, float theta_meas, float dtheta_meas) {
    // Measurement vector y
    Vectorx4 y = { x_meas, dx_meas, theta_meas, dtheta_meas };

    // --- 1. Compute control input: u = -K^T * x_hat
    float u = -(~K * x_hat)(0,0);

    // --- 2. Debug print ---
    Serial.print(">");
    Serial.printf("x_hat: %7.4f m, ", x_hat(0,0));
    Serial.printf("dx_hat: %6.3f m/s, ", x_hat(1,0));
    Serial.printf("theta_hat: %7.4f rad, ", x_hat(2,0));
    Serial.printf("dtheta_hat: %6.3f rad/s, ", x_hat(3,0));
    Serial.printf("tau: %6.3f Nm\n", u);

    // --- 3. Compute dx_hat: dx_hat = A * x_hat + B * u + L * (y - x_hat)
    Vectorx4 dx_hat = A * x_hat + B * u + L * (y - x_hat);

    // --- 4. Update state estimate: x_hat = x_hat + dx_hat * dt
    x_hat += dx_hat * dt;

    // --- 5. Recompute control input with updated state estimate
    u = -(~K * x_hat)(0,0);

    // --- 6. Apply smooth boost
    u = applySmoothBoost(u);

    return u;
}

    