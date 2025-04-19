import numpy as np
import sympy as sp
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from matplotlib.animation import FuncAnimation
from IPython.display import HTML

def low_pass_filter(new_value, prev_filtered_value, alpha=0.9):
    """
    Simple first-order low-pass filter.

    Parameters:
        new_value: current raw signal value
        prev_filtered_value: previous filtered value
        alpha: smoothing factor (between 0 and 1)
               higher alpha = smoother, but more lag

    Returns:
        filtered_value: the new filtered output
    """
    filtered_value = alpha * prev_filtered_value + (1 - alpha) * new_value
    return filtered_value

def apply_smooth_boost(u, deadzone_threshold=0.2, max_boost=0.05):
    if 0 < u < deadzone_threshold:
        scale = u / deadzone_threshold
        return u + (1.0 - scale) * max_boost
    elif 0 > u > -deadzone_threshold:
        scale = -u / deadzone_threshold
        return u - (1.0 - scale) * max_boost
    else:
        return u

def smooth_deadzone_compensation(u, deadzone_threshold=0.2, max_boost=0.05):
    # Use smooth tanh-based boost
    boost = max_boost * np.tanh(u / deadzone_threshold)
    return u + boost

def animate_inverted_pendulum(x_pos, theta, time, Len=0.1, r=0.06, margin=0.2, interval=30):
    fig, ax = plt.subplots(figsize=(4, 3)) 
    ax.set_aspect('equal')

    ax.set_xlim(-0.5, 0.5)
    ax.set_ylim(-0.3, 0.3)

    rod_line, = ax.plot([], [], 'b-', lw=2)
    wheel_dot, = ax.plot([], [], 'ko', ms=8)
    time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)

    def init():
        rod_line.set_data([], [])
        wheel_dot.set_data([], [])
        time_text.set_text('')
        return rod_line, wheel_dot, time_text

    def update(i):
        x = x_pos[i]
        angle = theta[i]
        x0, y0 = x, r
        x1 = x + Len * np.sin(angle)
        y1 = y0 + Len * np.cos(angle)

        rod_line.set_data([x0, x1], [y0, y1])
        wheel_dot.set_data([x], [r])
        time_text.set_text(f"t = {time[i]:.2f}s")
        return rod_line, wheel_dot, time_text

    skip = max(1, len(time) // 300)  # Limit to ~300 frames
    ani = FuncAnimation(fig, update, frames=range(0, len(time), skip), init_func=init, interval=interval, blit=True)
    return HTML(ani.to_jshtml())

def lqg_simulation_linear_continuous(x0, sys_c, K, L, t_span, t_eval,
                                     measurement_noise_std = None,
                                     disturbance_std = None,
                                     t_impulse = None,
                                     impulse_magnitude = None):
    """
    Simulate continuous-time LQG closed-loop system
    - Measurement noise: optional
    - Disturbance: hardcoded inside function
    - Impulse input: optional

    Parameters:
        x0: initial state (shape: [4])
        sys_c: continuous-time state-space system
        K: state feedback gain (shape: [1, 4])
        L: observer gain (shape: [4, 4])
        t_span: tuple (t_start, t_end)
        t_eval: time points to evaluate
        measurement_noise_std: standard deviation of measurement noise
        impulse_time: time of impulse (float, optional)
        impulse_magnitude: magnitude of impulse (float, optional)

    Returns:
        t: time vector
        x_true_traj: true state trajectory
        x_hat_traj: estimated state trajectory
        tau_traj: control input trajectory
    """
    A = sys_c.A
    B = sys_c.B
    C = sys_c.C

    tau_storage = []

    dt = t_eval[1] - t_eval[0]
    
    # === Measurement (optionally with noise) ===
    # encoder error in meters
    # IMU-integrated linear velocity (m/s)
    # integrated angle error (rad)
    # IMU gyro noise (rad/s)
    if measurement_noise_std is None:
        measurement_noise_std = np.array([0.00, 0.00, 0.0, 0.0]) * np.sqrt(dt)
    else:
        measurement_noise_std = measurement_noise_std * np.sqrt(dt)

    # === Disturbance (optionally with noise) ===
    # Position disturbance in meters
    # Velocity disturbance in m/s
    # Angle disturbance in radians
    # Angular velocity disturbance in rad/s
    if disturbance_std is None:
        disturbance_std = np.array([0.00, 0.00, 0.00, 0.00]) * np.sqrt(dt)
    else:
        disturbance_std = disturbance_std * np.sqrt(dt)

    # === Impulse ===
    # Time when impulse is applied
    if t_impulse is None:
        t_impulse = -1.0  # time that will never be hit

    # Impulse magnitude: array of size (4,)
    if impulse_magnitude is None:
        impulse_magnitude = np.zeros_like(x0)


    def closed_loop_ode(t, z):
        x_true = z[:4]
        x_hat = z[4:]

        # === Control law (use estimated state) ===
        u = float(-K @ x_hat)  # safer as scalar

        # === Store control input ===
        tau_storage.append(u)

        #   Measurement noise 
        measurement_noise = measurement_noise_std * np.random.randn(C.shape[0])
        y_meas = C @ x_true + measurement_noise

        # === Estimator dynamics ===
        dx_hat = A @ x_hat + B.flatten() * u + L @ (y_meas - C @ x_hat)

        # Disturbances noise
        disturbance_noise = disturbance_std*np.random.randn(A.shape[0])

        # === Impulse ===
        epsilon = 1e-3  # duration of impulse approximation
        if abs(t - t_impulse) < epsilon:
            impulse = impulse_magnitude / epsilon
        else:
            impulse = np.zeros_like(x_true)

        # === True system dynamics (apply control to real system) ===
        dx_true = A @ x_true + B.flatten() * u + disturbance_noise+impulse

        # === Pack derivative ===
        dz = np.concatenate((dx_true, dx_hat))
        return dz

    z0 = np.concatenate((x0, x0))
    sol = solve_ivp(closed_loop_ode, t_span, z0, t_eval=t_eval)

    x_true_traj = sol.y[:4, :]
    x_hat_traj = sol.y[4:, :]
    tau_traj = np.array(tau_storage)

    return sol.t, x_true_traj, x_hat_traj, tau_traj

def lqg_simulation_linear_discrete(x0, sys_d, K, L, t_eval,
                                   measurement_noise_std = None,
                                   disturbance_std = None,
                                   t_impulse = None,
                                   impulse_magnitude = None,
                                   low_pass_enabled = False,
                                   debug_enabled = False,
                                   ):
    """
    Simulate discrete-time LQG closed-loop system

    Parameters:
        x0: initial state (shape: [4])
        sys_d: discrete-time state-space system
        K: state feedback gain (shape: [1, 4])
        L: observer gain (shape: [4, 4])
        t_eval: time points to evaluate (numpy array)
        measurement_noise_std: standard deviation of measurement noise
        disturbance_func: function of time f(t) -> disturbance input (optional)
        impulse_time: time of impulse (float, optional)
        impulse_magnitude: magnitude of impulse (float, optional)

    Returns:
        t_eval: time vector
        x_true_traj: true state trajectory (shape: [4, N])
        x_hat_traj: estimated state trajectory (shape: [4, N])
        tau_traj: control input trajectory (shape: [N])
    """
    A_d = sys_d.A
    B_d = sys_d.B
    C_d = sys_d.C
    dt = sys_d.dt

    N = len(t_eval)

    # Initialize storage
    x_true_traj = np.zeros((4, N))
    x_hat_traj = np.zeros((4, N))
    tau_traj = np.zeros(N)

    # Initial conditions
    x_true = x0.copy()
    x_hat = x0.copy()

    # Store initial state
    x_true_traj[:, 0] = x_true
    x_hat_traj[:, 0] = x_hat

    # === Measurement (optionally with noise) ===
    # encoder error in meters
    # IMU-integrated linear velocity (m/s)
    # integrated angle error (rad)
    # IMU gyro noise (rad/s)
    if measurement_noise_std is None:
        measurement_noise_std = np.array([0.00, 0.00, 0.0, 0.0]) * np.sqrt(dt)
    else:
        measurement_noise_std = measurement_noise_std * np.sqrt(dt)

    # === Disturbance (optionally with noise) ===
    # Position disturbance in meters
    # Velocity disturbance in m/s
    # Angle disturbance in radians
    # Angular velocity disturbance in rad/s
    if disturbance_std is None:
        disturbance_std = np.array([0.00, 0.00, 0.00, 0.00]) * np.sqrt(dt)
    else:
        disturbance_std = disturbance_std * np.sqrt(dt)

    # === Impulse ===
    # Time when impulse is applied
    if t_impulse is None:
        t_impulse = -1.0  # time that will never be hit

    # Impulse magnitude: array of size (4,)
    if impulse_magnitude is None:
        impulse_magnitude = np.zeros_like(x0)

    #Filter intialization
    u = float(-K @ x0)  # initial estimate-based control
    u_filtered = u  # initialize filter to meaningful value

    # Simulation loop
    for k in range(N - 1):
        # Control based on current estimate
        u = float(-K @ x_hat)
        if low_pass_enabled:
            u_filtered = low_pass_filter(u, u_filtered)  # update filtered value
            u = u_filtered
        else:
            u = u
        u = smooth_deadzone_compensation(u)
        tau_traj[k] = u

        # Observer: predict and correct with ( Measurement noise )
        measurement_noise = measurement_noise_std * np.random.randn(C_d.shape[0])
        if(debug_enabled):
            y_meas = np.array([0.0, 0.0, 0.0, 0.0])
        else:
            y_meas = C_d @ x_true + measurement_noise
        x_hat_pred = A_d @ x_hat + B_d.flatten() * u
        x_hat = x_hat_pred + L @ (y_meas - C_d @ x_hat_pred)

        # True state update with (disturbances + impulse )
        epsilon = 1e-6  # duration of impulse approximation
        if abs(dt*k - t_impulse) < epsilon:
            impulse = impulse_magnitude
        else:
            impulse = np.zeros_like(x_true)
        disturbance_noise = disturbance_std*np.random.randn(A_d.shape[0])

        if(debug_enabled):
            x_true = np.array([0.0, 0.0, 0.0, 0.0])
        else:
            x_true = A_d @ x_true + B_d.flatten() * u + disturbance_noise + impulse

        # Store trajectories
        x_true_traj[:, k + 1] = x_true
        x_hat_traj[:, k + 1] = x_hat

    return t_eval, x_true_traj, x_hat_traj, tau_traj

def f_func(x, tau):
    pos, vel, theta, dtheta = x
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    cos_theta_squared = cos_theta ** 2

    denominator = 5.082572504e-5 - 3.90615696e-6 * cos_theta_squared

    # xdd (linear acceleration)
    term1_xdd = -0.000118584 * tau * cos_theta / denominator
    term2_xdd = -0.000669876 * tau / denominator
    term3_xdd = -3.83193997776e-5 * sin_theta * cos_theta / denominator
    term4_xdd = 1.3239429264e-6 * sin_theta * dtheta**2 / denominator
    xdd = term1_xdd + term2_xdd + term3_xdd + term4_xdd
    # print(f"xdd is {xdd}")

    # thetadd (angular acceleration)
    term1_thetadd = 0.0019764 * tau * cos_theta / denominator
    term2_thetadd = 0.0045524 * tau / denominator
    term3_thetadd = -3.90615696e-6 * sin_theta * cos_theta * dtheta**2 / denominator
    term4_thetadd = 0.00147106890936 * sin_theta / denominator
    thetadd = term1_thetadd + term2_thetadd + term3_thetadd + term4_thetadd
    # print(f"thetadd is {thetadd}")

    # Return state derivative: [dx, ddx, dtheta, ddtheta]
    return np.array([vel, xdd, dtheta, thetadd])

def A_func(x,tau):  #Continuous Jacobian function, need to discretize in order to use in ekf
    pos, vel, theta, dtheta = x
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    cos_theta_sq = cos_theta ** 2

    denom1 = (1 - 0.0768539348317381 * cos_theta_sq) ** 2
    denom2 = (5.082572504e-5 - 3.90615696e-6 * cos_theta_sq)

    A_num = np.array([
        [0, 1, 0, 0],
        [0, 0,
         0.358623393996421 * tau * sin_theta * cos_theta_sq / denom1 +
         2.02584838322831 * tau * sin_theta * cos_theta / denom1 +
         0.000118584 * tau * sin_theta / denom2 +
         0.115886065608755 * sin_theta ** 2 * cos_theta_sq / denom1 -
         0.00400388674461244 * sin_theta ** 2 * cos_theta * dtheta ** 2 / denom1 +
         3.83193997776e-5 * sin_theta ** 2 / denom2 -
         3.83193997776e-5 * cos_theta_sq / denom2 +
         1.3239429264e-6 * cos_theta * dtheta ** 2 / denom2,
         2.6478858528e-6 * sin_theta * dtheta / denom2
        ],
        [0, 0, 0, 1],
        [0, 0,
         -5.97705656660701 * tau * sin_theta * cos_theta_sq / denom1 -
         13.7674318527736 * tau * sin_theta * cos_theta / denom1 -
         0.0019764 * tau * sin_theta / denom2 +
         0.0118130545982421 * sin_theta ** 2 * cos_theta_sq * dtheta ** 2 / denom1 -
         4.44882720330986 * sin_theta ** 2 * cos_theta / denom1 +
         3.90615696e-6 * sin_theta ** 2 * dtheta ** 2 / denom2 -
         3.90615696e-6 * cos_theta_sq * dtheta ** 2 / denom2 +
         0.00147106890936 * cos_theta / denom2,
         -7.81231392e-6 * sin_theta * cos_theta * dtheta / denom2
        ]
    ])

    return A_num

def W_func(x, tau):
    W = np.array([
        [0.0, 0.0, 0.0, 0.0],  # position: no direct noise
        [0.0, 1.0, 0.0, 0.0],  # velocity: noise from acceleration
        [0.0, 0.0, 0.0, 0.0],  # angle: no direct noise
        [0.0, 0.0, 0.0, 1.0]   # angular velocity: noise from angular acceleration
    ])
    return W

def h_func(x):
    return x

def H_func(x):
    return np.eye(len(x))

def V_func(x):
    return np.eye(len(x))

def ekf_estimation(x_prev, P_prev, u_prev, Q, dt, f_func, A_func, W_func):
    """
    EKF Estimation (Prediction) step

    Parameters:
        x_prev : ndarray
            Previous state estimate (n,)
        P_prev : ndarray
            Previous covariance estimate (n, n)
        u_prev : ndarray
            Previous control input (m,)
        Q : ndarray
            Process noise covariance (l, l)
        dt : float
            Time step for discretization
        f_func : function
            Continuous-time system dynamics: f(x, u) -> dx/dt
        A_func : function
            Jacobian of f w.r.t. x (continuous-time)
        W_func : function
            Jacobian of f w.r.t. process noise (continuous-time)

    Returns:
        x_pred : ndarray
            Predicted state estimate (n,)
        P_pred : ndarray
            Predicted covariance estimate (n, n)
    """
    # 1. State prediction using Euler integration
    dx_pred = f_func(x_prev, u_prev)  # Continuous-time derivative
    x_pred = x_prev + dt * dx_pred    # Discrete-time state prediction

    # 2. Jacobians
    A_cont = A_func(x_prev, u_prev)   # Continuous-time Jacobian df/dx
    A_k = np.eye(len(x_prev)) + dt * A_cont  # Discrete-time Jacobian
    W_k = W_func(x_prev, u_prev)      # Noise mapping matrix

    # 3. Covariance prediction
    P_pred = A_k @ P_prev @ A_k.T + (dt**2) * W_k @ Q @ W_k.T

    return x_pred, P_pred

def ekf_correction(x_pred, P_prev, z_k, R, h_func, H_func, V_func):
    """
    EKF Correction (Measurement update) step

    Parameters:
        x_pred : ndarray
            Predicted state estimate (n,)
        P_pred : ndarray
            Predicted covariance estimate (n, n)
        z_k : ndarray
            Current measurement (m,)
        R : ndarray
            Measurement noise covariance (p, p)
        h_func : function
            Measurement function: h(x, v=0)
        H_func : function
            Jacobian of h w.r.t. x at (x_pred)
        V_func : function
            Jacobian of h w.r.t. v at (x_pred)

    Returns:
        x_upd : ndarray
            Updated state estimate (n,)
        P_upd : ndarray
            Updated covariance estimate (n, n)
        K_k : ndarray
            Kalman gain (n, m)
    """
    # 1. Jacobians
    v_zero = np.zeros_like(z_k)  #Assumed
    H_k = H_func(x_pred)
    V_k = V_func(x_pred)
   
    # 2. Kalman Gain
    S_k = H_k @ P_prev @ H_k.T + V_k @ R @ V_k.T
    K_k = P_prev @ H_k.T @ np.linalg.inv(S_k)
    print(K_k)

    # 3. State update
    y_k = z_k - h_func(x_pred)  # measurement residual
    x_upd = x_pred + K_k @ y_k

    # 4. Covariance update
    I = np.eye(P_prev.shape[0])
    P_upd = (I - K_k @ H_k) @ P_prev

    return x_upd, P_upd, K_k

def ekf_simulation_nonlinear_discrete(x0, P0, K,t_eval, Q, R,
                                      measurement_noise_std=None,
                                      disturbance_std=None,
                                      t_impulse=None,
                                      impulse_magnitude=None,
                                      low_pass_enabled=False,
                                      debug_enabled = False,
                                      f_func = f_func, A_func=A_func, W_func=W_func,
                                      h_func=h_func, H_func=H_func, V_func=V_func):
    """
    Simulate a discrete-time nonlinear Extended Kalman Filter (EKF) closed-loop system.

    This function simulates the closed-loop dynamics of a nonlinear system controlled
    by state feedback (LQR), with state estimation performed by an EKF.
  
    Note: In this simulation, I assume the observor are full state and linear.

    Parameters
    ----------
    x0 : Initial state of the true system.
    P0 : Initial estimate of the state covariance.
    K : State feedback gain matrix.
    t_eval :Discrete time points for the simulation.
    Q :Process noise covariance matrix.
    R :Measurement noise covariance matrix.
    f_func : function
        Nonlinear system dynamics function:
        f(x, u) -> dx (discrete step)
    A_func : function
        Jacobian of system dynamics with respect to state:
        A(x, u) -> ndarray of shape (n_states, n_states)
    W_func : function
        Jacobian of system dynamics with respect to process noise:
        W(x, u) -> ndarray of shape (n_states, n_process_noise)
    h_func : function
        Nonlinear measurement function (assumed full-state observer):
        h(x) -> measurement vector
    H_func : function
        Jacobian of measurement function with respect to state:
        H(x) -> ndarray of shape (n_measurements, n_states)
    V_func : function
        Jacobian of measurement function with respect to measurement noise:
        V(x) -> ndarray of shape (n_measurements, n_measurement_noise)
   
    Returns
    -------
    t_eval : ndarray of shape (N,)
        Time vector.

    x_true_traj : ndarray of shape (n_states, N)
        True state trajectory of the system.

    x_hat_traj : ndarray of shape (n_states, N)
        EKF estimated state trajectory.

    tau_traj : ndarray of shape (n_inputs, N)
        Control input (e.g., torque) trajectory.

    Notes
    -----
    - This simulation uses the discrete-time form of EKF.
    - Use `ekf_estimation` for the time-update (prediction) step.
    - Use `ekf_correction` for the measurement-update (correction) step.
    - System must be provided in discrete-time form, or properly discretized.
    - Measurement and process noises are optional, and default to zero noise.
    - Impulse disturbance is optional for simulating external disturbances.
    """
    n_states = x0.shape[0]
    N = len(t_eval)
    dt = t_eval[1] - t_eval[0]

    # Initialize storage
    x_true_traj = np.zeros((n_states, N))
    x_hat_traj = np.zeros((n_states, N))
    P_traj = np.zeros((n_states, n_states, N))
    tau_traj = np.zeros(N)

    # Initial conditions
    x_true = x0.copy()
    x_hat = x0.copy()
    P = P0.copy()

     # Measurement noise
    if measurement_noise_std is None:
        measurement_noise_std = np.array([0.00, 0.00, 0.0, 0.0]) * np.sqrt(dt)
    else:
        measurement_noise_std = measurement_noise_std * np.sqrt(dt)

    # Process disturbance
    if disturbance_std is None:
        disturbance_std = np.array([0.00, 0.00, 0.00, 0.00]) * np.sqrt(dt)
    else:
        disturbance_std = disturbance_std * np.sqrt(dt)

    # Impulse disturbance
    if t_impulse is None:
        t_impulse = -1.0  # time that will never be hit

    # Impulse magnitude: array of size (4,)
    if impulse_magnitude is None:
        impulse_magnitude = np.zeros_like(x0)

    u = float(-K @ x0)  # initial estimate-based control
    u_filtered = u  # initialize filter to meaningful value
    print(f"K:{K}")
    print(f"K:{x0}")
    print(f"u_filtered:{u_filtered}")

    for k in range(N-1):
        # Control based on current estimate
        u = float(-K @ x_hat)
        if low_pass_enabled:
            u_filtered = low_pass_filter(u, u_filtered)  # update filtered value
            u = u_filtered
        # u = smooth_deadzone_compensation(u)
        tau_traj[k] = u
        
        # ===== EKF Estimation =====
        x_pred, P_prev = ekf_estimation(x_hat, P, u, Q, dt, f_func, A_func, W_func)
    
        if(debug_enabled):
            z_k = np.array([0.0, 0.0, 0.0, 0.0])
        else:
        # Measurement noise
            measurement_noise = measurement_noise_std * np.random.randn(x_true_traj.shape[0])
            z_k = h_func(x_true) + measurement_noise

        # ===== EKF Correction =====
        x_hat, P, L = ekf_correction(x_pred, P_prev, z_k, R, h_func, H_func, V_func)

        if(debug_enabled):
            x_true = np.array([0.0, 0.0, 0.0, 0.0])
        else:
            # True state update with (disturbances + impulse )
            epsilon = 1e-6  # duration of impulse approximation
            if abs(dt*k - t_impulse) < epsilon:
                # print(f"Impulse Applied, Impulse is {impulse}")
                impulse = impulse_magnitude
            else:
                impulse = np.zeros_like(x_true)

            disturbance_noise = disturbance_std*np.random.randn(x_true_traj.shape[0])

            dx_true = f_func(x_true, u) 
            x_true = x_true + dt * dx_true + disturbance_noise+impulse

        x_true_traj[:, k+1] = x_true
        x_hat_traj[:, k+1] = x_hat
        P_traj[:, :, k+1] = P

    return t_eval, x_true_traj, x_hat_traj, tau_traj

def ekf_simulation_nonlinear_discrete_v1(x0, P0, K,t_eval, Q, R,
                                      measurement_noise_std=None,
                                      disturbance_std=None,
                                      t_impulse=None,
                                      impulse_magnitude=None,
                                      low_pass_enabled=False,
                                      debug_enabled = False,
                                      f_func = f_func, A_func=A_func, W_func=W_func,
                                      h_func=h_func, H_func=H_func, V_func=V_func):
    """
    Simulate a discrete-time nonlinear Extended Kalman Filter (EKF) closed-loop system.

    This function simulates the closed-loop dynamics of a nonlinear system controlled
    by state feedback (LQR), with state estimation performed by an EKF.
  
    Note: In this simulation, I assume the observor are full state and linear.

    Parameters
    ----------
    x0 : Initial state of the true system.
    P0 : Initial estimate of the state covariance.
    K : State feedback gain matrix.
    t_eval :Discrete time points for the simulation.
    Q :Process noise covariance matrix.
    R :Measurement noise covariance matrix.
    f_func : function
        Nonlinear system dynamics function:
        f(x, u) -> dx (discrete step)
    A_func : function
        Jacobian of system dynamics with respect to state:
        A(x, u) -> ndarray of shape (n_states, n_states)
    W_func : function
        Jacobian of system dynamics with respect to process noise:
        W(x, u) -> ndarray of shape (n_states, n_process_noise)
    h_func : function
        Nonlinear measurement function (assumed full-state observer):
        h(x) -> measurement vector
    H_func : function
        Jacobian of measurement function with respect to state:
        H(x) -> ndarray of shape (n_measurements, n_states)
    V_func : function
        Jacobian of measurement function with respect to measurement noise:
        V(x) -> ndarray of shape (n_measurements, n_measurement_noise)
   
    Returns
    -------
    t_eval : ndarray of shape (N,)
        Time vector.

    x_true_traj : ndarray of shape (n_states, N)
        True state trajectory of the system.

    x_hat_traj : ndarray of shape (n_states, N)
        EKF estimated state trajectory.

    tau_traj : ndarray of shape (n_inputs, N)
        Control input (e.g., torque) trajectory.

    Notes
    -----
    - This simulation uses the discrete-time form of EKF.
    - Use `ekf_estimation` for the time-update (prediction) step.
    - Use `ekf_correction` for the measurement-update (correction) step.
    - System must be provided in discrete-time form, or properly discretized.
    - Measurement and process noises are optional, and default to zero noise.
    - Impulse disturbance is optional for simulating external disturbances.
    """
    n_states = x0.shape[0]
    N = len(t_eval)
    dt = t_eval[1] - t_eval[0]

    # Initialize storage
    x_true_traj = np.zeros((n_states, N))
    x_hat_traj = np.zeros((n_states, N))
    P_traj = np.zeros((n_states, n_states, N))
    tau_traj = np.zeros(N)

    # Initial conditions
    x_true = x0.copy()
    x_hat = x0.copy()
    P = P0.copy()

     # Measurement noise
    if measurement_noise_std is None:
        measurement_noise_std = np.array([0.00, 0.00, 0.0, 0.0]) * np.sqrt(dt)
    else:
        measurement_noise_std = measurement_noise_std * np.sqrt(dt)

    # Process disturbance
    if disturbance_std is None:
        disturbance_std = np.array([0.00, 0.00, 0.00, 0.00]) * np.sqrt(dt)
    else:
        disturbance_std = disturbance_std * np.sqrt(dt)

    # Impulse disturbance
    if t_impulse is None:
        t_impulse = -1.0  # time that will never be hit

    # Impulse magnitude: array of size (4,)
    if impulse_magnitude is None:
        impulse_magnitude = np.zeros_like(x0)

    u = float(-K @ x0)  # initial estimate-based control
    u_filtered = u  # initialize filter to meaningful value

    for k in range(N-1):
        # Control based on current estimate
        u = float(-K @ x_hat)
        if low_pass_enabled:
            u_filtered = low_pass_filter(u, u_filtered)  # update filtered value
            u = u_filtered
        # u = smooth_deadzone_compensation(u)
        tau_traj[k] = u
        # ===== EKF Estimation =====
        x_pred, P_prev = ekf_estimation(x_hat, P, u, Q, dt, f_func, A_func, W_func)
    
        # Measurement noise
        measurement_noise = measurement_noise_std * np.random.randn(x_true_traj.shape[0])
        z_k = h_func(x_true) + measurement_noise
        # ===== EKF Correction =====
        x_hat, P, L = ekf_correction(x_pred, P_prev, z_k, R, h_func, H_func, V_func)
        
        # True state update with (disturbances + impulse )
        if(debug_enabled):
            x_true = np.array([0.0, 0.0, 0.0, 0.0])

        else:
            epsilon = 1e-2  # duration of impulse approximation
            if abs(dt*k - t_impulse) < epsilon:
                # print(f"Impulse Applied, Impulse is {impulse}")
                impulse = impulse_magnitude
            else:
                impulse = np.zeros_like(x_true)
            disturbance_noise = disturbance_std*np.random.randn(x_true_traj.shape[0])

            dx_true = f_func(x_true, u) 
            x_true = x_true + dt * dx_true + disturbance_noise+impulse

        x_true_traj[:, k+1] = x_true
        x_hat_traj[:, k+1] = x_hat
        P_traj[:, :, k+1] = P

    return t_eval, x_true_traj, x_hat_traj, tau_traj