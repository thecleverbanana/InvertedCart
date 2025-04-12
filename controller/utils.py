import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from matplotlib.animation import FuncAnimation
from IPython.display import HTML

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

    ani = FuncAnimation(fig, update, frames=len(time), init_func=init, interval=interval, blit=True)
    return HTML(ani.to_jshtml())


def xdd_solution_numerical(pos, vel, theta, dtheta, tau):
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    cos_theta_squared = cos_theta * cos_theta

    denominator = 5.082572504e-5 - 3.90615696e-6 * cos_theta_squared

    term1 = -0.000118584 * tau * cos_theta / denominator
    term2 = -0.000669876 * tau / denominator
    term3 = -3.83193997776e-5 * sin_theta * cos_theta / denominator
    term4 = 1.3239429264e-6 * sin_theta * dtheta * dtheta / denominator

    return term1 + term2 + term3 + term4

def thetadd_solution_numerical(pos, vel, theta, dtheta, tau):
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    cos_theta_squared = cos_theta * cos_theta

    denominator = 5.082572504e-5 - 3.90615696e-6 * cos_theta_squared

    term1 = 0.0019764 * tau * cos_theta / denominator
    term2 = 0.0045524 * tau / denominator
    term3 = -3.90615696e-6 * sin_theta * cos_theta * dtheta * dtheta / denominator
    term4 = 0.00147106890936 * sin_theta / denominator

    return term1 + term2 + term3 + term4

def lqg_simulation_linear_continuous(x0, sys_c, K, L, t_span, t_eval):
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

    def closed_loop_ode(t, z):
        x_true = z[:4]
        x_hat = z[4:]

        # === Disturbance and Impulse (optional) ===
        disturbance = 0.0
        # impulse = 0.0
        # if impulse_time is not None:
        #     epsilon = 1e-3
        #     if abs(t - impulse_time) < epsilon:
        #         impulse = impulse_magnitude / epsilon
        # total_disturbance = disturbance + impulse

        # === Measurement (optionally with noise) ===
        x_std_d = 0.01      # encoder error in meters
        dx_std_d = 0.01       # IMU-integrated linear velocity (m/s)
        theta_std_d = 0.1    # integrated angle error (rad)
        dtheta_std_d = 0.1  # IMU gyro noise (rad/s)
        measurement_noise_std = np.array([x_std_d, dx_std_d, theta_std_d, dtheta_std_d])
        measurement_noise = measurement_noise_std * np.random.randn(C.shape[0])
        y_meas = C @ x_true + measurement_noise

        # === Control law (use estimated state) ===
        u = float(-K @ x_hat)  # safer as scalar

        # === Estimator dynamics ===
        dx_hat = A @ x_hat + B.flatten() * u + L @ (y_meas - C @ x_hat)

        # === True system dynamics (apply control to real system) ===
        dx_true = A @ x_true + B.flatten() * u # + total_disturbance

        # === Store control input ===
        tau_storage.append(u)

        # === Pack derivative ===
        dz = np.concatenate((dx_true, dx_hat))
        return dz

    z0 = np.concatenate((x0, x0))
    sol = solve_ivp(closed_loop_ode, t_span, z0, t_eval=t_eval)

    x_true_traj = sol.y[:4, :]
    x_hat_traj = sol.y[4:, :]
    tau_traj = np.array(tau_storage)

    return sol.t, x_true_traj, x_hat_traj, tau_traj



def lqg_simulation_linear_discrete(x0, sys_c, sys_d, K, L, t_eval):
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
    A_c = sys_c.A
    B_c = sys_c.B
    # C_c = sys_c.C

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

    u = float(-K @ x_hat)

    # Simulation loop
    for k in range(N - 1):
        # Control based on current estimate
        u = float(-K @ x_hat)
        tau_traj[k] = u

        # True state update
        x_true = A_d @ x_true + B_d.flatten() * u

        # Measurement
        x_std_d = 0.01      # encoder error in meters
        dx_std_d = 0.01       # IMU-integrated linear velocity (m/s)
        theta_std_d = 0.1    # integrated angle error (rad)
        dtheta_std_d = 0.01  # IMU gyro noise (rad/s)
        measurement_noise_std = np.array([x_std_d, dx_std_d, theta_std_d, dtheta_std_d])*np.sqrt(dt)
        measurement_noise = measurement_noise_std * np.random.randn(C_d.shape[0])
        y_meas = C_d @ x_true + measurement_noise

        # Observer: predict and correct
        x_hat_pred = A_d @ x_hat + B_d.flatten() * u
        x_hat = x_hat_pred + L @ (y_meas - C_d @ x_hat_pred)

        # Store trajectories
        x_true_traj[:, k + 1] = x_true
        x_hat_traj[:, k + 1] = x_hat

    return t_eval, x_true_traj, x_hat_traj, tau_traj
