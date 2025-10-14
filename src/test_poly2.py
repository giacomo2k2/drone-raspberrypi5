import numpy as np
import math
import cvxpy as cp
import matplotlib.pyplot as plt

# ------------------ Polynomial / QP helpers ------------------
def compute_Q_unit(order, min_order):
    n = order
    Q = np.zeros((n+1, n+1))
    for i in range(min_order, n+1):
        for j in range(min_order, n+1):
            Q[i, j] = (math.factorial(i)/math.factorial(i-min_order) *
                       math.factorial(j)/math.factorial(j-min_order) /
                       (i+j-2*min_order+1))
    return Q

def poly_derivative_row(order, deriv, tau):
    row = np.zeros(order+1)
    for i in range(deriv, order+1):
        row[i] = math.factorial(i)/math.factorial(i-deriv) * tau**(i-deriv)
    return row

def eval_poly(coeffs, tau, der_order=0):
    val = np.zeros_like(tau)
    n = len(coeffs)
    for i in range(der_order, n):
        val += coeffs[i] * math.factorial(i)/math.factorial(i-der_order) * tau**(i-der_order)
    return val

# ------------------ Trajectory QP ------------------
def solve_min_QP_axis(nodes, periods, order, continuity_order, min_derivative_order):
    N = len(periods)
    H = np.zeros((N*(order+1), N*(order+1)))
    A = []
    b = []

    Q_unit = compute_Q_unit(order, min_derivative_order)

    for i in range(N):
        scale = periods[i]**(1 - 2*min_derivative_order)
        H[i*(order+1):(i+1)*(order+1), i*(order+1):(i+1)*(order+1)] = Q_unit * scale

    # Start boundary
    for r in range(continuity_order):
        row = np.zeros(N*(order+1))
        row[:order+1] = poly_derivative_row(order, r, 0) / (periods[0]**r if r>0 else 1)
        b.append(nodes[0] if r == 0 else 0.0)
        A.append(row)

    # End boundary
    for r in range(continuity_order):
        row = np.zeros(N*(order+1))
        row[-(order+1):] = poly_derivative_row(order, r, 1) / (periods[-1]**r if r>0 else 1)
        b.append(nodes[-1] if r == 0 else 0.0)
        A.append(row)

    # Intermediate continuity
    for i in range(1, N):
        T_prev = periods[i-1]
        T_i = periods[i]
        for r in range(continuity_order):
            row = np.zeros(N*(order+1))
            row[(i-1)*(order+1):i*(order+1)] = poly_derivative_row(order, r, 1) / (T_prev**r if r>0 else 1)
            row[i*(order+1):(i+1)*(order+1)] = -poly_derivative_row(order, r, 0) / (T_i**r if r>0 else 1)
            b.append(0.0)
            A.append(row)

    # Internal positions
    for i in range(1, N):
        row = np.zeros(N*(order+1))
        row[(i-1)*(order+1):i*(order+1)] = poly_derivative_row(order, 0, 1)
        b.append(nodes[i])
        A.append(row)

    A = np.vstack(A)
    b = np.array(b)

    x = cp.Variable(N*(order+1))
    prob = cp.Problem(cp.Minimize(cp.quad_form(x, cp.psd_wrap(H))),
                      [A @ x == b])
    prob.solve(solver=cp.OSQP)

    if prob.status != cp.OPTIMAL:
        raise RuntimeError("QP failed")

    coeffs = np.array(x.value).reshape(N, order+1)
    return coeffs

# ------------------ Time / period helpers ------------------
def initial_periods(waypoints, vmax, amax, safety_factor=1.2, min_duration=0.05):
    distances = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)
    periods = np.zeros_like(distances, dtype=float)
    for i, d in enumerate(distances):
        T_v = d / vmax
        T_a = np.sqrt(d / amax)
        periods[i] = max(T_v, T_a) * safety_factor
        periods[i] = max(periods[i], min_duration)
    return periods / np.sum(periods)

def build_trajectory_normalized_3d(waypoints, vmax, amax, order=5, continuity_order=3, min_derivative_order=2):
    periods = initial_periods(waypoints, vmax, amax)
    coeffs_xyz = []
    for dim in range(waypoints.shape[1]):
        coeffs = solve_min_QP_axis(waypoints[:, dim], periods, order, continuity_order, min_derivative_order)
        coeffs_xyz.append(coeffs)
    return np.array(coeffs_xyz), periods


def uniform_rescale_periods_3d_norm(coeffs_x, coeffs_y, coeffs_z, periods, vmax, amax, sample_rate=100):
    periods = np.array(periods, dtype=float)
    N = len(periods)
    scale = 1.0

    for i in range(N):
        tau = np.linspace(0, 1, sample_rate)

        # normalized derivatives
        vel_x = eval_poly(coeffs_x[i], tau, 1)
        vel_y = eval_poly(coeffs_y[i], tau, 1)
        vel_z = eval_poly(coeffs_z[i], tau, 1)

        acc_x = eval_poly(coeffs_x[i], tau, 2)
        acc_y = eval_poly(coeffs_y[i], tau, 2)
        acc_z = eval_poly(coeffs_z[i], tau, 2)

        # physical derivatives
        vel_phys = np.column_stack([vel_x, vel_y, vel_z]) / periods[i]
        acc_phys = np.column_stack([acc_x, acc_y, acc_z]) / (periods[i]**2)

        # norm along trajectory
        vel_norm = np.linalg.norm(vel_phys, axis=1)
        acc_norm = np.linalg.norm(acc_phys, axis=1)

        # required scaling
        scale_v = np.max(vel_norm) / vmax if vmax > 0 else 0
        scale_a = np.sqrt(np.max(acc_norm) / amax) if amax > 0 else 0

        scale = max(scale, scale_v, scale_a, 1.0)

    periods *= scale
    return periods


# ------------------ Sampling ------------------
def sample_trajectory_3d(coeffs_xyz, periods, sample_rate=200):
    t = np.array([])
    pos = np.zeros((0,3))
    vel = np.zeros((0,3))
    acc = np.zeros((0,3))
    N = coeffs_xyz.shape[0]

    for i, T in enumerate(periods):
        ts = np.linspace(0, T, sample_rate)
        tau = ts / T

        p_seg = np.array([eval_poly(coeffs_xyz[dim, i], tau) for dim in range(3)]).T
        v_seg = np.array([eval_poly(coeffs_xyz[dim, i], tau, 1)/T for dim in range(3)]).T
        a_seg = np.array([eval_poly(coeffs_xyz[dim, i], tau, 2)/(T**2) for dim in range(3)]).T

        if t.size == 0:
            t = ts
            pos = p_seg
            vel = v_seg
            acc = a_seg
        else:
            t = np.concatenate((t, ts[1:] + t[-1]))
            pos = np.vstack((pos, p_seg[1:]))
            vel = np.vstack((vel, v_seg[1:]))
            acc = np.vstack((acc, a_seg[1:]))

    return t, pos, vel, acc

# ------------------ Example ------------------
if __name__ == '__main__':
    waypoints = np.array([
        [0,0,0],
        [0,0,10],
        [40,0, 10],
        [40,30,10],
        [0,0,10],
        [0,0,0],
    ])
    vmax = 2.0
    amax = 5.0
    order = 7
    continuity_order = 4
    min_derivative_order = 2

    coeffs_xyz, norm_periods = build_trajectory_normalized_3d(waypoints, vmax, amax, order, continuity_order, min_derivative_order)

     # Split coefficients per axis
    coeffs_x = coeffs_xyz[0, :, :]  # axis 0 = x, axis 1 = segments → shape (N_segments, order+1)
    coeffs_y = coeffs_xyz[1, :, :]  # y
    coeffs_z = coeffs_xyz[2, :, :]  # z



    periods = uniform_rescale_periods_3d_norm(
        coeffs_x, coeffs_y, coeffs_z,
        norm_periods, vmax, amax,
        
    )
    t, pos, vel, acc = sample_trajectory_3d(coeffs_xyz, periods)

    # ------------------ Plotting ------------------
    fig, axs = plt.subplots(3, 3, figsize=(15, 10), sharex=True)

    # Compute cumulative times at waypoints
    t_waypoints = np.zeros(len(periods)+1)
    t_waypoints[1:] = np.cumsum(periods)

    # Position
    axs[0,0].plot(t, pos[:,0], 'b-'); 
    axs[0,0].plot(t_waypoints, waypoints[:,0], 'ro')  # waypoints
    axs[0,0].set_ylabel("X [m]"); axs[0,0].grid(True)

    axs[0,1].plot(t, pos[:,1], 'g-'); 
    axs[0,1].plot(t_waypoints, waypoints[:,1], 'ro')
    axs[0,1].set_ylabel("Y [m]"); axs[0,1].grid(True)

    axs[0,2].plot(t, pos[:,2], 'r-'); 
    axs[0,2].plot(t_waypoints, waypoints[:,2], 'ro')
    axs[0,2].set_ylabel("Z [m]"); axs[0,2].grid(True)

    axs[0,0].set_title("Position vs Time"); 
    axs[0,1].set_title("Position vs Time"); 
    axs[0,2].set_title("Position vs Time")


    # Velocity
    axs[1,0].plot(t, vel[:,0], 'b-'); axs[1,0].set_ylabel("Vx [m/s]"); axs[1,0].grid(True)
    axs[1,1].plot(t, vel[:,1], 'g-'); axs[1,1].set_ylabel("Vy [m/s]"); axs[1,1].grid(True)
    axs[1,2].plot(t, vel[:,2], 'r-'); axs[1,2].set_ylabel("Vz [m/s]"); axs[1,2].grid(True)
    axs[1,0].set_title("Velocity vs Time"); axs[1,1].set_title("Velocity vs Time"); axs[1,2].set_title("Velocity vs Time")

    # Acceleration
    axs[2,0].plot(t, acc[:,0], 'b-'); axs[2,0].set_ylabel("Ax [m/s²]"); axs[2,0].set_xlabel("Time [s]"); axs[2,0].grid(True)
    axs[2,1].plot(t, acc[:,1], 'g-'); axs[2,1].set_ylabel("Ay [m/s²]"); axs[2,1].set_xlabel("Time [s]"); axs[2,1].grid(True)
    axs[2,2].plot(t, acc[:,2], 'r-'); axs[2,2].set_ylabel("Az [m/s²]"); axs[2,2].set_xlabel("Time [s]"); axs[2,2].grid(True)
    axs[2,0].set_title("Acceleration vs Time"); axs[2,1].set_title("Acceleration vs Time"); axs[2,2].set_title("Acceleration vs Time")

    plt.tight_layout()
    plt.show()


    # Compute norms
    vel_norm = np.linalg.norm(vel, axis=1)
    acc_norm = np.linalg.norm(acc, axis=1)

    # Plot norms
    plt.figure(figsize=(10,4))
    plt.subplot(2,1,1)
    plt.plot(t, vel_norm, 'b-')
    plt.axhline(vmax, color='r', linestyle='--', label='vmax')
    plt.ylabel("Velocity Norm [m/s]")
    plt.grid(True)
    plt.legend()
    plt.title("Velocity Norm vs Time")

    plt.subplot(2,1,2)
    plt.plot(t, acc_norm, 'g-')
    plt.axhline(amax, color='r', linestyle='--', label='amax')
    plt.ylabel("Acceleration Norm [m/s²]")
    plt.xlabel("Time [s]")
    plt.grid(True)
    plt.legend()
    plt.title("Acceleration Norm vs Time")

    plt.tight_layout()
    plt.show()
