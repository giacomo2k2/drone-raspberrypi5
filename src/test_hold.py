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



if __name__ == '__main__':
    # ------------------ Waypoints and Parameters ------------------
    waypoints = np.array([
        [0,0,0],
        [0,0,10],
        [40,0,10],
        [40,30,10],
        [0,0,10],
        [0,0,0],
    ])
    holding_indices = [0, 1, 3, 4,  5]       # indices of waypoints where holding may occur
    holding_times = [0.0, 2.0, 3.0, 1.0, 0.0]  # seconds to wait at each holding point

    vmax = 2.0
    amax = 5.0
    order = 7
    continuity_order = 4
    min_derivative_order = 2

    # ------------------ Generate Trajectory Segments ------------------
    trajectory_segments = []
    cumulative_time = 0.0

    for seg_idx in range(len(holding_indices)-1):
        start_idx = holding_indices[seg_idx]
        end_idx = holding_indices[seg_idx+1]

        segment_waypoints = waypoints[start_idx:end_idx+1]

        # Build segment trajectory
        coeffs_xyz, norm_periods = build_trajectory_normalized_3d(
            segment_waypoints, vmax, amax, order, continuity_order, min_derivative_order
        )
        coeffs_x = coeffs_xyz[0, :, :]
        coeffs_y = coeffs_xyz[1, :, :]
        coeffs_z = coeffs_xyz[2, :, :]

        periods = uniform_rescale_periods_3d_norm(
            coeffs_x, coeffs_y, coeffs_z, norm_periods, vmax, amax
        )

        # Sample trajectory
        t_seg, pos_seg, vel_seg, acc_seg = sample_trajectory_3d(coeffs_xyz, periods)
        t_seg += cumulative_time  # apply cumulative offset

        # Store trajectory segment
        trajectory_segments.append({
            't': t_seg,
            'pos': pos_seg,
            'vel': vel_seg,
            'acc': acc_seg,
            'waypoints': segment_waypoints,
            'hold_time': holding_times[seg_idx+1],
            'hold_at': t_seg[-1]  # time of holding at the last waypoint of the segment
        })

        # Update cumulative time (trajectory + hold at the last waypoint)
        cumulative_time = t_seg[-1] + holding_times[seg_idx+1]

    # ------------------ Concatenate all segments ------------------
    t_full = np.concatenate([seg['t'] for seg in trajectory_segments])
    pos_full = np.vstack([seg['pos'] for seg in trajectory_segments])
    vel_full = np.vstack([seg['vel'] for seg in trajectory_segments])
    acc_full = np.vstack([seg['acc'] for seg in trajectory_segments])

    
    # ------------------ Plot Trajectories with all waypoints ------------------
    fig, axs = plt.subplots(3, 3, figsize=(15, 10), sharex=True)

    colors = ['b', 'g', 'r']

    for i in range(3):
        axs[0,i].plot(t_full, pos_full[:,i], colors[i]+'-')
        axs[1,i].plot(t_full, vel_full[:,i], colors[i]+'-')
        axs[2,i].plot(t_full, acc_full[:,i], colors[i]+'-')

        # Plot all waypoints along the trajectory
        for seg in trajectory_segments:
            for wp_idx in range(seg['waypoints'].shape[0]):
                wp_time = seg['t'][int(wp_idx*(len(seg['t'])-1)/(seg['waypoints'].shape[0]-1))]
                wp_pos = seg['waypoints'][wp_idx, i]
                if wp_idx == 0:
                    axs[0,i].plot(wp_time, wp_pos, 'ro')  # start waypoint of segment
                elif wp_idx == seg['waypoints'].shape[0]-1:
                    axs[0,i].plot(wp_time, wp_pos, 'mo')  # holding point
                else:
                    axs[0,i].plot(wp_time, wp_pos, 'ko', markersize=4)  # internal waypoints

        # Plot holding windows
        for seg in trajectory_segments:
            if seg['hold_time'] > 0.0:
                axs[0,i].axvspan(seg['hold_at'], seg['hold_at']+seg['hold_time'], color='y', alpha=0.2)
                axs[1,i].axvspan(seg['hold_at'], seg['hold_at']+seg['hold_time'], color='y', alpha=0.2)
                axs[2,i].axvspan(seg['hold_at'], seg['hold_at']+seg['hold_time'], color='y', alpha=0.2)

    # Labels and titles
    axs[0,0].set_ylabel("X [m]"); axs[0,1].set_ylabel("Y [m]"); axs[0,2].set_ylabel("Z [m]")
    axs[0,0].set_title("Position vs Time"); axs[0,1].set_title("Position vs Time"); axs[0,2].set_title("Position vs Time")
    axs[1,0].set_ylabel("Vx [m/s]"); axs[1,1].set_ylabel("Vy [m/s]"); axs[1,2].set_ylabel("Vz [m/s]")
    axs[1,0].set_title("Velocity vs Time"); axs[1,1].set_title("Velocity vs Time"); axs[1,2].set_title("Velocity vs Time")
    axs[2,0].set_ylabel("Ax [m/s²]"); axs[2,1].set_ylabel("Ay [m/s²]"); axs[2,2].set_ylabel("Az [m/s²]")
    axs[2,0].set_xlabel("Time [s]"); axs[2,1].set_xlabel("Time [s]"); axs[2,2].set_xlabel("Time [s]")
    axs[2,0].set_title("Acceleration vs Time"); axs[2,1].set_title("Acceleration vs Time"); axs[2,2].set_title("Acceleration vs Time")
    plt.tight_layout()
    plt.show()
