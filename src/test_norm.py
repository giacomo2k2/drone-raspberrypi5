import numpy as np
import math
import cvxpy as cp
import matplotlib.pyplot as plt

def duplicate_holds(waypoints):
    nodes = []
    dwells = []
    for wp in waypoints:
        nodes.append(float(wp["pos"]))
        dwells.append(0.0)
        if wp.get("hold", 0) > 0:
            nodes.append(float(wp["pos"]))
            dwells.append(float(wp["hold"]))
    return np.array(nodes), np.array(dwells[1:])

def get_trapz_periods(nodes, dwells, vmax, amax):
    distances = np.abs(np.diff(nodes))
    periods = []
    for d, dwell in zip(distances, dwells):
        if dwell > 0:
            periods.append(dwell)
        else:
            if d <= (vmax**2)/amax:
                periods.append(2*math.sqrt(d/amax))
            else:
                periods.append(d/vmax + vmax/amax)
    return np.array(periods)

def compute_Q_matrix(order, min_order):
    """Normalized time Q matrix for min-snap or min-derivative problems."""
    n = order
    Q = np.zeros((n+1, n+1))
    for i in range(min_order, n+1):
        for j in range(min_order, n+1):
            Q[i,j] = (math.factorial(i)/math.factorial(i-min_order)) * \
                     (math.factorial(j)/math.factorial(j-min_order)) / (i+j-2*min_order+1)
    return Q

def poly_derivative_matrix(order, deriv, tau):
    """Row vector mapping coefficients -> r-th derivative at tau in normalized [0,1]."""
    row = np.zeros(order+1)
    for i in range(deriv, order+1):
        row[i] = math.factorial(i)/math.factorial(i-deriv) * tau**(i-deriv)
    return row

def solve_min_QP_1d_normalized(nodes, periods, order=7, continuity_order=4, vmax=None, amax=None, num_samples=10):
    N = len(periods)
    H = np.zeros((N*(order+1), N*(order+1)))
    constraints = []
    x = cp.Variable(N*(order+1))

    # Build block-diagonal Q in normalized time, scaled by segment duration
    for i in range(N):
        Qi = compute_Q_matrix(order, continuity_order)
        # Scale cost by T^(1-2*continuity_order)
        T = periods[i]
        H[i*(order+1):(i+1)*(order+1), i*(order+1):(i+1)*(order+1)] = Qi / T**(2*continuity_order-1)

    # 1) Start constraints at tau=0
    for r in range(continuity_order):
        row = np.zeros(N*(order+1))
        row[:order+1] = poly_derivative_matrix(order, r, 0)
        constraints.append(row @ x == (nodes[0] if r==0 else 0.0))

    # 2) End constraints at tau=1
    for r in range(continuity_order):
        row = np.zeros(N*(order+1))
        row[-(order+1):] = poly_derivative_matrix(order, r, 1)
        constraints.append(row @ x == (nodes[-1] if r==0 else 0.0))

    # 3) Intermediate continuity constraints at tau=1 (end) and tau=0 (start)
    for i in range(1, N):
        for r in range(continuity_order):
            row = np.zeros(N*(order+1))
            row[(i-1)*(order+1):i*(order+1)] = poly_derivative_matrix(order, r, 1) / periods[i-1]**r
            row[i*(order+1):(i+1)*(order+1)] = -poly_derivative_matrix(order, r, 0) / periods[i]**r
            constraints.append(row @ x == 0.0)

    # 4) Internal positions at segment starts (tau=0)
    for i in range(1, N):
        row = np.zeros(N*(order+1))
        row[i*(order+1):(i+1)*(order+1)] = poly_derivative_matrix(order, 0, 0)
        constraints.append(row @ x == nodes[i])

    # 5) Optional explicit derivative limits at sampled points
    
    for i in range(N):
        T = periods[i]
        for tau in np.linspace(0, 1, num_samples):
            if vmax is not None:
                row = np.zeros(N*(order+1))
                row[i*(order+1):(i+1)*(order+1)] = poly_derivative_matrix(order, 1, tau) / periods[i]
                constraints.append(row @ x <= vmax)
                constraints.append(row @ x >= -vmax)

            if amax is not None:
                row = np.zeros(N*(order+1))
                row[i*(order+1):(i+1)*(order+1)] = poly_derivative_matrix(order, 2, tau) / periods[i]**2
                constraints.append(row @ x <= amax)
                constraints.append(row @ x >= -amax)


    # Solve QP
    cost = cp.quad_form(x, cp.psd_wrap(H))
    prob = cp.Problem(cp.Minimize(cost), constraints)
    prob.solve(solver=cp.OSQP)

    if prob.status != cp.OPTIMAL and prob.status != cp.OPTIMAL_INACCURATE:
        raise RuntimeError("QP failed")

    coeffs = np.array(x.value).reshape(N, order+1)
    return coeffs

def eval_poly(coeffs, ts, der_order=0):
    val = np.zeros_like(ts)
    n = len(coeffs)
    for i in range(der_order, n):
        val += coeffs[i] * math.factorial(i)/math.factorial(i-der_order) * ts**(i-der_order)
    return val

def sample_trajectory_normalized(coeffs, periods, sample_rate=100):
    t_total = np.array([])
    p_total = np.array([])
    v_total = np.array([])
    a_total = np.array([])
    N = len(periods)

    for i in range(N):
        T = periods[i]
        tau = np.linspace(0, 1, sample_rate)
        pos = eval_poly(coeffs[i], tau)
        vel = eval_poly(coeffs[i], tau, 1) / T
        acc = eval_poly(coeffs[i], tau, 2) / T**2

        if t_total.size == 0:
            t_total = tau*T
            p_total = pos
            v_total = vel
            a_total = acc
        else:
            t_total = np.concatenate((t_total, tau[1:]*T + t_total[-1]))
            p_total = np.concatenate((p_total, pos[1:]))
            v_total = np.concatenate((v_total, vel[1:]))
            a_total = np.concatenate((a_total, acc[1:]))

    return t_total, p_total, v_total, a_total

def build_limited_trajectory_normalized(waypoints, vmax, amax, order=7, continuity_order=4):
    nodes, dwells = duplicate_holds(waypoints)
    periods = get_trapz_periods(nodes, dwells, vmax, amax)
    coeffs = solve_min_QP_1d_normalized(nodes, periods, order, continuity_order, vmax, amax)
    return coeffs, periods, nodes

# ------------------ Example ------------------
if __name__ == '__main__':
    waypoints = [{"pos": 0}, {"pos": 100}, {"pos": -200}, {"pos": 0}]
    order = 7
    vmax = 2.0
    amax = 5.0

    coeffs, periods, nodes = build_limited_trajectory_normalized(waypoints, vmax, amax, order)

    t, p, v, a = sample_trajectory_normalized(coeffs, periods, sample_rate=100)

    # Plot
    plt.figure(figsize=(10, 8))
    plt.subplot(3,1,1)
    plt.plot(np.cumsum(np.concatenate(([0.0], periods))), nodes, 'ro', label="Waypoints")
    plt.plot(t, p, 'b-', label="Trajectory")
    plt.ylabel("Position [m]")
    plt.legend(); plt.grid(True)

    plt.subplot(3,1,2)
    plt.plot(t, v, 'g-', label="Velocity")
    plt.ylabel("Velocity [m/s]"); plt.legend(); plt.grid(True)

    plt.subplot(3,1,3)
    plt.plot(t, a, 'r-', label="Acceleration")
    plt.ylabel("Acceleration [m/s²]"); plt.xlabel("Time [s]"); plt.legend(); plt.grid(True)

    plt.suptitle("Min-Snap Trajectory with Normalized Time", fontsize=14)
    plt.tight_layout(rect=[0,0,1,0.96])
    plt.show()
