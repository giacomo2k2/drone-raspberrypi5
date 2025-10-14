import numpy as np
import math
import cvxpy as cp
import matplotlib.pyplot as plt


# ============================================================
# Helper functions
# ============================================================

def factorial(n):
    return math.factorial(n)

def poly_derivative_matrix(order, deriv, t):
    """Return row vector mapping coefficients -> r-th derivative at time t."""
    row = np.zeros(order + 1)
    for i in range(deriv, order + 1):
        row[i] = factorial(i) / factorial(i - deriv) * (t ** (i - deriv))
    return row


def segment_time_trapezoid(distance, v_max, a_max):
    """Estimate time from distance with trapezoidal velocity profile."""
    t_acc = v_max / a_max
    d_acc = 0.5 * a_max * t_acc ** 2
    if 2 * d_acc > distance:
        # triangle profile
        t_acc = np.sqrt(distance / a_max)
        return 2 * t_acc
    d_flat = distance - 2 * d_acc
    t_flat = d_flat / v_max
    return 2 * t_acc + t_flat


def compute_Q_matrix(order, deriv_order, T):
    """Compute the cost matrix for ∫ (d^k p/dt^k)^2 dt"""
    n = order
    Q = np.zeros((n + 1, n + 1))
    for i in range(deriv_order, n + 1):
        for j in range(deriv_order, n + 1):
            Q[i, j] = (factorial(i) / factorial(i - deriv_order)) * \
                      (factorial(j) / factorial(j - deriv_order)) / \
                      (i + j - 2 * deriv_order + 1) * (T ** (i + j - 2 * deriv_order + 1))
    return Q


# ============================================================
# Min-Snap trajectory solver
# ============================================================

def solve_min_snap_1d(waypoints, T, order=7):
    """
    Solve 1D minimum-snap polynomial for given waypoints and segment times T.
    """
    N = len(waypoints) - 1
    coeffs = []

    # Construct the QP
    H = np.zeros(((order + 1) * N, (order + 1) * N))
    A_eq = []
    b_eq = []

    for i in range(N):
        Q = compute_Q_matrix(order, 4, T[i])
        H[i*(order+1):(i+1)*(order+1), i*(order+1):(i+1)*(order+1)] = Q

    # Boundary constraints
    # start and end pos, vel, acc, jerk = 0
    derivs = 4  # up to jerk continuity
    for r in range(derivs):
        row = np.zeros(((order+1)*N))
        row[:order+1] = poly_derivative_matrix(order, r, 0)
        A_eq.append(row)
        b_eq.append(0.0)
    for r in range(derivs):
        row = np.zeros(((order+1)*N))
        row[-(order+1):] = poly_derivative_matrix(order, r, T[-1])
        A_eq.append(row)
        b_eq.append(0.0)

    # intermediate continuity + position
    for i in range(N - 1):
        # position continuity
        row = np.zeros(((order+1)*N))
        row[i*(order+1):(i+1)*(order+1)] = poly_derivative_matrix(order, 0, T[i])
        row[(i+1)*(order+1):(i+2)*(order+1)] = -poly_derivative_matrix(order, 0, 0)
        A_eq.append(row)
        b_eq.append(0.0)

        # continuity of vel, acc, jerk
        for r in range(1, derivs):
            row = np.zeros(((order+1)*N))
            row[i*(order+1):(i+1)*(order+1)] = poly_derivative_matrix(order, r, T[i])
            row[(i+1)*(order+1):(i+2)*(order+1)] = -poly_derivative_matrix(order, r, 0)
            A_eq.append(row)
            b_eq.append(0.0)

    # position constraints (waypoints)
    for i in range(N + 1):
        row = np.zeros(((order+1)*N))
        if i == 0:
            row[:order+1] = poly_derivative_matrix(order, 0, 0)
        elif i == N:
            row[-(order+1):] = poly_derivative_matrix(order, 0, T[-1])
        else:
            row[i*(order+1):(i+1)*(order+1)] = poly_derivative_matrix(order, 0, 0)
        A_eq.append(row)
        b_eq.append(waypoints[i])

    A_eq = np.vstack(A_eq)
    b_eq = np.array(b_eq)

    x = cp.Variable((order+1)*N)
    cost = cp.quad_form(x, H)
    constraints = [A_eq @ x == b_eq]
    prob = cp.Problem(cp.Minimize(cost), constraints)
    prob.solve(solver=cp.OSQP)

    if prob.status != cp.OPTIMAL:
        raise RuntimeError("Min-snap QP failed")

    coeffs = np.array(x.value).reshape(N, order+1)
    return coeffs


# ============================================================
# Evaluation utilities
# ============================================================

def eval_poly(coeffs, t, deriv=0):
    n = len(coeffs) - 1
    val = 0.0
    for i in range(deriv, n + 1):
        val += coeffs[i] * factorial(i) / factorial(i - deriv) * (t ** (i - deriv))
    return val


# ============================================================
# Iterative time-scaling enforcement (recommended)
# ============================================================

def enforce_limits_min_snap(waypoints, v_max, a_max, order=7, max_iters=10):
    # initial times
    seg_dists = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)
    Ts = [segment_time_trapezoid(d, v_max, a_max) for d in seg_dists]

    for iteration in range(max_iters):
        coeffs_x = solve_min_snap_1d(waypoints[:, 0], Ts, order)
        coeffs_y = solve_min_snap_1d(waypoints[:, 1], Ts, order)
        coeffs_z = solve_min_snap_1d(waypoints[:, 2], Ts, order)

        violated = False
        for i, T in enumerate(Ts):
            ts = np.linspace(0, T, 100)
            for t in ts:
                vx = eval_poly(coeffs_x[i], t, 1)
                vy = eval_poly(coeffs_y[i], t, 1)
                vz = eval_poly(coeffs_z[i], t, 1)
                v = np.linalg.norm([vx, vy, vz])

                ax = eval_poly(coeffs_x[i], t, 2)
                ay = eval_poly(coeffs_y[i], t, 2)
                az = eval_poly(coeffs_z[i], t, 2)
                a = np.linalg.norm([ax, ay, az])

                if v > v_max or a > a_max:
                    Ts[i] *= 1.2
                    violated = True
                    break
            if violated:
                break
        if not violated:
            print(f"Trajectory feasible after {iteration+1} iterations.")
            return coeffs_x, coeffs_y, coeffs_z, Ts
    raise RuntimeError("Could not meet limits after all iterations.")


# ============================================================
# Example usage
# ============================================================

if __name__ == "__main__":
    # Define waypoints [x, y, z]
    waypoints = np.array([
        [0, 0, 0],
        [2, 1, 1],
        [4, 0, 2],
        [6, -1, 1],
        [8, 0, 0]
    ])

    v_max = 2.0   # m/s
    a_max = 2.0   # m/s^2

    coeffs_x, coeffs_y, coeffs_z, Ts = enforce_limits_min_snap(waypoints, v_max, a_max)

    # Plot trajectory
    traj = []
    for i, T in enumerate(Ts):
        ts = np.linspace(0, T, 50)
        for t in ts:
            traj.append([
                eval_poly(coeffs_x[i], t),
                eval_poly(coeffs_y[i], t),
                eval_poly(coeffs_z[i], t)
            ])
    traj = np.array(traj)

    plt.figure()
    plt.plot(waypoints[:, 0], waypoints[:, 1], 'ro--', label="Waypoints")
    plt.plot(traj[:, 0], traj[:, 1], 'b-', label="Trajectory")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.legend()
    plt.axis("equal")
    plt.title("Min-Snap Trajectory with Iterative Time-Scaling")
    plt.show()
