import numpy as np
import math
import cvxpy as cp

# ------------------ Polynomial helpers ------------------
def compute_Q_unit(order, min_order):
    """Compute the Q matrix for minimum-derivative cost in QP."""
    Q = np.zeros((order+1, order+1))
    for i in range(min_order, order+1):
        for j in range(min_order, order+1):
            Q[i, j] = (math.factorial(i) / math.factorial(i - min_order) *
                       math.factorial(j) / math.factorial(j - min_order) /
                       (i + j - 2 * min_order + 1))
    return Q

def poly_derivative_row(order, deriv, tau):
    """Return a row of the derivative constraint matrix for a polynomial of given order."""
    row = np.zeros(order+1)
    for i in range(deriv, order+1):
        row[i] = math.factorial(i)/math.factorial(i-deriv) * tau**(i-deriv)
    return row

def eval_poly(coeffs, tau, der_order=0):
    """Evaluate polynomial or its derivative at given tau."""
    val = np.zeros_like(tau)
    n = len(coeffs)
    for i in range(der_order, n):
        val += coeffs[i] * math.factorial(i)/math.factorial(i-der_order) * tau**(i-der_order)
    return val

# ------------------ Trajectory QP solver ------------------
def solve_min_QP_axis(nodes, periods, order=5, continuity_order=3, min_derivative_order=2, debug=False):
    """
    Solve a QP for one axis to minimize the integral of squared min_derivative_order derivative.
    Returns coefficients for each segment.
    """
    N = len(periods)  # number of segments
    H = np.zeros((N*(order+1), N*(order+1)))
    A = []
    b = []

    # Compute unit Q matrix
    Q_unit = compute_Q_unit(order, min_derivative_order)
    if debug: print("Q_unit:\n", Q_unit)

    # Build block-diagonal H matrix for all segments
    for i in range(N):
        scale = periods[i]**(1 - 2*min_derivative_order)
        H[i*(order+1):(i+1)*(order+1), i*(order+1):(i+1)*(order+1)] = Q_unit * scale

    # --- Boundary conditions ---
    # Start
    for r in range(continuity_order):
        row = np.zeros(N*(order+1))
        row[:order+1] = poly_derivative_row(order, r, 0) / (periods[0]**r if r>0 else 1)
        b.append(nodes[0] if r==0 else 0.0)
        A.append(row)
    # End
    for r in range(continuity_order):
        row = np.zeros(N*(order+1))
        row[-(order+1):] = poly_derivative_row(order, r, 1) / (periods[-1]**r if r>0 else 1)
        b.append(nodes[-1] if r==0 else 0.0)
        A.append(row)

    # --- Continuity at intermediate nodes ---
    for i in range(1, N):
        T_prev, T_i = periods[i-1], periods[i]
        for r in range(continuity_order):
            row = np.zeros(N*(order+1))
            row[(i-1)*(order+1):i*(order+1)] = poly_derivative_row(order, r, 1) / (T_prev**r if r>0 else 1)
            row[i*(order+1):(i+1)*(order+1)] = -poly_derivative_row(order, r, 0) / (T_i**r if r>0 else 1)
            b.append(0.0)
            A.append(row)

    # --- Internal position constraints ---
    for i in range(1, N):
        row = np.zeros(N*(order+1))
        row[(i-1)*(order+1):i*(order+1)] = poly_derivative_row(order, 0, 1)
        b.append(nodes[i])
        A.append(row)

    A = np.vstack(A)
    b = np.array(b)

    # Solve QP
    x = cp.Variable(N*(order+1))
    prob = cp.Problem(cp.Minimize(cp.quad_form(x, cp.psd_wrap(H))),
                      [A @ x == b])
    prob.solve(solver=cp.OSQP)

    if prob.status != cp.OPTIMAL:
        raise RuntimeError("QP failed to find a solution.")

    coeffs = np.array(x.value).reshape(N, order+1)
    if debug:
        print("QP solved, coefficients shape:", coeffs.shape)
    return coeffs

# ------------------ Time / period allocation ------------------
def initial_periods(waypoints, vmax, amax, safety_factor=1.2, min_duration=0.05, debug=False):
    """Compute initial normalized segment durations based on vmax and amax constraints."""
    distances = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)
    periods = np.zeros_like(distances, dtype=float)
    for i, d in enumerate(distances):
        T_v = d / vmax
        T_a = np.sqrt(d / amax)
        periods[i] = max(T_v, T_a) * safety_factor
        periods[i] = max(periods[i], min_duration)
    # Normalize to sum = 1 for QP
    periods /= np.sum(periods)
    if debug: print("Initial periods:", periods)
    return periods

def build_trajectory_normalized_3d(waypoints, vmax, amax, order=5, continuity_order=3, min_derivative_order=2, debug=False):
    """Build trajectory for 3D waypoints. Returns coeffs_xyz and normalized periods."""
    periods = initial_periods(waypoints, vmax, amax, debug=debug)
    coeffs_xyz = []
    for dim in range(waypoints.shape[1]):
        coeffs = solve_min_QP_axis(
            waypoints[:, dim],
            periods,
            order,
            continuity_order,
            min_derivative_order,
            debug=debug
        )
        coeffs_xyz.append(coeffs)
    return np.array(coeffs_xyz), periods

# ------------------ Rescale periods to satisfy physical vmax/amax ------------------
def uniform_rescale_periods_3d_norm(coeffs_x, coeffs_y, coeffs_z, periods, vmax, amax, sample_rate=100, debug=False):
    """Rescale segment times to ensure velocity/acceleration limits are respected."""
    periods = np.array(periods, dtype=float)
    N = len(periods)
    scale = 1.0

    for i in range(N):
        tau = np.linspace(0, 1, sample_rate)
        vel_x = eval_poly(coeffs_x[i], tau, 1)
        vel_y = eval_poly(coeffs_y[i], tau, 1)
        vel_z = eval_poly(coeffs_z[i], tau, 1)
        acc_x = eval_poly(coeffs_x[i], tau, 2)
        acc_y = eval_poly(coeffs_y[i], tau, 2)
        acc_z = eval_poly(coeffs_z[i], tau, 2)

        # Convert normalized derivatives to physical units
        vel_phys = np.column_stack([vel_x, vel_y, vel_z]) / periods[i]
        acc_phys = np.column_stack([acc_x, acc_y, acc_z]) / (periods[i]**2)

        vel_norm = np.linalg.norm(vel_phys, axis=1)
        acc_norm = np.linalg.norm(acc_phys, axis=1)

        scale_v = np.max(vel_norm) / vmax if vmax > 0 else 0
        scale_a = np.sqrt(np.max(acc_norm) / amax) if amax > 0 else 0
        scale = max(scale, scale_v, scale_a, 1.0)

    periods *= scale
    if debug: print("Rescaled periods:", periods)
    return periods
