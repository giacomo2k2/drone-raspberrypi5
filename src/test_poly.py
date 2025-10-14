import numpy as np
import math
import cvxpy as cp
import matplotlib.pyplot as plt



def compute_Q_matrix(order, min_order, T):
    n = order
    Q = np.zeros((n+1, n+1))

    for i in range(min_order, n+1): 
        for j in range(min_order, n+1):
            Q[i, j] = math.factorial(i)/math.factorial(i-min_order) * math.factorial(j)/math.factorial(j-min_order) * T**(i+j-2*min_order +1)/(i+j-2*min_order+1)
    return Q


def poly_derivative_matrix(order, deriv, t):
    """Row vector mapping coefficients -> r-th derivative at tau in normalized [0,1]."""
    row = np.zeros(order+1)
    for i in range(deriv, order+1):
        row[i] = math.factorial(i)/math.factorial(i-deriv) * t**(i-deriv)
    return row


def eval_poly(coeffs, ts, der_order=0):
    val = np.zeros_like(ts)
    n = len(coeffs)
    for i in range(der_order, n):
        val += coeffs[i] * math.factorial(i)/math.factorial(i-der_order) * ts**(i-der_order)
    return val

def solve_min_QP_1d(nodes, periods, order, continuity_order):
    N = len(periods)

    coeffs = []

    H = np.zeros((N*(order+1), N*(order+1)))
    A = []
    b = []

    for i in range(N):
        Q = compute_Q_matrix(order, continuity_order, periods[i])
        H[i*(order+1):(i+1)*(order+1), i*(order+1):(i+1)*(order+1)] = Q

    # boundary constraints

    # 1) Start boundary constraints (t=0)
    for r in range(continuity_order):
        row = np.zeros(N*(order+1))
        row[:order+1] = poly_derivative_matrix(order, deriv=r, t=0)
        if r == 0:
            b.append(nodes[0])  # position
        else:
            b.append(0.0)       # derivatives
        A.append(row)

    # 2) End boundary constraints (t=T_last)
    for r in range(continuity_order):
        row = np.zeros(N*(order+1))
        row[-(order+1):] = poly_derivative_matrix(order, deriv=r, t=periods[-1])
        if r == 0:
            b.append(nodes[-1])  # position
        else:
            b.append(0.0)        # derivatives
        A.append(row)

    # 3 Intermediate continuity (derivatives 1..continuity_order-1)
    for i in range(1, N):
        for r in range(continuity_order):
            row = np.zeros(N*(order+1))
            # end of segment i
            row[i*(order+1):(i+1)*(order+1)] = poly_derivative_matrix(order, deriv=r, t=0)
            # start of segment i-1
            row[(i-1)*(order+1):i*(order+1)] = -poly_derivative_matrix(order, deriv=r, t=periods[i-1])
            b.append(0.0)
            A.append(row)   


    # 4 Internal position constraints
    for i in range(1, N):  # internal nodes only
        row = np.zeros(N*(order+1))
        # end of segment i-1
        row[i*(order+1):(i+1)*(order+1)] = poly_derivative_matrix(order, deriv=0, t=0)
        b.append(nodes[i])
        A.append(row)



    A = np.vstack(A) 
    b = np.array(b)

    x = cp.Variable((order+1)*N)
    x = cp.Variable(H.shape[0])
    cost = cp.quad_form(x, H)

    cost = cp.quad_form(x, cp.psd_wrap(H))
    constraints = [A @ x == b]
    prob = cp.Problem(cp.Minimize(cost), constraints=constraints)
    prob.solve(solver = cp.OSQP)

    if prob.status != cp.OPTIMAL:
        raise RuntimeError("QP failed")

    coeffs = np.array(x.value).reshape(N, order+1)
    return coeffs


def find_norm_periods(waypoints):
    
    distances = np.linalg.norm(np.diff(waypoints, n=1, axis=0), axis=1)
    total_distance = np.sum(distances)

    norm_periods = distances / total_distance

    return norm_periods


def build_limited_trajectory_normalized(waypoints, vmax, amax, order=7, continuity_order=4):
    
    norm_periods = find_norm_periods(waypoints)
    coeffs = solve_min_QP_1d(waypoints[:, 0], norm_periods, order, continuity_order)

    
    return coeffs, norm_periods



def sample_trajectory(coeffs, periods, sample_rate = 10):
    t = np.array([])
    p = np.array([])
    v = np.array([])
    a = np.array([])

    for i, T in enumerate(periods):
        ts = np.linspace(0, T, sample_rate)

        
        pos = eval_poly(coeffs[i], ts, der_order = 0)
        vel = eval_poly(coeffs[i], ts, der_order = 1)
        acc = eval_poly(coeffs[i], ts, der_order = 2)

        
        if t.size == 0:
            t = ts
            p = pos
            v = vel
            a = acc
        else:
            t = np.concatenate((t, ts[1:]+t[-1]))
            p = np.concatenate((p, pos[1:]))
            v = np.concatenate((v, vel[1:]))
            a = np.concatenate((a, acc[1:]))
        
    return t, p, v, a


import numpy as np

def rescale_coeffs(coeffs, scaling_factor):
   
    coeffs = np.asarray(coeffs)
    s = scaling_factor
    # Compute per-power scale factors: [1, 1/s, 1/s^2, ..., 1/s^(M-1)]
    scale_factors = s ** (-np.arange(coeffs.shape[1]))
    # Broadcast across all segments
    
    scaled_coeffs =  coeffs * scale_factors[np.newaxis, :]

    return scaled_coeffs


def rescale_periods(norm_coeffs, norm_periods, vmax, amax, max_iter = 100):

    periods = norm_periods.copy()
    coeffs = norm_coeffs.copy()
    scaling_factor = 1.1

    for _ in range(max_iter):
        t_violates = False
        for i, T in enumerate(periods):
            ts = np.linspace(0, T, 100)
            vel = eval_poly(coeffs[i], ts, 1)
            acc = eval_poly(coeffs[i], ts, 2)
            if np.max(np.abs(vel)) > vmax or np.max(np.abs(acc)) > amax:
                t_violates = True
                break
        if t_violates:
            periods *= scaling_factor
            coeffs = rescale_coeffs(coeffs, scaling_factor)
        else:
            break


        
    return periods, coeffs

# ------------------ Example ------------------
if __name__ == '__main__':
    waypoints = np.array([
        [0], 
        [1],
        [10],
        [50],
        [100],
    ])

    order = 5
    continuity_order = 3
    vmax = 2.0
    amax = 5.0

    norm_coeffs, norm_periods = build_limited_trajectory_normalized(waypoints, vmax, amax, order, continuity_order=continuity_order)
    periods, coeffs = rescale_periods(norm_coeffs, norm_periods, vmax, amax)

    t, p, v, a = sample_trajectory(coeffs, periods, sample_rate=100)

    # Plot
    plt.figure(figsize=(10, 8))
    plt.subplot(3,1,1)
    plt.plot(np.cumsum(np.concatenate(([0.0], periods))), waypoints[:, 0], 'ro', label="Waypoints")
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
