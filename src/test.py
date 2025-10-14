import numpy as np
import math
import cvxpy as cp
import matplotlib.pyplot as plt

def duplicate_holds(waypoints): 
    
    nodes = []
    dwells = []

    for wp in waypoints: 
        nodes.append(float(wp["pos"]))
        dwells.append(float(0.0))

        if wp.get("hold", None) and wp["hold"] > 0:
            nodes.append(float(wp["pos"]))
            dwells.append(float(wp["hold"]))

    nodes = np.array(nodes)
    dwells = np.array(dwells[1:])
    
    return nodes, dwells

def get_trapz_periods(nodes, dwells, vmax, amax): 
    
    periods = []
    

    ############### CHANGE ABS WHEN SWITCHING TO 3D ###############################
    distances = np.abs(np.diff(nodes, n=1))

    
    
    for dist, dwell in zip(distances, dwells):
        period = 0.0
        if dwell > 0:
            period = dwell
        else: 
            if dist <= (vmax**2)/amax:
                period = 2*math.sqrt(dist/amax)
            else:
                period = dist/vmax + vmax/amax
        periods.append(period)

    periods = np.array(periods)

    
    return periods


def compute_Q_matrix(order, min_order, T):
    n = order
    Q = np.zeros((n+1, n+1))

    for i in range(min_order, n+1): 
        for j in range(min_order, n+1):
            Q[i, j] = math.factorial(i)/math.factorial(i-min_order) * math.factorial(j)/math.factorial(j-min_order) * T**(i+j-2*min_order +1)/(i+j-2*min_order+1)
    return Q


def poly_derivative_matrix(order, deriv, t):
    """Return row vector mapping coefficients -> r-th derivative at time t."""
    row = np.zeros(order + 1, dtype=float)
    for i in range(deriv, order + 1):
        row[i] = math.factorial(i) / math.factorial(i - deriv) * (t ** (i - deriv))
    return row




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
        for r in range(2):
            row = np.zeros(N*(order+1))
            # end of segment i
            row[i*(order+1):(i+1)*(order+1)] = poly_derivative_matrix(order, deriv=r, t=0)
            # start of segment i-1
            print(periods[i-1])
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


def eval_poly(coeffs, ts, der_order):
    n = len(coeffs)
    val = np.zeros(np.shape(ts))
    
    for i in range(der_order, n):
        val += coeffs[i] * math.factorial(i) / math.factorial(i-der_order) * np.power(ts, i - der_order)

    return val


def build_limited_trajectory_QP(waypoints, vmax, amax, order = 7, continuity_order = 4, max_iter=10, alpha = 1.2): 

    # get nodes array and dwells array
    nodes, dwells = duplicate_holds(waypoints)

    print(f"nodes: {nodes}")
    print(f"dwells: {dwells}")

    periods = get_trapz_periods(nodes, dwells, vmax, amax)
    
    print(f"periods: {periods}")


    for iteration in range(max_iter):
        coeffs = solve_min_QP_1d(nodes, periods, order, continuity_order)

        violated = False

        for i, T in enumerate(periods):
            ts = np.linspace(0, T, 10)

            vel = np.abs(eval_poly(coeffs[i], ts, der_order = 1))
            acc = np.abs(eval_poly(coeffs[i], ts, der_order = 2))

            vel_max = np.max(vel)
            acc_max = np.max(acc)
            

            if vel_max > vmax or acc_max > amax:
                
                periods[i] *= alpha
                violated = True
                break

            if violated:
                break

        if not violated:
            print(f"Trajectory feasible after {iteration+1} iterations.")
            return coeffs, periods, nodes
    
    raise RuntimeError("Could not meet limits after all iterations.")



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




if __name__ == '__main__':
    waypoints = [
        {"pos": 0},
        {"pos": 50}, 
        {"pos": 100},     
    ]  

    order = 3

    coeffs, periods, nodes = build_limited_trajectory_QP(waypoints, vmax=1.0, amax=5.0, continuity_order=2, order=order, max_iter=10)

    print(periods)

    t, p, v, a = sample_trajectory(coeffs, periods, sample_rate=100)

    
    

    # Plot trajectory    
    plt.figure(figsize=(10, 8))

    # ---- Position ----
    plt.subplot(3, 1, 1)
    plt.plot(np.cumsum(np.concatenate(([0.0], periods))), nodes, 'ro', label="Waypoints")
    plt.plot(t, p, 'b-', label="Trajectory")
    plt.ylabel("Position [m]")
    plt.legend()
    plt.grid(True)

    # ---- Velocity ----
    plt.subplot(3, 1, 2)
    plt.plot(t, v, 'g-', label="Velocity")
    plt.ylabel("Velocity [m/s]")
    plt.legend()
    plt.grid(True)

    # ---- Acceleration ----
    plt.subplot(3, 1, 3)
    plt.plot(t, a, 'r-', label="Acceleration")
    plt.xlabel("Time [s]")
    plt.ylabel("Acceleration [m/s²]")
    plt.legend()
    plt.grid(True)

    plt.suptitle("Min-Snap Trajectory with Iterative Time-Scaling", fontsize=14)
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.show()


    # Print some sample points near the special waypoints to verify behavior
    # print("\nSample outputs (time, pos, vel, acc) head and around key points:")
    # for i in range(0, min(10, len(t))):
    #     print(f"{t[i]:.3f}  {p[i]:.3f}  {v[i]:.3f}  {a[i]:.3f}")
