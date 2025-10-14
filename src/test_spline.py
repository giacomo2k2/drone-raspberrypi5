#!/usr/bin/env python3
import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt

# ------------------------
# User settings
# ------------------------
waypoints = np.array([
    [0.0, 0.0, 0.0],
    [0.0, 0.0, 10.0],
    [1.0, 6.0, 9.0],
    [0.0, 10.0, 10.0],

])
v_max = 2.0   # [units/s] max linear speed
a_max = 1.5   # [units/s^2] max linear acceleration
N = 1000      # number of samples along spline
EPS_SDOT = 1e-3  # minimum allowed s_dot to avoid div-by-zero (raised slightly)

# ------------------------
# 1) parameterize by cumulative chord length (s)
# ------------------------
n_wp = len(waypoints)
distances = np.zeros(n_wp)
for i in range(1, n_wp):
    distances[i] = distances[i-1] + np.linalg.norm(waypoints[i] - waypoints[i-1])
s_total = distances[-1]

# cubic splines for each coordinate (param = s)
cs_x = CubicSpline(distances, waypoints[:, 0])
cs_y = CubicSpline(distances, waypoints[:, 1])
cs_z = CubicSpline(distances, waypoints[:, 2])

# ------------------------
# 2) sample spline and derivatives w.r.t s
# ------------------------
s_sample = np.linspace(0.0, s_total, N)
dx_ds = cs_x.derivative()(s_sample)
dy_ds = cs_y.derivative()(s_sample)
dz_ds = cs_z.derivative()(s_sample)
d2x_ds2 = cs_x.derivative(2)(s_sample)
d2y_ds2 = cs_y.derivative(2)(s_sample)
d2z_ds2 = cs_z.derivative(2)(s_sample)

# magnitude of tangent (|p'(s)|) and curvature-like magnitude (|p''(s)|)
tangent_mag = np.sqrt(dx_ds**2 + dy_ds**2 + dz_ds**2)   # |p'(s)|
p2_mag = np.sqrt(d2x_ds2**2 + d2y_ds2**2 + d2z_ds2**2)   # |p''(s)|

# numerical protection (avoid exact zeros)
tangent_mag = np.maximum(tangent_mag, 1e-8)
p2_mag = np.maximum(p2_mag, 1e-12)

# ------------------------
# 3) compute conservative local s_dot_max(s)
# ------------------------
s_dot_from_v = v_max / tangent_mag
s_dot_from_a = np.sqrt(np.maximum(a_max / p2_mag, 0.0))
s_dot_max = np.minimum(s_dot_from_v, s_dot_from_a)
s_dot_max = np.maximum(s_dot_max, EPS_SDOT)

# ------------------------
# 4) forward-backward pass to enforce acceleration limits along s
#    using local tangential acceleration limit that accounts for curvature
# ------------------------
s_dot_fb = np.zeros_like(s_sample)
s_dot_fb[0] = 0.0  # start from rest

# forward pass
for i in range(1, N):
    ds = s_sample[i] - s_sample[i-1]
    s_dot_prev = s_dot_fb[i-1]
    a_t_sq = np.maximum(a_max**2 - (s_dot_prev**4) * (p2_mag[i-1]**2), 0.0)
    a_t_local = np.sqrt(a_t_sq) / tangent_mag[i-1]
    reachable = np.sqrt(s_dot_prev**2 + 2.0 * a_t_local * ds)
    s_dot_fb[i] = min(reachable, s_dot_max[i])

# backward pass
for i in range(N-2, -1, -1):
    ds = s_sample[i+1] - s_sample[i]
    s_dot_next = s_dot_fb[i+1]
    a_t_sq = np.maximum(a_max**2 - (s_dot_next**4) * (p2_mag[i+1]**2), 0.0)
    a_t_local = np.sqrt(a_t_sq) / tangent_mag[i+1]
    reachable = np.sqrt(s_dot_next**2 + 2.0 * a_t_local * ds)
    s_dot_fb[i] = min(s_dot_fb[i], reachable)

# ensure no very small zeros (practical floor)
s_dot_fb = np.maximum(s_dot_fb, EPS_SDOT)

# ------------------------
# 5) compute time profile using trapezoidal rule per segment:
#    dt_i = 2 * ds / (s_dot_i + s_dot_{i+1})
#    this avoids division by near-zero when one endpoint has s_dot ~ 0
# ------------------------
ds_array = np.diff(s_sample)
den = s_dot_fb[:-1] + s_dot_fb[1:]
den = np.maximum(den, 2.0 * EPS_SDOT)  # avoid tiny denominators
dt = 2.0 * ds_array / den
# guard tiny/negative dt (shouldn't happen)
dt = np.maximum(dt, 1e-12)
t_profile = np.concatenate(([0.0], np.cumsum(dt)))

# ------------------------
# 6) compute pos, vel, acc in world frame
# ------------------------
pos = np.vstack([cs_x(s_sample), cs_y(s_sample), cs_z(s_sample)]).T
s_dot = s_dot_fb
# s_ddot: derivative of s_dot with respect to time (t_profile may be non-uniform)
s_ddot = np.gradient(s_dot, t_profile)

# world velocities and accelerations (chain rule)
vel = np.vstack([dx_ds * s_dot, dy_ds * s_dot, dz_ds * s_dot]).T
acc = np.vstack([
    d2x_ds2 * s_dot**2 + dx_ds * s_ddot,
    d2y_ds2 * s_dot**2 + dy_ds * s_ddot,
    d2z_ds2 * s_dot**2 + dz_ds * s_ddot
]).T

vel_mag = np.linalg.norm(vel, axis=1)
acc_mag = np.linalg.norm(acc, axis=1)

# ------------------------
# 7) verification (print maxima)
# ------------------------
print(f"Total time: {t_profile[-1]:.6f} s")
print(f"max linear speed observed: {vel_mag.max():.6f}  (v_max = {v_max})")
print(f"max linear accel observed: {acc_mag.max():.6f}  (a_max = {a_max})")

# ------------------------
# 8) plotting (2D projections and v/a)
# ------------------------
plt.figure(figsize=(12, 5))

plt.subplot(1, 3, 1)
plt.plot(pos[:, 0], pos[:, 1], '-', linewidth=1)
plt.scatter(waypoints[:, 0], waypoints[:, 1], c='r')
plt.xlabel('X'); plt.ylabel('Y'); plt.title('X-Y (plan view)'); plt.axis('equal')

plt.subplot(1, 3, 2)
plt.plot(pos[:, 0], pos[:, 2], '-', linewidth=1)
plt.scatter(waypoints[:, 0], waypoints[:, 2], c='r')
plt.xlabel('X'); plt.ylabel('Z'); plt.title('X-Z view'); plt.axis('equal')

plt.subplot(1, 3, 3)
plt.plot(pos[:, 1], pos[:, 2], '-', linewidth=1)
plt.scatter(waypoints[:, 1], waypoints[:, 2], c='r')
plt.xlabel('Y'); plt.ylabel('Z'); plt.title('Y-Z view'); plt.axis('equal')

plt.tight_layout()
plt.show()

plt.figure()
plt.plot(t_profile, vel_mag, label='speed (|v|)')
plt.plot(t_profile, acc_mag, label='acceleration (|a|)')
plt.hlines(v_max, t_profile[0], t_profile[-1], colors='k', linestyles='dashed', label='v_max')
plt.hlines(a_max, t_profile[0], t_profile[-1], colors='k', linestyles='dotted', label='a_max')
plt.xlabel('time [s]'); plt.legend(); plt.title('speed and acceleration magnitude over time')
plt.grid(True)
plt.show()
