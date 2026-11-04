"""LQR design for the Furuta pendulum with a STEPPER arm (input = arm angular acceleration).

Run:  uv run --no-project --with numpy --with scipy python analysis/lqr_design.py

Why acceleration input: the stepper sets the arm motion directly (it does not apply a chosen torque), so as long as it
does not miss steps the arm angle obeys  theta'' = u.  Only the pendulum equation needs physical parameters:

    J_p a'' + b_p a' + m_p g l_p sin(a) = s * m_p l_p L_r cos(a) theta''     (a measured from HANGING DOWN)
    divide by J_p:   a'' + 2 zeta w_n a' + w_n^2 sin(a) = s * (L_r / l_eq) cos(a) u,   l_eq = J_p/(m_p l_p) = g / w_n^2

All of w_n, zeta come from the free-decay tests, L_r is measured, and the sign s = +1 was read from the sine test
(1 Hz drive: pendulum about 180 deg out of phase with the arm, as the model with s = +1 predicts below resonance).
m_p and l_p are NOT needed for this model (they are needed for the handout's torque model / parameter sheet).

Upright: d = a - pi (d = 0 upright). Linearised:  d'' = w_n^2 d - 2 zeta w_n d' - s K u,   K = L_r / l_eq.
State x = [theta, d, theta', d'] (rad, rad/s), u = theta'' (rad/s^2). Sign conventions as in the logs:
theta > 0 = clockwise seen from above (commanded steps), d from the AS5600 counts direction.
"""
import numpy as np
from scipy.linalg import solve_discrete_are, expm

# ---- measured / identified (SI) ----
g = 9.81
w_n = 2 * np.pi / 0.52          # decay tests: T = 0.516 / 0.523 s
zeta = 0.06                      # equivalent viscous, first cycle (hinge friction is mostly dry)
L_r = 0.070                      # motor axis -> pendulum hinge [m] (user, 7 cm)
l_eq = g / w_n**2                # equivalent length J_p/(m_p l_p)
K = L_r / l_eq
s = +1
dt = 0.005                       # 200 Hz loop

A = np.array([[0, 0, 1, 0],
              [0, 0, 0, 1],
              [0, 0, 0, 0],
              [0, w_n**2, 0, -2 * zeta * w_n]])
B = np.array([[0], [0], [1], [-s * K]])

# ---- discretise (zero-order hold) ----
M = expm(np.block([[A, B], [np.zeros((1, 5))]]) * dt)
Ad, Bd = M[:4, :4], M[:4, 4:]

# ---- weights: 1 / (acceptable size)^2  (Bryson's rule) ----
import sys
# Two weightings for the handout's in-lab task 6 ("repeat with a modified LQR weighting and compare"):
#   A (moderate, first hardware test): pendulum error 0.08 rad, acceleration 15 rad/s^2
#   B (stiffer pendulum hold):         pendulum error 0.05 rad, pendulum rate 3 rad/s, acceleration 20 rad/s^2
#   Run:  ... python analysis/lqr_design.py A   (or B)
WEIGHTS = {"A": dict(theta=0.5, d=0.08, theta_dot=3.0, d_dot=5.0, u=15.0),
           "B": dict(theta=0.5, d=0.05, theta_dot=3.0, d_dot=3.0, u=20.0)}
SET = sys.argv[1].upper() if len(sys.argv) > 1 else "A"
wt = WEIGHTS[SET]
print(f"weighting {SET}: {wt}")
Q = np.diag([1 / wt["theta"]**2, 1 / wt["d"]**2, 1 / wt["theta_dot"]**2, 1 / wt["d_dot"]**2])  # Bryson: 1/(acceptable size)^2
R = np.array([[1 / wt["u"]**2]])
# (Chosen after a sweep: tighter weights (0.03 rad, 40 rad/s^2) balanced in simulation but chattered at the
#  acceleration limit because of encoder quantisation + derivative noise; these give smooth control.)

P = solve_discrete_are(Ad, Bd, Q, R)
Kd = np.linalg.solve(R + Bd.T @ P @ Bd, Bd.T @ P @ Ad)     # u = -Kd x
eig_ol = np.linalg.eigvals(A)
eig_cl = np.abs(np.linalg.eigvals(Ad - Bd @ Kd))

print(f"w_n = {w_n:.2f} rad/s, l_eq = {l_eq*100:.2f} cm, K = L_r/l_eq = {K:.3f}")
print("open-loop eigenvalues (continuous):", np.round(eig_ol, 2), "-> one unstable mode, as expected")
ctrb = np.hstack([B, A @ B, A @ A @ B, A @ A @ A @ B])
print("controllability rank:", np.linalg.matrix_rank(ctrb))
print("LQR gains  u = -(k1 theta + k2 d + k3 theta' + k4 d'):", np.round(Kd.ravel(), 3))
print("closed-loop |eig| (discrete, must be < 1):", np.round(eig_cl, 4))

# ---- nonlinear simulation, run the way the sketch runs: 5 deg start, quantised encoder, d' from a filtered
#      difference, theta' = commanded speed, 1-sample delay, saturation. Repeated with the plant gain K off by +-20 %.
U_MAX = 60.0                # rad/s^2, same cap as the sketch
V_MAX = np.radians(360)     # rad/s
TAU = 1 / (2 * np.pi * 25)  # derivative low-pass, 25 Hz (same as the sketch)
q = 2 * np.pi / 4096

KI_TRIM = 0.023             # slow auto-trim of the upright offset (same as the sketch)
def simulate(K_plant, d0=np.radians(5), T=3.0, bias=0.0, trim=True):
    x = np.array([0.0, d0, 0.0, 0.0]); u_prev = 0.0; d_prev = None; dd_f = 0.0; c = 0.0; log = []
    for k in range(int(T / dt)):
        d_meas = np.round((x[1] + bias) / q) * q - c   # bias = upright calibration error, c = trim
        if trim: c = float(np.clip(c - KI_TRIM * x[0] * dt, -0.05, 0.05))   # see the sketch: trims until theta settles at 0
        dd_raw = 0.0 if d_prev is None else (d_meas - d_prev) / dt
        d_prev = d_meas
        dd_f += (dt / (TAU + dt)) * (dd_raw - dd_f)
        u = float(np.clip(-(Kd @ np.array([x[0], d_meas, x[2], dd_f]))[0], -U_MAX, U_MAX))
        for _ in range(10):  # plant integration, previous command applied (1-sample delay)
            h = dt / 10
            a = x[1] + np.pi
            th_dd = u_prev if abs(x[2] + u_prev * h) < V_MAX else 0.0
            a_dd = -2 * zeta * w_n * x[3] - w_n**2 * np.sin(a) + s * K_plant * np.cos(a) * th_dd
            x = x + h * np.array([x[2], x[3], th_dd, a_dd])
        u_prev = u
        log.append((k * dt, *x, u))
        if abs(x[1]) > np.radians(45): break
    return np.array(log)

for f, b, tr in ((1.0, 0, True), (0.8, 0, True), (1.2, 0, True), (1.0, np.radians(1), False), (1.0, np.radians(1), True)):
    log = simulate(K * f, bias=b, trim=tr, T=8.0 if b else 3.0)
    T = 8.0 if b else 3.0
    ok = len(log) >= int(T / dt) - 1
    last = log[-int(1 / dt):]   # last second: residual oscillation (encoder quantisation + derivative noise)
    print(f"sim K x{f:.1f}, upright bias {np.degrees(b):.0f} deg, trim {tr}: {f'balanced {T:.0f} s' if ok else 'FELL'} | max |theta| {np.degrees(abs(log[:,1]).max()):.1f} deg, "
          f"max |u| {abs(log[:,5]).max():.0f} rad/s^2 | last 1 s: d rms {np.degrees(np.sqrt((last[:,2]**2).mean())):.2f} deg, "
          f"theta range {np.degrees(last[:,1].min()):.1f}..{np.degrees(last[:,1].max()):.1f} deg, |u| rms {np.sqrt((last[:,5]**2).mean()):.0f} rad/s^2")

print("\nPaste into balance.ino:")
print("const float K_THETA = %.4f, K_ALPHA = %.4f, K_THETA_DOT = %.4f, K_ALPHA_DOT = %.4f;" % tuple(Kd.ravel()))
