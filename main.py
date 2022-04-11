from sympy.interactive.printing import init_printing
from sympy.printing.latex import latex

import sympy as sp
from sympy import sin, cos, Function
from scipy.linalg import solve_continuous_are, inv, eig
from ekf import ExtendedKalmanFilter as EKF
from filterpy.common import Q_discrete_white_noise

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

from control import lqr, pzmap, StateSpace

# Create all parameters
t = sp.symbols('t')
g = sp.symbols('g')

l_1 = sp.symbols('l_1')
l_2 = sp.symbols('l_2')

m_1 = sp.symbols('m_1')
m_2 = sp.symbols('m_2')

# Create theta_i as a function of time
theta_1 = Function('theta_1')
theta_2 = Function('theta_2')

# The equations for position in cartesian coordinates
x_2 = l_2*sin(theta_2(t))
y_2 = l_2*cos(theta_2(t))

x_1 = l_1*sin(theta_1(t)) + x_2
y_1 = l_1*cos(theta_1(t)) + y_2

# Find the velocities by differential
x_dot_1 = sp.diff(x_1, t)
y_dot_1 = sp.diff(y_1, t)
x_dot_1 = sp.simplify(x_dot_1)
y_dot_1 = sp.simplify(y_dot_1)

x_dot_2 = sp.diff(x_2, t)
y_dot_2 = sp.diff(y_2, t)
x_dot_2 = sp.simplify(x_dot_2)
y_dot_2 = sp.simplify(y_dot_2)

# Define the potential energy part of the Lagrangeian
P_1 = m_1*g*y_1
P_2 = m_2*g*y_2
V = P_1 + P_2

# Define the momentum part of the Lagrangeian
E_1 = m_1*(x_dot_1**2 + y_dot_1**2)/2
E_2 = m_2*(x_dot_2**2 + y_dot_2**2)/2
T = E_1 + E_2
T = sp.simplify(T)

# Set up the full Lagrangeian
L = T - V
L = sp.simplify(L)

# Solve the d/dt(dL/dq_dot) part of the Lagrangeian
d_1_1 = sp.diff(sp.diff(L, 'Derivative(theta_1(t), t)'), 't')
d_2_1 = sp.diff(sp.diff(L, 'Derivative(theta_2(t), t)'), 't')

# Solve the dL/dq part of the Lagrangeian
d_1_2 = sp.diff(L, 'theta_1(t)')
d_2_2 = sp.diff(L, 'theta_2(t)')

L_theta1 = d_1_1 - d_1_2
L_theta2 = d_2_1 - d_2_2

# Solve for d^2/dt^2 of theta_i
theta_ddot_1 = sp.solve(L_theta1, 'Derivative(theta_1(t), (t,2))')
theta_ddot_1 = sp.simplify(theta_ddot_1[0])

theta_ddot_2 = sp.solve(L_theta2, 'Derivative(theta_2(t), (t,2))')
theta_ddot_2 = sp.simplify(theta_ddot_2[0])

# Change variables
theta1, theta2, omega1, omega2, omega1_dot, omega2_dot = sp.symbols('theta1 theta2 omega1 omega2 omega1_dot omega2_dot')
change = {theta_1(t): theta1, theta_2(t): theta2, sp.Derivative(theta_1(t), t): omega1, sp.Derivative(theta_2(t), t): omega2, sp.Derivative(theta_1(t), (t,2)): omega1_dot, sp.Derivative(theta_2(t), (t,2)): omega2_dot}
L_theta1 = L_theta1.subs(change)
L_theta2 = L_theta2.subs(change)

L_theta1 = sp.simplify(L_theta1)
L_theta2 = sp.simplify(L_theta2)

# Get the system on matrix form
M, F = sp.linear_eq_to_matrix([L_theta1, L_theta2], [omega1_dot, omega2_dot])
A_cont = sp.simplify(sp.simplify(M.inv()) @ F)

# Add omega 1 and 2 to compleate the system
A_cont = A_cont.row_insert(0, sp.Matrix([omega1]))
A_cont = A_cont.row_insert(1, sp.Matrix([omega2]))

# Linearize (calc the jacobian)
A_lin = A_cont.jacobian(sp.Matrix([theta1, theta2, omega1, omega2]))
B_lin = sp.Matrix([[0],[0],[0],[1]])
C_lin = sp.Matrix([[1, 0, 0, 0],
                   [0, 1, 0, 0]])
# C_lin = sp.Matrix([[0, 1, 0, 0]])
D_lin= sp.Matrix([0])

# Substitute parameters for values
sub_values = {g: 9.82, l_1: 2, l_2: 2, m_1: 1, m_2: 1}
A_lin = A_lin.subs(sub_values)
omega_dot_mts = A_cont.subs(sub_values)
eq_omega1_dot = omega_dot_mts[2]
eq_omega2_dot = omega_dot_mts[3]

dt = 0.001
x_dot = np.zeros(4)
# x0 = np.array([np.pi, np.pi, 0, 0])
x0 = np.array([0, 0, 0, 0])
# x0 = np.array([0, 0, 0, 0])
# Initialize the kalman filter
input = sp.symbols('u')
dx = sp.Matrix([theta1, theta2, omega1, omega2])
A_non_lin = sp.Matrix([omega1, omega2, eq_omega1_dot, eq_omega2_dot + input])
R_kf = np.diag([0.5, 0.5])
P_kf = np.diag([0.01, 0.01, 0.1, 0.1])
Q_kf = Q_discrete_white_noise(dim=len(dx), dt=dt, var=0.13)
ekf = EKF(x0 = x0, P = P_kf, dt = dt, Q = Q_kf, R = R_kf, A_non_lin = A_non_lin, dx = dx, C =  np.array(C_lin).astype(np.float64), input_symbol = input)
x_out_hat = np.array(x0.reshape(4, 1))

# Linearize around stationary point
stationary_point = {theta1: 0, theta2: 0, omega1: 0, omega2: 0}
A_stat = A_lin.subs(stationary_point)

# Convert to numpy array from Matrix (sympy)
A_stat = np.array(A_stat).astype(np.float64)
B_stat = np.array(B_lin).astype(np.float64)
C_stat = np.array(C_lin).astype(np.float64)
D_stat = np.array(D_lin).astype(np.float64)

# Calculate if the system is controllable
AB = A_lin @ B_lin
AAB = A_lin @ AB
AAAB = A_lin @ AAB
Controllability = sp.Matrix.hstack(B_lin, AB, AAB, AAAB)
cont_val = Controllability.subs(stationary_point).evalf().det()
if np.abs(cont_val) < 0.01:
    raise "Not controllable"

# Check observability of system
CA = C_lin @ A_lin
CAA = CA @ A_lin
CAAA = CAA @ A_lin
Observability = sp.Matrix.vstack(C_lin, CA, CAA, CAAA)
Observability = Observability.subs(stationary_point).evalf()
Observability = np.array(Observability).astype(np.float64) # Convert to np array
rank_observ = np.linalg.matrix_rank(Observability)
if rank_observ != 4:
    raise "System is not observable"

# Calc the continuous LQR controller
Q = np.diag([1, 100, 1, 1])
R = 1
P = solve_continuous_are(a = A_stat, b = B_stat, q = Q, r = R)
K = np.matrix((1/R)*(B_stat.T @ P))
K = np.array(K).astype(np.float64) # Convert to np array

# Convert sympy expression to lambda function to decrease computation cost
symbols_omega1 = list(eq_omega1_dot.free_symbols)
l_dot_omega1 = sp.lambdify(symbols_omega1, eq_omega1_dot, "numpy")
symbols_omega2 = list(eq_omega2_dot.free_symbols)
l_dot_omega2 = sp.lambdify(symbols_omega2, eq_omega2_dot, "numpy")

# Plot pole zero map
# sys = StateSpace(A_stat, B_stat, C_stat, D_stat)
# A_closed = A_stat - B_stat @ K
# e, f = eig(A_stat)
# w, v = eig(A_closed)

x0 = np.array([0.1, 0.1, 0, 0])
x = x0

n = 2000

start = time.time()
x_out = np.array(x)

disturbance = np.zeros(n)
# disturbance += np.random.normal(0, 1, size=n)
# disturbance[100] = 100
disturbance[200:400] = 0
# disturbance[2000:2400] = -0.5
stop_u = np.ones(n)
# stop_u[3000:-1] = 0
u_t = []
for i in range(n):
    print(i)
    x_val = {"theta1": x[0], "theta2": x[1], "omega1": x[2], "omega2": x[3]}
    u = -K @ x
    u_t.append(u)
    x_dot[0] = x_val["omega1"]
    x_dot[1] = x_val["omega2"]
    x_dot[2] = l_dot_omega1(**x_val) + disturbance[i]
    x_dot[3] = l_dot_omega2(**x_val) + u * stop_u[i]

    x = x + dt * x_dot
    x_out = np.vstack([x_out, x])

    ekf.predict(u=u[0])
    ekf.update(z=np.array([x[0], x[1]]).reshape(2, 1))
    # ekf.update(z=x[1])
    x_out_hat = np.hstack([x_out_hat, ekf.x_hat])

end = time.time()
print(end - start)

theta_1 = x_out[:,0]
theta_2 = x_out[:,1]
omega_1 = x_out[:,2]
omega_2 = x_out[:,3]

# Calculate the kartesioan coordinates from the angles
x2_pos = sub_values[l_2] * np.sin(theta_2)
y2_pos = sub_values[l_2] * np.cos(theta_2)

x1_pos = sub_values[l_1] * np.sin(theta_1) + x2_pos
y1_pos = sub_values[l_1] * np.cos(theta_1) + y2_pos

# # Plot poles for the open and closed loop system
# fig_pz = plt.figure()
# plt.scatter(e.real, e.imag, label='Open system poles')
# plt.scatter(w.real, w.imag, label='Closed system poles')
# plt.grid()
# plt.legend()

# Plot the states and control signal
fig_sys = plt.figure()
plt.subplot(3, 1, 1)
plt.plot(x_out_hat[0,:], '--r', label='Estimated theta_1')
plt.plot(x_out_hat[1,:], '--b', label='Estimated theta_2')
plt.plot(theta_1, label='theta_1')
plt.plot(theta_2, label='theta_2')
plt.legend()
plt.grid()

plt.subplot(3, 1, 2)
plt.plot(x_out_hat[2,:], '--r', label='Estimated omega_1')
plt.plot(x_out_hat[3,:], '--b', label='Estimated omega_2')
plt.plot(omega_1, label='omega_1')
plt.plot(omega_2, label='omega_2')
plt.legend()
plt.grid()

plt.subplot(3, 1, 3)
plt.plot(u_t, label='Control signal')
plt.plot(disturbance, label='disturbance')
plt.legend()
plt.grid()

fig = plt.figure(figsize=(5, 5))
ax = fig.add_subplot(autoscale_on=False, xlim=(-2.5, 2.5), ylim=(-0.5, 4.5))
line, = ax.plot([], [], 'o-', lw=2)
plt.grid()


def animate(i):
    thisx = [x1_pos[i*20], x2_pos[i*20], 0]
    thisy = [y1_pos[i*20], y2_pos[i*20], 0]
    line.set_data(thisx, thisy)
    return line


# Animate the system
ani = animation.FuncAnimation(fig, animate, int(len(x1_pos)/20), interval=dt, blit=False)

# Show all plt objects
plt.show()
