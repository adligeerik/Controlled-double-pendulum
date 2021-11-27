from sympy.interactive.printing import init_printing
from sympy.printing.latex import latex

import sympy as sp
from sympy import sin, cos, Function

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

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

#Solve the d/dt(dL/dq_dot) part of the Lagrangeian
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

# print(latex(theta_ddot_1))
# print(latex(theta_ddot_2))

# Change variables
theta1, theta2, omega1, omega2, omega1_dot, omega2_dot = sp.symbols('theta1 theta2 omega1 omega2 omega1_dot omega2_dot')
change = {theta_1(t): theta1, theta_2(t): theta2, sp.Derivative(theta_1(t), t): omega1, sp.Derivative(theta_2(t), t): omega2, sp.Derivative(theta_1(t), (t,2)): omega1_dot, sp.Derivative(theta_2(t), (t,2)): omega2_dot}
L_theta1 = L_theta1.subs(change)
L_theta2 = L_theta2.subs(change)

L_theta1 = sp.simplify(L_theta1)
L_theta2 = sp.simplify(L_theta2)

print(latex(L_theta1))
print(latex(L_theta2))

# Get the system on matrix form
A, B = sp.linear_eq_to_matrix([L_theta1, L_theta2], [omega1_dot, omega2_dot])
A_inv = sp.simplify(A.inv())
omega_dot_mts = sp.simplify(A_inv @ B)

sub_values = {g: 9.82, l_1: 1, l_2: 1, m_1: 1, m_2:1}
omega_dot_mts = omega_dot_mts.subs(sub_values)
eq_omega1_dot = omega_dot_mts[0]
eq_omega2_dot = omega_dot_mts[1]

dt = 0.001

x_dot = np.zeros(4)
# x0 = np.array([np.pi, np.pi, 0, 0])
x0 = np.array([0, 0, 0, 0])
x = x0

start = time.time()
x_out = np.array(x)

n = 4000
u = np.sin(np.linspace(0,n,n+1)/200)

for i in range(n):
    print(i)
    x_val = {theta1: x[0], theta2: x[1], omega1: x[2], omega2: x[3]}

    x_dot[0] = x_val[omega1]
    x_dot[1] = x_val[omega2]
    x_dot[2] = eq_omega1_dot.xreplace(x_val)
    x_dot[3] = eq_omega2_dot.xreplace(x_val) + 5*u[i]

    x = x + dt * x_dot
    x_out = np.vstack([x_out, x])

end = time.time()
print(end - start)

theta_1 = x_out[:,0]
theta_2 = x_out[:,1]

x2_pos = sub_values[l_2] * np.sin(theta_2)
y2_pos = sub_values[l_2] * np.cos(theta_2)

x1_pos = sub_values[l_1] * np.sin(theta_1) + x2_pos
y1_pos = sub_values[l_1] * np.cos(theta_1) + y2_pos

# plt.plot(theta_1, label='theta_1')
# plt.plot(theta_2, label='theta_2')
# plt.legend()
# plt.show()

fig = plt.figure(figsize=(5, 4))
ax = fig.add_subplot(autoscale_on=False, xlim=(-3, 3), ylim=(-3, 3))
line, = ax.plot([], [], 'o-', lw=2)

def animate(i):
    thisx = [x1_pos[i*20], x2_pos[i*20], 0]
    thisy = [y1_pos[i*20], y2_pos[i*20], 0]
    line.set_data(thisx, thisy)
    return line

ani = animation.FuncAnimation(fig, animate, int(len(x1_pos)/20), interval=dt, blit=False)
plt.show()
