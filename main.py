from sympy.interactive.printing import init_printing
from sympy.printing.latex import latex

import sympy as sp
from sympy import sin, cos, Function

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
x_1 = l_1*sin(theta_1(t)) + l_2*sin(theta_2(t))
y_1 = l_1*cos(theta_1(t)) + l_2*cos(theta_2(t))

x_2 = l_2*sin(theta_2(t))
y_2 = l_2*cos(theta_2(t))

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
P_1 = -m_1*g*y_1
P_2 = -m_2*g*y_2
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

# Solve for d^2/dt^2 of theta_1
theta_ddot_1 = sp.solve(d_1_1 - d_1_2, 'Derivative(theta_1(t), (t,2))')
theta_ddot_1 = sp.simplify(theta_ddot_1[0])

theta_ddot_2 = sp.solve(d_2_1 - d_2_2, 'Derivative(theta_2(t), (t,2))')
theta_ddot_2 = sp.simplify(theta_ddot_2[0])

print(latex(theta_ddot_1))
print(latex(theta_ddot_2))
