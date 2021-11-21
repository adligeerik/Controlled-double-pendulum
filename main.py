from sympy.interactive.printing import init_printing
from sympy.printing.latex import latex

import sympy as sp
from sympy import sin, cos, Function

t = sp.symbols('t')
g = sp.symbols('g')

l_1 = sp.symbols('l_1')
l_2 = sp.symbols('l_2')

m_1 = sp.symbols('m_1')
m_2 = sp.symbols('m_2')

theta_1 = Function('theta_1')
theta_2 = Function('theta_2')

x_1 = l_1*sin(theta_1(t)) + l_2*sin(theta_2(t))
y_1 = l_1*cos(theta_1(t)) + l_2*cos(theta_2(t))

x_2 = l_2*sin(theta_2(t))
y_2 = l_2*cos(theta_2(t))

P_1 = m_1*g*y_1
P_2 = m_2*g*y_2
V = P_1+P_2

x_dot_1 = sp.diff(x_1, t)
y_dot_1 = sp.diff(y_1, t)

x_dot_2 = sp.diff(x_2, t)
y_dot_2 = sp.diff(y_2, t)

E_1 = m_1*(x_dot_1**2 + y_dot_1**2)/2
E_2 = m_2*(x_dot_2**2 + y_dot_2**2)/2

T = E_1 - E_2

L = T - V

