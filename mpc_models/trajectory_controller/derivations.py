#!/usr/bin/python

import sympy as sym

# Rotation from body frame to stability frame
s_alpha = sym.rot_axis2(sym.Symbol('alpha')).T
# Rotation from stability frame to wind frame
s_beta = sym.rot_axis3(sym.Symbol('beta'))

print(s_beta)
print(s_alpha)

p, q, r = sym.symbols(['p', 'q', 'r'])
omega = sym.Matrix([[p, q, r]]).T

omega_w = s_beta * s_alpha * omega

sym.pprint(omega_w)

phi, theta, psi = sym.symbols(['phi', 'theta', 'psi'])
s_phi = sym.rot_axis1(phi)
s_theta = sym.rot_axis2(theta)
s_psi = sym.rot_axis3(psi)
Reb = s_phi * s_theta * s_psi

g0 = sym.Matrix([[0, 0, sym.Symbol('g0')]]).T

g = s_beta * s_alpha * Reb * g0
sym.pprint(g)