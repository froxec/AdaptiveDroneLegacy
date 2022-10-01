from symbol import lambdef
import sympy as sym
import numpy as np
from numpy import deg2rad
quad_parameters = {
    'm': 1.025,
    'g': 9.81,
    'Kt': 3.122e-6,
    'Kd': 4.759e-9,
    'Ix': 0.12,
    'Iy': 0.12,
    'Iz': 0.048,
    'l': 0.27,
    'arm_angle': deg2rad(45) # X-configuration, angle between arm 2 or 4 and x axis
}
pendulum_parameters = {
    'm': 1,
    'l': 0.2,
    'g': 9.81,
    'r': 0.1,
    'Cd': 0.47,
    'ro': 1.23
}
def Jacobian(vars, funs):
    J = sym.zeros(len(funs),len(vars))
    for i, fi in enumerate(funs):
        for j, s in enumerate(vars):
            J[i, j] = sym.diff(fi, s)
    return J
    
def printJacobian(J):
    for i in range(J.shape[0]):
        print("fun", i, ": ", J[i, :])
    return 0
state_vars_str = 'x ,y, z, Vx, Vy, Vz, phi, theta, psi, phi_dot, theta_dot, psi_dot' #, alpha, alpha_dot, betha, betha_dot
input_vars_str = 'u1, u2, u3, u4'
parameters_str = 'm_q, g, Ix, Iy, Iz, Kt, Kd, l_q, arm_angle' # m_p, l_p
state_vars = sym.sympify(state_vars_str)
input_vars = sym.sympify(input_vars_str)
parameters = sym.sympify(parameters_str)
parameters = {
    'm_q': parameters[0],
    'g': parameters[1],
    'Ix': parameters[2],
    'Iy': parameters[3],
    'Iz': parameters[4],
    'Kt': parameters[5],
    'Kd': parameters[6],
    'l_q': parameters[7], 
    'arm_angle': parameters[8]
    # 'm_p': parameters[9],
    # 'l_p': parameters[10]
}
c = {
    'phi': sym.cos(state_vars[6]),
    'theta': sym.cos(state_vars[7]),
    'psi': sym.cos(state_vars[8])
    # 'alpha': sym.cos(state_vars[12]),
    # 'betha': sym.cos(state_vars[14])
}
s = {
    'phi': sym.sin(state_vars[6]),
    'theta': sym.sin(state_vars[7]),
    'psi': sym.sin(state_vars[8])
    # 'alpha': sym.sin(state_vars[12]),
    # 'betha': sym.sin(state_vars[14])
}

f1 = parameters['Kt']*input_vars[0]**2
f2 = parameters['Kt']*input_vars[1]**2
f3 = parameters['Kt']*input_vars[2]**2
f4 = parameters['Kt']*input_vars[3]**2

f = f1 + f2 + f3 + f4

m1 = parameters['Kd']*input_vars[0]**2
m2 = parameters['Kd']*input_vars[1]**2
m3 = parameters['Kd']*input_vars[2]**2
m4 = parameters['Kd']*input_vars[3]**2

t1 = (f1 + f2 - f3 - f4)*parameters['l_q']*sym.cos(parameters['arm_angle'])
t2 = (f2 + f4 - f1 - f3)*parameters['l_q']*sym.sin(parameters['arm_angle'])
t3 = -m1 + m2 + m3 - m4

# tension_force_x = -fun4*s['alpha']*c['betha'] 
# tension_force_y = 
# tension_force_z = 
fun1 = state_vars[3]
fun2 = state_vars[4]
fun3 = state_vars[5]
fun4 = (1/parameters['m_q'])*f*(c['phi']*s['theta']*c['psi'] + s['phi']*s['psi'])
fun5 = (1/parameters['m_q'])*f*(c['phi']*s['theta']*s['psi'] - s['phi']*c['psi'])
fun6 = (1/parameters['m_q'])*(f*(c['phi']*c['theta']) - parameters['m_q']*parameters['g'])
fun7 = state_vars[9]
fun8 = state_vars[10]
fun9 = state_vars[11]
fun10 = (1/parameters['Ix'])*(t1 - (state_vars[10]*state_vars[11]*parameters['Iz'] - state_vars[11]*state_vars[10]*parameters['Iy']))
fun11 = (1/parameters['Iy'])*(t2 - (state_vars[11]*state_vars[9]*parameters['Ix'] - state_vars[9]*state_vars[11]*parameters['Iz']))
fun12 = (1/parameters['Iz'])*(t3 - (state_vars[9]*state_vars[10]*parameters['Iy'] - state_vars[10]*state_vars[9]*parameters['Ix']))
# fun13 = state_vars[13]
# fun14 = (1/(parameters['m_p']*parameters['l_p']**2))*(-c['alpha']*s['betha']*fun4 - s['alpha']*c['betha']*(parameters['m_p']*parameters['g'] + fun6))
# fun15 = state_vars[15]
# fun16 = (1/(parameters['m_p']*parameters['l_p']**2))*(c['alpha']*s['betha']*(parameters['m_p']*parameters['g'] + fun6)+c['alpha']*c['betha']*fun5)
funs = [fun1, fun2, fun3, fun4, fun5, fun6, fun7, fun8, fun9, fun10, fun11, fun12] # , fun13, fun14, fun15, fun16
state_values = np.zeros(12)
input_values = np.zeros(4)
quad_parameter_values = [quad_parameters[key] for key in ('m', 'g', 'Ix', 'Iy', 'Iz', 'Kt', 'Kd', 'l', 'arm_angle')]
load_parameter_values =[pendulum_parameters[key] for key in ('m', 'l')] 
substitutions_state = [(state, value) for state, value in zip(state_vars, state_values)]
substitutions_input = [(input, value) for input, value in zip(input_vars, input_values)]
substitutions_params_quad = [(param, value) for param, value in zip(parameters, quad_parameter_values)]
A = Jacobian(state_vars, funs)
printJacobian(A)
B = Jacobian(input_vars, funs)
printJacobian(B)
# A = A.subs(substitutions_params_quad)
# A = A.subs(substitutions_state)
# A = A.subs(substitutions_input)
# print(A.eigenvals())

# ## calculation of critical points
# critical_points = sym.solve(c['phi']*s['theta']*c['psi'] + s['phi']*s['psi'], state_vars[6:9])
# print(critical_points)