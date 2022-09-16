import sympy as sym

state_vars_str = 'x ,y, z, Vx, Vy, Vz, phi, theta, psi, phi_dot, theta_dot, psi_dot'
input_vars_str = 'u1, u2, u3, u4'
parameters_str = 'm, g, Ix, Iy, Iz, Kt, Kd'
state_vars = sym.sympify(state_vars_str)
input_vars = sym.sympify(input_vars_str)
parameters = sym.sympify(parameters_str)
parameters = {
    'm': parameters[0],
    'g': parameters[1],
    'Ix': parameters[2],
    'Iy': parameters[3],
    'Iz': parameters[4],
    'Kt': parameters[5],
    'Kd': parameters[6],
}
c = {
    'phi': sym.cos(state_vars[6]),
    'theta': sym.cos(state_vars[7]),
    'psi': sym.cos(state_vars[8])
}
s = {
    'phi': sym.sin(state_vars[6]),
    'theta': sym.sin(state_vars[7]),
    'psi': sym.sin(state_vars[8])
}
f1 = parameters['Kt']*input_vars[0]**2
f2 = parameters['Kt']*input_vars[1]**2
f3 = parameters['Kt']*input_vars[2]**2
f4 = parameters['Kt']*input_vars[3]**2
f = f1 + f2 + f2 + f4
fun1 = state_vars[3]
fun2 = state_vars[4]
fun3 = state_vars[5]
fun4 = (1/parameters['m'])*f*(c['phi']*s['theta']*c['psi'] + s['phi']*s['psi'])
fun5 = (1/parameters['m'])*f*(c['phi']*s['theta']*c['psi'] + s['phi']*s['psi'])
# fun5 = ''
# fun6 = ''
# fun7 = 'phi_dot'
# fun8 = 'theta_dot'
# fun9 = 'psi_dot'
# fun10 = ''
# fun11 = ''
# fun12 = ''
# fun = [fun1, fun2, fun3, fun4, fun5, fun6, fun7, fun8, fun9, fun10, fun11, fun12]
print(f1)