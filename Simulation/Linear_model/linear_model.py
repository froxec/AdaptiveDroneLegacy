import numpy as np
from numpy import linspace
from math import cos, sin
from ModelsFactory.model_parameters import quad_parameters
from Simulation.plots import plotTrajectory
from numpy.linalg import eig

##Linearized quadcopter model
def linearModelEulerMethod(A, B, delta_x, delta_u, dt):
    return ((np.matmul(A, delta_x) + np.matmul(B, delta_u))*dt + delta_x).reshape((12))
class linearizedModel():
    def __init__(self, equlibrium_state, equilibrium_input):
        self.Kt = quad_parameters['Kt']
        self.Kd = quad_parameters['Kd']
        self.m_q = quad_parameters['m']
        self.l_q = quad_parameters['l']
        self.arm_angle = quad_parameters['arm_angle']
        self.Ix = quad_parameters['I'].item(0)
        self.Iy = quad_parameters['I'].item(1)
        self.Iz = quad_parameters['I'].item(2)
        phi = equlibrium_state.item(6)
        theta = equlibrium_state.item(7)
        psi = equlibrium_state.item(8)
        phi_dot = equlibrium_state.item(9)
        theta_dot = equlibrium_state.item(10)
        psi_dot = equlibrium_state.item(11)
        u1 = equilibrium_input.item(0)
        u2 = equilibrium_input.item(1)
        u3 = equilibrium_input.item(2)
        u4 = equilibrium_input.item(3)
        # ordinary indexing was throwing ragged nested sequences warning
        self.A = np.array([[0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, (-sin(phi)*sin(theta)*cos(psi) + sin(psi)*cos(phi))*(self.Kt*u1**2 + self.Kt*u2**2 + self.Kt*u3**2 + self.Kt*u4**2)/self.m_q, (self.Kt*u1**2 + self.Kt*u2**2 + self.Kt*u3**2 + self.Kt*u4**2)*cos(phi)*cos(psi)*cos(theta)/self.m_q, (sin(phi)*cos(psi) - sin(psi)*sin(theta)*cos(phi))*(self.Kt*u1**2 + self.Kt*u2**2 + self.Kt*u3**2 + self.Kt*u4**2)/self.m_q, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, (-sin(phi)*sin(psi)*sin(theta) - cos(phi)*cos(psi))*(self.Kt*u1**2 + self.Kt*u2**2 + self.Kt*u3**2 + self.Kt*u4**2)/self.m_q, (self.Kt*u1**2 + self.Kt*u2**2 + self.Kt*u3**2 + self.Kt*u4**2)*sin(psi)*cos(phi)*cos(theta)/self.m_q, (sin(phi)*sin(psi) + sin(theta)*cos(phi)*cos(psi))*(self.Kt*u1**2 + self.Kt*u2**2 + self.Kt*u3**2 + self.Kt*u4**2)/self.m_q, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, -(self.Kt*u1**2 + self.Kt*u2**2 + self.Kt*u3**2 + self.Kt*u4**2)*sin(phi)*cos(theta)/self.m_q, -(self.Kt*u1**2 + self.Kt*u2**2 + self.Kt*u3**2 + self.Kt*u4**2)*sin(theta)*cos(phi)/self.m_q, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, (self.Iy*psi_dot - self.Iz*psi_dot)/self.Ix, (self.Iy*theta_dot - self.Iz*theta_dot)/self.Ix],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, (-self.Ix*psi_dot + self.Iz*psi_dot)/self.Iy, 0, (-self.Ix*phi_dot + self.Iz*phi_dot)/self.Iy],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, (self.Ix*theta_dot - self.Iy*theta_dot)/self.Iz, (self.Ix*phi_dot - self.Iy*phi_dot)/self.Iz, 0]])
        self.B = np.array([[0, 0, 0, 0],
                           [0, 0, 0, 0],
                           [0, 0, 0, 0],
                           [2*self.Kt*u1*(sin(phi)*sin(psi) + sin(theta)*cos(phi)*cos(psi))/self.m_q, 2*self.Kt*u2*(sin(phi)*sin(psi) + sin(theta)*cos(phi)*cos(psi))/self.m_q, 2*self.Kt*u3*(sin(phi)*sin(psi) + sin(theta)*cos(phi)*cos(psi))/self.m_q, 2*self.Kt*u4*(sin(phi)*sin(psi) + sin(theta)*cos(phi)*cos(psi))/self.m_q],
                           [2*self.Kt*u1*(-sin(phi)*cos(psi) + sin(psi)*sin(theta)*cos(phi))/self.m_q, 2*self.Kt*u2*(-sin(phi)*cos(psi) + sin(psi)*sin(theta)*cos(phi))/self.m_q, 2*self.Kt*u3*(-sin(phi)*cos(psi) + sin(psi)*sin(theta)*cos(phi))/self.m_q, 2*self.Kt*u4*(-sin(phi)*cos(psi) + sin(psi)*sin(theta)*cos(phi))/self.m_q],
                           [2*self.Kt*u1*cos(phi)*cos(theta)/self.m_q, 2*self.Kt*u2*cos(phi)*cos(theta)/self.m_q, 2*self.Kt*u3*cos(phi)*cos(theta)/self.m_q, 2*self.Kt*u4*cos(phi)*cos(theta)/self.m_q],
                           [0, 0, 0, 0],
                           [0, 0, 0, 0],
                           [0, 0, 0, 0],
                           [2*self.Kt*self.l_q*u1*cos(self.arm_angle)/self.Ix, 2*self.Kt*self.l_q*u2*cos(self.arm_angle)/self.Ix, -2*self.Kt*self.l_q*u3*cos(self.arm_angle)/self.Ix, -2*self.Kt*self.l_q*u4*cos(self.arm_angle)/self.Ix],
                           [-2*self.Kt*self.l_q*u1*sin(self.arm_angle)/self.Iy, 2*self.Kt*self.l_q*u2*sin(self.arm_angle)/self.Iy, -2*self.Kt*self.l_q*u3*sin(self.arm_angle)/self.Iy, 2*self.Kt*self.l_q*u4*sin(self.arm_angle)/self.Iy],
                           [-2*self.Kd*u1/self.Iz, 2*self.Kd*u2/self.Iz, 2*self.Kd*u3/self.Iz, -2*self.Kd*u4/self.Iz]])
    def __call__(self, equlibrium_state, equilibrium_input):
        self.Kt = quad_parameters['Kt']
        self.Kd = quad_parameters['Kd']
        self.m_q = quad_parameters['m']
        self.l_q = quad_parameters['l']
        self.arm_angle = quad_parameters['arm_angle']
        self.Ix = quad_parameters['I'].item(0)
        self.Iy = quad_parameters['I'].item(1)
        self.Iz = quad_parameters['I'].item(2)
        phi = equlibrium_state.item(6)
        theta = equlibrium_state.item(7)
        psi = equlibrium_state.item(8)
        phi_dot = equlibrium_state.item(9)
        theta_dot = equlibrium_state.item(10)
        psi_dot = equlibrium_state.item(11)
        u1 = equilibrium_input.item(0)
        u2 = equilibrium_input.item(1)
        u3 = equilibrium_input.item(2)
        u4 = equilibrium_input.item(3)
        self.A = np.array([[0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, (-sin(phi)*sin(theta)*cos(psi) + sin(psi)*cos(phi))*(self.Kt*u1**2 + self.Kt*u2**2 + self.Kt*u3**2 + self.Kt*u4**2)/self.m_q, (self.Kt*u1**2 + self.Kt*u2**2 + self.Kt*u3**2 + self.Kt*u4**2)*cos(phi)*cos(psi)*cos(theta)/self.m_q, (sin(phi)*cos(psi) - sin(psi)*sin(theta)*cos(phi))*(self.Kt*u1**2 + self.Kt*u2**2 + self.Kt*u3**2 + self.Kt*u4**2)/self.m_q, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, (-sin(phi)*sin(psi)*sin(theta) - cos(phi)*cos(psi))*(self.Kt*u1**2 + self.Kt*u2**2 + self.Kt*u3**2 + self.Kt*u4**2)/self.m_q, (self.Kt*u1**2 + self.Kt*u2**2 + self.Kt*u3**2 + self.Kt*u4**2)*sin(psi)*cos(phi)*cos(theta)/self.m_q, (sin(phi)*sin(psi) + sin(theta)*cos(phi)*cos(psi))*(self.Kt*u1**2 + self.Kt*u2**2 + self.Kt*u3**2 + self.Kt*u4**2)/self.m_q, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, -(self.Kt*u1**2 + self.Kt*u2**2 + self.Kt*u3**2 + self.Kt*u4**2)*sin(phi)*cos(theta)/self.m_q, -(self.Kt*u1**2 + self.Kt*u2**2 + self.Kt*u3**2 + self.Kt*u4**2)*sin(theta)*cos(phi)/self.m_q, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, (self.Iy*psi_dot - self.Iz*psi_dot)/self.Ix, (self.Iy*theta_dot - self.Iz*theta_dot)/self.Ix],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, (-self.Ix*psi_dot + self.Iz*psi_dot)/self.Iy, 0, (-self.Ix*phi_dot + self.Iz*phi_dot)/self.Iy],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, (self.Ix*theta_dot - self.Iy*theta_dot)/self.Iz, (self.Ix*phi_dot - self.Iy*phi_dot)/self.Iz, 0]])
        self.B = np.array([[0, 0, 0, 0],
                [0, 0, 0, 0],
                [0, 0, 0, 0],
                [2*self.Kt*u1*(sin(phi)*sin(psi) + sin(theta)*cos(phi)*cos(psi))/self.m_q, 2*self.Kt*u2*(sin(phi)*sin(psi) + sin(theta)*cos(phi)*cos(psi))/self.m_q, 2*self.Kt*u3*(sin(phi)*sin(psi) + sin(theta)*cos(phi)*cos(psi))/self.m_q, 2*self.Kt*u4*(sin(phi)*sin(psi) + sin(theta)*cos(phi)*cos(psi))/self.m_q],
                [2*self.Kt*u1*(-sin(phi)*cos(psi) + sin(psi)*sin(theta)*cos(phi))/self.m_q, 2*self.Kt*u2*(-sin(phi)*cos(psi) + sin(psi)*sin(theta)*cos(phi))/self.m_q, 2*self.Kt*u3*(-sin(phi)*cos(psi) + sin(psi)*sin(theta)*cos(phi))/self.m_q, 2*self.Kt*u4*(-sin(phi)*cos(psi) + sin(psi)*sin(theta)*cos(phi))/self.m_q],
                [2*self.Kt*u1*cos(phi)*cos(theta)/self.m_q, 2*self.Kt*u2*cos(phi)*cos(theta)/self.m_q, 2*self.Kt*u3*cos(phi)*cos(theta)/self.m_q, 2*self.Kt*u4*cos(phi)*cos(theta)/self.m_q],
                [0, 0, 0, 0],
                [0, 0, 0, 0],
                [0, 0, 0, 0],
                [2*self.Kt*self.l_q*u1*cos(self.arm_angle)/self.Ix, 2*self.Kt*self.l_q*u2*cos(self.arm_angle)/self.Ix, -2*self.Kt*self.l_q*u3*cos(self.arm_angle)/self.Ix, -2*self.Kt*self.l_q*u4*cos(self.arm_angle)/self.Ix],
                [-2*self.Kt*self.l_q*u1*sin(self.arm_angle)/self.Iy, 2*self.Kt*self.l_q*u2*sin(self.arm_angle)/self.Iy, -2*self.Kt*self.l_q*u3*sin(self.arm_angle)/self.Iy, 2*self.Kt*self.l_q*u4*sin(self.arm_angle)/self.Iy],
                [-2*self.Kd*u1/self.Iz, 2*self.Kd*u2/self.Iz, 2*self.Kd*u3/self.Iz, -2*self.Kd*u4/self.Iz]])

u_val_eq = np.sqrt((quad_parameters['m']*quad_parameters['g']/4)/quad_parameters['Kt'])
equilibrium_state = np.zeros((12, 1))
equilibrium_input = np.full((4, 1), u_val_eq)  # F = m*g
model = linearizedModel(equilibrium_state, equilibrium_input)
## linear_model validation
dt = 0.1
t = np.arange(0, 100, dt)
samples = t.size
state_trajectory = np.zeros((samples, 12))
for i in range(samples):
    if i == 0:
        continue
    state_trajectory[i] = linearModelEulerMethod(model.A, model.B, state_trajectory[i - 1].reshape((12, 1)), np.array([1, 0, 0, 1]).reshape((4, 1)), dt)

plotTrajectory(t, state_trajectory.transpose(), 4, 3)
w, v = eig(model.A)
print(w)