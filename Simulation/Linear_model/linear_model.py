import numpy as np
from math import cos, sin
from model_parameters import quad_parameters
##Linearized quadcopter model

class linearizedModel():
    def __init__(self, equlibrium_state, equilibrium_input):
        self.Kt = quad_parameters['Kt']
        self.Kd = quad_parameters['Kd']
        self.m_q = quad_parameters['m_q']
        self.l_q = quad_parameters['l']
        self.arm_angle = quad_parameters['arm_angle']
        self.Ix = quad_parameters['I'][0]
        self.Iy = quad_parameters['I'][1]
        self.Iz = quad_parameters['I'][2]
        phi = equlibrium_state[6]
        theta = equlibrium_state[7]
        psi = equlibrium_state[8]
        phi_dot = equlibrium_state[9]
        theta_dot = equlibrium_state[10]
        psi_dot = equlibrium_state[11]
        u1 = equilibrium_input[0]
        u2 = equilibrium_input[1]
        u3 = equilibrium_input[2]
        u4 = equilibrium_input[3]
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
        self.m_q = quad_parameters['m_q']
        self.l_q = quad_parameters['l']
        self.arm_angle = quad_parameters['arm_angle']
        self.Ix = quad_parameters['I'][0]
        self.Iy = quad_parameters['I'][1]
        self.Iz = quad_parameters['I'][2]
        phi = equlibrium_state[6]
        theta = equlibrium_state[7]
        psi = equlibrium_state[8]
        phi_dot = equlibrium_state[9]
        theta_dot = equlibrium_state[10]
        psi_dot = equlibrium_state[11]
        u1 = equilibrium_input[0]
        u2 = equilibrium_input[1]
        u3 = equilibrium_input[2]
        u4 = equilibrium_input[3]
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
u_val_eq = np.sqrt((quad_parameters['m_q']*quad_parameters['g']/4)/quad_parameters['Kt'])
equilibrium_state = np.zeros((12))
equilibrium_input = np.full((12), u_val_eq) # F = m*g
model = linearizedModel(equilibrium_state, equilibrium_input)