from math import sin, cos
import numpy as np
from numpy import deg2rad
from model_parameters import pendulum_parameters
#X-configuration
#ccw is positive
#   cw  4  🢁🢁🢁  2 ccw
#         \    /  
#          \  /
#           \/->y
#           /\
#          /  \
#         /    \
#   ccw 3        1 cw



class quadcopterModel():
    def __init__(self, state0, quad_parameters):
        self.mass = quad_parameters['m']
        self.g = quad_parameters['g'] ## move to Environment class
        self.inertia = quad_parameters['I']
        self.arm_length = quad_parameters['l']
        self.arm_angle = quad_parameters['arm_angle']
        self.Kt = quad_parameters['Kt']
        self.Kd = quad_parameters['Kd']
        self.state_names = ('x', 'y', 'z',
                            'dx', 'dy', 'dz',
                            'roll', 'pitch', 'yaw',
                            'droll', 'dpitch', 'dyaw')
        self.state_dict = {key: state_value for key, state_value in zip(self.state_names, state0)}
        self.state = state0 if isinstance(state0, np.ndarray) else np.array(state0)
        self.load = loadPendulum(np.array[0, 0], pendulum_parameters, self)
        self.F = np.zeros((3))
        self.M = np.zeros((3))
        self.translational_accelerations = np.zeros((3))
    def updateStateDict(self):
        self.state_dict = {key: state_value for key, state_value in zip(self.state_names, self.state)}

    def transfomationMatrix(self):
        cR = cos(self.state[6])
        cP = cos(self.state[7])
        cY = cos(self.state[8])
        sR = sin(self.state[6])
        sP = sin(self.state[7])
        sY = sin(self.state[8])
        self.R = np.array([[cP*cY, sR*sP*cY - cR*sY, cR*sP*cY + sR*sY],
                    [cP*sY, sR*sP*sY - cR*cY, cR*sP*sY - sR*cY],
                    [-sP, sR*cP, cR*cP]], dtype=np.float32)
        return self.R

    def translationalMotion(self):
        f = sum(self.F)
        accelerations = np.zeros((3, 1), dtype=np.float32)
        bodyFrameThrust = np.array([[0], 
                                [0],
                                [f]], dtype=np.float32)
        referenceFrame = np.matmul(self.R, bodyFrameThrust) + np.array([[0],
                                                                [0],
                                                                [-self.mass*self.g]], dtype=np.float32)
        referenceFrame = referenceFrame + self.load.tension_forces
        accelerations = referenceFrame/self.mass
        accelerations = np.reshape(accelerations, 3)
        self.translational_accelerations = accelerations
        return accelerations

    def angularMotion(self):
        T = np.zeros(3)
        T[0] = (self.F[0] + self.F[1] - self.F[2] - self.F[3])*self.arm_length*cos(self.arm_angle)
        T[1] = (self.F[1] + self.F[3] - self.F[0] - self.F[2])*self.arm_length*sin(self.arm_angle)
        T[2] = self.M[1] + self.M[2] - self.M[0] - self.M[3]
        accelerations =  T - np.cross(self.state[9:12], np.multiply(self.inertia, self.state[9:12]))
        accelerations = np.divide(accelerations, self.inertia)
        return accelerations

    def inputToForces(self, omega):
        self.F = np.multiply(self.Kt, omega**2)
        return self.F

    def inputToMomentum(self, omega):
        self.M = np.multiply(self.Kd, omega**2)
        return self.M

    def model_ode(self, x, t):
        self.state = x if isinstance(x, np.ndarray) else np.array(x)
        self.updateStateDict()
        dstate = np.zeros(14)
        omega = np.array([2000, 898, 2000, 898])
        self.inputToForces(omega)
        self.inputToMomentum(omega)
        self.transfomationMatrix()
        dstate[0:3] = self.state[3:6]
        dstate[3:6] = self.translationalMotion()
        dstate[6:9] = self.state[9:12]
        dstate[9:12] = self.angularMotion()
        dstate[12:14] = loadPendulum.updateState()
        return dstate 

    def modelRT(self, u, deltaT):
        state = np.zeros(12)
        omega = np.array(u)
        self.inputToForces(omega)
        self.inputToMomentum(omega)
        self.transfomationMatrix()
        self.state[0:3] = self.state[3:6] * deltaT + self.state[0:3]
        state[3:6] = self.translationalMotion() * deltaT + self.state[3:6]
        state[6:9] = self.state[self.state_names[9:12]] * deltaT + self.state[6:9]
        state[9:12] = self.angularMotion() * deltaT + self.state[9:12]
        self.updateStateDict()
        if (self.state[2] < 0):
            self.state[2] = 0
        return self.state

    def __call__(self, mode, ):
        pass

class loadPendulum():
    ##  assumption -> force vector at quad CoG
    def __init__(self, state0, pendulum_parameters, quadcopter_object):
        self.mass = pendulum_parameters['m']
        self.length = pendulum_parameters['l']
        self.I = self.mass*(self.length**2)
        self.g = pendulum_parameters['g']
        self.state_names = ('alpha', 'beta')
        self.state_dict = {key: state_value for key, state_value in zip(self.state_names, state0)}
        self.state = state0 if isinstance(state0, np.ndarray) else np.array(state0)
        self.quadcopter = quadcopter_object
        self.tension_force = np.zeros((3)) 
    def __call__():
        pass
    def vectorProjection(vector, direction):
        #projects vector a on vector b
        return (np.dot(vector, direction)/np.dot(direction, direction))*direction
    def inertialForce(self, quad_acceleration):
        return -self.mass*quad_acceleration
    def angularMotion(self, net_force):
        direction_x = self.directionVectors()[0]
        direction_y = self.directionVectors()[1]
        torque_x = self.length*self.vectorProjection(net_force, direction_x)
        torque_y = self.length*self.vectorProjection(net_force, direction_y)
        acceleration_alpha = torque_y/self.I
        acceleration_beta = torque_x/self.I
        return np.array([acceleration_alpha, acceleration_beta])
    def tensionForce(self, load_direction, netForce):
        ## tension force acting at cog of quadcopter
        return self.vectorProjection(netForce, load_direction)
    def netForce(forcesList):
        netF = np.zeros((3))
        for force in forcesList:
            if isinstance(force, np.ndarray):
                netF += force
            else:
                raise("Each of the force should be ndarray of shape(3)")
        return netF
    def directionVectors(self):
        ## unit vector in direction from quadcopter COG to load COG
        alpha = self.state[0] ## alpha is rotation in y axis
        beta = self.state[1] ## beta is rotation in x axis
        r_x = np.array([cos(alpha), 0, -sin(alpha)])
        r_y = np.array([0, cos(beta), -sin(beta)])
        r_z = np.array([-sin(alpha)*cos(beta), cos(alpha)*sin(beta), -cos(alpha)*cos(beta)])
        return (r_x, r_y, r_z)
    def calculateForces(self):
        gravityForce = np.array([0, 0, -self.mass*self.g])
        inertialForce = self.inertialForces(self.quadcopter.translational_accelerations)
        return [gravityForce, inertialForce]
    def updateStateDict(self):
        self.state_dict = {key: state_value for key, state_value in zip(self.state_names, self.state)}
    def updateState(self, deltaT):
        forces = self.calculateForces()
        net_force = self.netForce(forces)
        self.tension_force = self.tensionForce(self.DirectionVector()[2], net_force)
        self.state = self.angularMotion(net_force)*deltaT + self.state
        self.updateStateDict()
        return self.state