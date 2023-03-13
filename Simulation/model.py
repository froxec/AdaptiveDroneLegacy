from math import sin, cos
import numpy as np
from numpy import deg2rad
import copy
#X-configuration
#ccw is positive
#   cw  4        2 ccw
#         \    /  
#          \  /
#           \/->y forward
#           /\
#          /  \
#         /    \
#   ccw 3  ⬇⬇  1 cw
# right-hand, ccw is positive, z: up, y: forward, x: right

def exchangeData(quadcopter_object, load_pendulum_object):
    load_pendulum_object.updateQuadStateRepresentation(quadcopter_object)
    quadcopter_object.updateTensionForce(load_pendulum_object)
    return True

def odeSystem(x, t, quadcopter_object, load_pendulum_object):
    #doesnt work ATM
    exchangeData(quadcopter_object, load_pendulum_object)
    quad_dstate = quadcopter_object.model_ode(x[0:12], t)
    load_dstate = load_pendulum_object.updateStateOde(x[12:16], t)
    dstate = np.concatenate([quad_dstate, load_dstate])
    return dstate

def system(u, deltaT, quadcopter_object, load_pendulum_object=None, solver='RK'):
    if load_pendulum_object==None:
        if solver == 'Euler':
            quadcopter_object.modelRT(u, deltaT)
        if solver == 'RK':
            RungeKutta4(deltaT, quadcopter_object, u)
        return quadcopter_object.state
    else:
        exchangeData(quadcopter_object, load_pendulum_object)
        if solver == 'Euler':
            quadcopter_object.modelRT(u, deltaT)
        if solver == 'RK':
            RungeKutta4(deltaT, quadcopter_object, u)
        #load_pendulum_object.updateState(deltaT)
        RungeKutta4(deltaT, load_pendulum_object)
        return np.concatenate([quadcopter_object.state, load_pendulum_object.state])

def RungeKutta4(deltaT, model_object, u=np.array([None])):
    model = copy.deepcopy(model_object) #might be bottleneck/make class
    state0 = model.state
    if u.any() == None:
        k1 = deltaT*model.updateStateOde(state0)
        k2 = deltaT*model.updateStateOde(state0 + 0.5*k1)
        k3 = deltaT*model.updateStateOde(state0 + 0.5*k2)
        k4 = deltaT*model.updateStateOde(state0 + k3)
    else:
        k1 = deltaT * model.updateStateOde(state0, u)
        k2 = deltaT * model.updateStateOde(state0 + 0.5 * k1, u)
        k3 = deltaT * model.updateStateOde(state0 + 0.5 * k2, u)
        k4 = deltaT * model.updateStateOde(state0 + k3, u)
    model_object.state = model_object.state + (1.0 / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
    return model_object.state

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
        self.state = state0 if isinstance(state0, np.ndarray) else np.array(state0, dtype=np.float32)
        self.tension_force = np.zeros((3), dtype=np.float32)
        self.F = np.zeros((4), dtype=np.float32)
        self.M = np.zeros((4), dtype=np.float32)
        self.translational_accelerations = np.zeros((3), dtype=np.float32)
        self.angular_accelerations = np.zeros((3), dtype=np.float32)

    def updateStateDict(self):
        #change - use existing structre
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
        # self.R = np.array([[cP * cY, sY * cP, sP],
        #               [sP * sR * cY - sY * cR, sP * sR * sY + cR * cY, -sR * cP],
        #               [-sP * cR * cY - sR * sY, -sP * sY * cR + sR * cY, cP * cR]], dtype=np.float32)
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
        referenceFrame = referenceFrame + self.tension_force.reshape((3, 1))
        accelerations = referenceFrame/self.mass
        accelerations = np.reshape(accelerations, 3)
        self.translational_accelerations = accelerations
        return accelerations

    def angularMotion(self):
        T = np.zeros((3), dtype=np.float32)
        T[0] = (self.F[0] + self.F[1] - self.F[2] - self.F[3])*self.arm_length*cos(self.arm_angle)
        T[1] = (self.F[1] + self.F[3] - self.F[0] - self.F[2])*self.arm_length*sin(self.arm_angle)
        T[2] = self.M[1] + self.M[2] - self.M[0] - self.M[3]
        accelerations =  T - np.cross(self.state[9:12], np.multiply(self.inertia, self.state[9:12]))
        accelerations = np.divide(accelerations, self.inertia)
        self.angular_accelerations = accelerations
        return accelerations

    def updateTensionForce(self, load_pendulum_object):
        self.tension_force = load_pendulum_object.tension_force
        return self.tension_force

    def inputToForces(self, omega):
        self.F = np.multiply(self.Kt, omega**2)
        return self.F

    def inputToMomentum(self, omega):
        self.M = np.multiply(self.Kd, omega**2)
        return self.M

    def updateStateOde(self, x, u):
        self.state = x if isinstance(x, np.ndarray) else np.array(x)
        self.updateStateDict()
        dstate = np.zeros(12)
        omega = u
        self.inputToForces(omega)
        self.inputToMomentum(omega)
        self.transfomationMatrix()
        dstate[0:3] = self.state[3:6]
        dstate[3:6] = self.translationalMotion()
        dstate[6:9] = self.state[9:12]
        dstate[9:12] = self.angularMotion()
        return dstate 

    def modelRT(self, u, deltaT):
        state = np.zeros(12)
        omega = np.array(u)
        self.inputToForces(omega)
        self.inputToMomentum(omega)
        self.transfomationMatrix()
        state[0:3] = self.state[3:6] * deltaT + self.state[0:3]
        state[3:6] = self.translationalMotion() * deltaT + self.state[3:6]
        state[6:9] = self.state[9:12] * deltaT + self.state[6:9]
        state[9:12] = self.angularMotion() * deltaT + self.state[9:12]
        self.state = state
        self.updateStateDict()
        if (self.state[2] < 0):
            self.state[2] = 0
        return self.state
    def update_parameters(self, parameters):
        parameters = copy.deepcopy(parameters)
        self.mass = parameters['m']
        self.g = parameters['g']  ## move to Environment class
        self.inertia = parameters['I']
        self.arm_length = parameters['l']
        self.arm_angle = parameters['arm_angle']
        self.Kt = parameters['Kt']
        self.Kd = parameters['Kd']
    def __call__(self, mode, ):
        pass

class loadPendulum():
    ##  assumption -> force vector at quad CoG
    def __init__(self, state0, pendulum_parameters, quad_acceleration, quad_speed):
        self.mass = pendulum_parameters['m']
        self.length = pendulum_parameters['l']
        self.I = self.mass*(self.length**2)
        self.g = pendulum_parameters['g']
        self.r = pendulum_parameters['r']
        self.Cd = pendulum_parameters['Cd']
        self.ro = pendulum_parameters['ro'] #move to environment class/parameters
        self.A = np.pi*(self.r**2)
        self.state_names = ('alpha', 'beta',
                            'dalpha', 'dbeta')
        self.state_dict = {key: state_value for key, state_value in zip(self.state_names, state0)}
        self.state = state0 if isinstance(state0, np.ndarray) else np.array(state0)
        self.quad_acceleration = quad_acceleration
        self.quad_speed = quad_speed
        self.tension_force = np.zeros((3)) 
        self.direction_vectors = ()
    def __call__():
        pass
    def vectorProjection(self, vector, direction):
        #projects vector a on vector b
        return (np.dot(vector, direction)/(np.dot(direction, direction)**2))*direction
    def inertialForce(self, quad_acceleration):
        return -self.mass*quad_acceleration
    def angularMotion(self, net_force):
        # direction_x = self.direction_vectors[0]
        # direction_y = self.direction_vectors[1]
        direction_z = self.direction_vectors[2]
        r = direction_z*self.length
        # force_x = self.vectorProjection(net_force, direction_x)
        # force_y = self.vectorProjection(net_force, direction_y)
        #x_magnitude = np.sqrt(np.dot(force_x, force_x))
        #y_magnitude = np.sqrt(np.dot(force_y, force_y))
        torque = np.cross(r, net_force) #right-hand rule
        # torque_x = self.length*x_magnitude
        # torque_y = self.length*y_magnitude
        # acceleration_alpha = torque_x/self.I
        # acceleration_beta = torque_y/self.I
        angular_acceleration = torque/self.I
        return np.array([-angular_acceleration[1], angular_acceleration[0]])
    def tensionForce(self, load_direction, netForce):
        ## tension force acting at cog of quadcopter
        return self.vectorProjection(netForce, load_direction)
    def netForce(self, forcesList):
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
        r_x = np.array([cos(alpha), 0, sin(alpha)])
        r_y = np.array([0, cos(beta), sin(beta)])
        r_z = np.array([sin(alpha)*cos(beta), cos(alpha)*sin(beta), -cos(alpha)*cos(beta)])
        self.direction_vectors = (r_x, r_y, r_z)
        return (r_x, r_y, r_z)
    def calculateLoadSpeed(self):
        alpha = self.state[0] ## alpha is rotation in y axis
        beta = self.state[1] ## beta is rotation in x axis
        Vx = self.state[2]*self.length*np.array([cos(alpha), 0, sin(alpha)])
        Vy = self.state[3]*self.length*np.array([0, cos(beta), sin(beta)])
        V = Vx + Vy + self.quad_speed
        return V
    def calculateDragForce(self, V):
        return -self.Cd*self.A*self.ro*(V**2)
    def calculateForces(self):
        gravityForce = np.array([0, 0, -self.mass*self.g])
        inertialForce = self.inertialForce(self.quad_acceleration)
        dragForce = self.calculateDragForce(self.calculateLoadSpeed())
        return [gravityForce, inertialForce]
    def updateStateDict(self):
        self.state_dict = {key: state_value for key, state_value in zip(self.state_names, self.state)}
    def updateQuadStateRepresentation(self, quadcopter_object):
        self.quad_acceleration = quadcopter_object.translational_accelerations
        self.quad_speed = quadcopter_object.state[3:6]
    def updateStateOde(self, x, t=0):
        self.state = x if isinstance(x, np.ndarray) else np.array(x)
        #self.updateStateDict()
        dstate = np.zeros(4)
        forces = self.calculateForces()
        net_force = self.netForce(forces)
        self.directionVectors()
        self.tension_force = self.tensionForce(self.direction_vectors[2], net_force)
        dstate[2:4] = self.angularMotion(net_force)
        dstate[0:2] = self.state[2:4]
        return dstate
    def updateState(self, deltaT):
        forces = self.calculateForces()
        net_force = self.netForce(forces)
        self.directionVectors()
        self.tension_force = self.tensionForce(self.direction_vectors[2], net_force)
        self.state[2:4] = self.angularMotion(net_force)*deltaT + self.state[2:4]
        self.state[0:2] = self.state[2:4] * deltaT + self.state[0:2] ## calculation order seems to have an impact on increasing energy in oscilatory system
        self.updateStateDict()
        return self.state