from math import sin, cos
import numpy as np
from model_parameters import m, g, I, l, Kt, Kd

def transfomationMatrix(roll, pitch, yaw):
    cR = cos(roll)
    cP = cos(pitch)
    cY = cos(yaw)
    sR = sin(roll)
    sP = sin(pitch)
    sY = sin(yaw)
    R = np.array([[cP*cY, sR*sP*cY - cR*sY, cR*sP*cY + sR*sY],
                  [cP*sY, sR*sP*sY - cR*cY, cR*sP*sY - sR*cY],
                  [-sP, sR*cP, cR*cP]], dtype=np.float32)
    return R

def translationalMotion(R, F):
    f = F[0] + F[1]  + F[2] + F[3]
    accelerations = np.zeros((3, 1), dtype=np.float32)
    bodyFrameThrust = np.array([[0], 
                               [0],
                               [f]], dtype=np.float32)
    referenceFrame = np.matmul(R, bodyFrameThrust) + np.array([[0],
                                                               [0],
                                                               [-m*g]], dtype=np.float32)
    accelerations = referenceFrame/m
    accelerations = np.reshape(accelerations, 3)
    return accelerations

def angularMotion(F, M, eulerAnglesPrim):
    T = np.zeros(3)
    T[0] = (F[0] - F[3])*l
    T[1] = (F[1] - F[2])*l
    T[2] = M[0] + M[1] + M[2] + M[3]
    accelerations =  T - np.cross(eulerAnglesPrim, np.multiply(I, eulerAnglesPrim))
    accelerations = np.divide(accelerations, I)
    return accelerations

def inputToForces(omega):
    return np.multiply(Kt, omega)

def inputToMomentum(omega):
    return np.multiply(Kd, omega)

def model(x):
    dstate = np.zeros(12)
    omega = np.array([2000, 2000, 2000, 2000])
    F = inputToForces(omega)
    M = inputToMomentum(omega)
    R = transfomationMatrix(x[3], x[4], x[5])
    dstate[0:3] = x[3:6]
    dstate[3:6] = translationalMotion(R, F)
    dstate[6:9] = x[9:12]
    dstate[9:12] = angularMotion(F, M, x[9:12])
    return dstate 

def modelRT(x, u, deltaT):
    state = np.zeros(12)
    omega = np.array(u)
    F = inputToForces(omega)
    M = inputToMomentum(omega)
    R = transfomationMatrix(x[3], x[4], x[5])
    state[0:3] = x[3:6] * deltaT + x[0:3]
    state[3:6] = translationalMotion(R, F) * deltaT + x[3:6]
    state[6:9] = x[9:12] * deltaT + x[6:9]
    state[9:12] = angularMotion(F, M, x[9:12]) * deltaT + x[9:12]
    if (state[2] < 0):
        state[2] = 0
    return state 