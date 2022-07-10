from curses.ascii import CR
from math import sin, cos
import numpy as np
from model_parameters import m, g

def transfomationMatrix(roll, pitch, yaw):
    cR = cos(roll)
    cP = cos(pitch)
    cY = cos(yaw)
    sR = sin(roll)
    sP = sin(pitch)
    sY = sin(yaw)
    R = np.array([[cP*cY, sR*sP*cY - cR*sY, cR*sP*cY + sR*sY],
                  [cP*sY, sR*sP*sY - cR*cY, cR*sP*sY - sR*cY],
                  [-sP, sR*cP, cR*cP]], dtype=float)
def translationalMotion(R, F):
    accelerations = np.zeros((3, 1), dtype=float)
    bodyFrameThrust = np.array([[0], 
                               [0],
                               [F]], dtype=float)
    referenceFrame = np.matmul(R, bodyFrameThrust) + np.array([[0]
                                                               [0]
                                                               [-m*g]], dtype=float)
    accelerations = referenceFrame/m
def angularMotion()