import math

def euler_to_quaternion(roll, pitch, yaw):
    cY = math.cos(yaw*0.5)
    sY = math.sin(yaw*0.5)
    cR = math.cos(roll*0.5)
    sR = math.sin(roll*0.5)
    cP = math.cos(pitch*0.5)
    sP = math.sin(pitch*0.5)

    w = cY * cR * cP + sY * sR * sP
    x = cY * sR * cP - sY * cR * sP
    y = cY * cR * sP + sY * sR * cP
    z = sY * cR * cP - cY * sR * sP

    return [w, x, y, z]