#X-configuration
#ccw is positive
#   cw  4  🢁🢁🢁  2 ccw
#         \    /  
#          \  /
#           \/
#           /\
#          /  \
#         /    \
#   ccw 3        1 cw

import numpy as np
from controler_parameters import pitch_pid, roll_pid, yaw_pid

class PID():
    def __init__(self, pid_settings, dT):
        self.Kp = pid_settings['Kp']
        self.Ki = pid_settings['Ki']
        self.Kd = pid_settings['Kd']
        self.signals = np.zeros(3)
        self.signal_names = {'P', 'I', 'D'}
        self.signal_dict = {key: signal_value for key, signal_value in zip(self.signal_names, self.signals)}
        self.deltaT = dT
        self.prev_error = 0
    def __call__(self, error):
        P = self.pTerm(error)
        I = self.iTerm(error)
        D = self.dTerm(error)
        u = P + I + D
        self.updateDict()
        return u
    def pTerm(self, error):
        self.signals[0] = self.Kp*error
        return self.signals[0]
    def iTerm(self, error):
        self.signals[1] = self.signals[1] + self.Ki*self.deltaT*error
        return self.signals[1]
    def dTerm(self, error):
        self.signals[2] = (error - self.prev_error)/self.deltaT
        self.signals[2] = self.signals[2]*self.Kd
        self.prev_error = error
        return self.signals[2]
    def updateDict(self):
        for key, signal_value in zip(self.signal_names, self.signals):
            self.signal_dict[key] = signal_value

class angleControler():
    def __init__(self, pid_parameters, dT):
        self.pid_angle = PID(pid_parameters, dT)
        self.pid_rate = PID(pid_parameters, dT)
    def __call__(self, angle_setpoint, angle, angle_rate):
        error = angle_setpoint - angle
        rate_setpoint = self.pid_angle(error)
        error_rate = rate_setpoint - angle_rate
        u = self.pid_rate(error_rate)
        return u

def attitudeControl(attitude_setpoint, attitude, attitude_rate, dT):
    roll_control = angleControler(roll_pid, dT)
    pitch_control = angleControler(pitch_pid, dT)
    yaw_control = angleControler(yaw_pid, dT)
    controlers = [roll_control, pitch_control, yaw_control]
    error = attitude_setpoint - attitude
    u = np.zeros(3)
    for i, controller in enumerate(controlers):
        u[i] = controller(attitude_setpoint[i], attitude[i], attitude_rate[i])
    return (u[0], u[1], u[2])
def quadControl(attitude_setpoint, attitude, attitude_rate, thrust_setpoint, dT):
    (roll_u, pitch_u, yaw_u)  = attitudeControl(attitude_setpoint, attitude, attitude_rate, dT)
    motors = motor_mixing_algorithm(thrust_setpoint, roll_u, pitch_u, yaw_u)
    limits(motors)
    return motors
def limits(motors):
    for motor in motors:
        if motor < 0:
            motor = 0
    return motors
def motor_mixing_algorithm(thrust, roll, pitch, yaw):
    motor1 = thrust + roll - pitch - yaw
    motor2 = thrust + roll + pitch + yaw
    motor3 = thrust - roll - pitch + yaw
    motor4 = thrust - roll + pitch - yaw
    return [motor1, motor2, motor3, motor4]