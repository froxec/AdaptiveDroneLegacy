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
    def __init__(self, pid_settings, dT, save_data = False):
        self.Kp = pid_settings['Kp']
        self.Ki = pid_settings['Ki']
        self.Kd = pid_settings['Kd']
        self.signals = np.zeros((3), dtype=np.float32)
        self.signal_names = {'P', 'I', 'D'}
        self.signal_dict = {key: signal_value for key, signal_value in zip(self.signal_names, self.signals)}
        self.deltaT = dT
        self.prev_error = 0
        self.save_data = save_data
        if self.save_data:
            self.error_history = []
            self.PID_history = []
    def __call__(self, error):
        P = self.pTerm(error)
        I = self.iTerm(error)
        D = self.dTerm(error)
        u = P + I + D
        self.update(error)
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
    def update(self, error):
        for key, signal_value in zip(self.signal_names, self.signals):
            self.signal_dict[key] = signal_value
        if self.save_data:
            self.saveData(error)
    def saveData(self, error):
        self.PID_history.append(sum(self.signals)) 
        self.error_history.append(error)
    
class angleControler():
    def __init__(self, pid_parameters, dT):
        self.pid_angle = PID(pid_parameters['angle'], dT, save_data = True) #make it possible to pass individual parameters for each PID
        self.pid_rate = PID(pid_parameters['rate'], dT, save_data = True)
        self.pid_acceleration = PID(pid_parameters['acceleration'], dT, save_data = True)
        self.pids = [self.pid_angle, self.pid_rate, self.pid_acceleration]
    def __call__(self, angle_setpoint, angle, angle_rate, angle_acceleration):
        error = angle_setpoint - angle
        rate_setpoint = self.pid_angle(error)
        error_rate = rate_setpoint - angle_rate
        acceleration_setpoint = self.pid_rate(error_rate)
        error_acceleration = acceleration_setpoint - angle_acceleration
        u = self.pid_acceleration(error_acceleration)
        return u

class attitudeControler():
    def __init__(self, dT):
        roll_control = angleControler(roll_pid, dT) ## pass parameters as arguments 
        pitch_control = angleControler(pitch_pid, dT)
        yaw_control = angleControler(yaw_pid, dT)
        self.controlers = [roll_control, pitch_control, yaw_control]
    def __call__(self, attitude_setpoint, attitude, attitude_rate, attitude_acceleration):
        u = np.zeros(3)
        for i, controller in enumerate(self.controlers):
            u[i] = controller(attitude_setpoint[i], attitude[i], attitude_rate[i], attitude_acceleration[i])
        return (u[0], u[1], u[2])

class quadControler():
    def __init__(self, dT):
        self.attitude_control = attitudeControler(dT)
    def __call__(self, attitude_setpoint, attitude, attitude_rate, attitude_acceleration, thrust_setpoint):
        (roll_u, pitch_u, yaw_u)  = self.attitude_control(attitude_setpoint, attitude, attitude_rate, attitude_acceleration)
        motors = motor_mixing_algorithm(thrust_setpoint, roll_u, pitch_u, yaw_u)
        limits(motors)
        return motors

def limits(motors):
    for motor in motors:
        if motor < 0:
            motor = 0
        if motor > 2000:
            motor = 2000
    return motors
    
def motor_mixing_algorithm(throttle, roll, pitch, yaw):
    motor1 = throttle + roll - pitch - yaw
    motor2 = throttle + roll + pitch + yaw
    motor3 = throttle - roll - pitch + yaw
    motor4 = throttle - roll + pitch - yaw
    return [motor1, motor2, motor3, motor4]