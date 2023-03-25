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
from Factories.ControllersFactory.attitude_controllers.controler_parameters import pitch_pid, roll_pid, yaw_pid
from Factories.ToolsFactory.Converters import Saturation, RateOfChangeSaturation, LinearScaler


## TODO PRZENIESC KLASY DO MODUŁU "BUILDING BLOCKS"
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
    ##TODO ADD ANTI-WINDUP TO iTerm
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
        self.pid_angle = None
        self.pid_rate = None
        self.pid_acceleration = None
        self.pids = [self.pid_angle, self.pid_rate, self.pid_acceleration]
    def __call__(self, top_setpoint, angle=None, angle_rate=None, angle_acceleration=None):
        setpoints = [top_setpoint]
        measured_state = [angle, angle_rate, angle_acceleration]
        for i, pid in enumerate(self.pids):
            if pid == None:
                continue
            else:
                error = setpoints[-1] - measured_state[i]
                pid_output = pid(error)
                setpoints.append(pid_output)
        u = pid_output
        return u

class TripleCascadeLoop(angleControler):
    def __init__(self, pid_parameters, dT):
        self.pid_angle = PID(pid_parameters['angle'], dT, save_data=True)
        self.pid_rate = PID(pid_parameters['rate'], dT, save_data=True)
        self.pid_acceleration = PID(pid_parameters['acceleration'], dT, save_data=True)
        self.pids = [self.pid_angle, self.pid_rate, self.pid_acceleration]

class DoubleCascadeLoop(angleControler):
    def __init__(self, pid_parameters, dT):
        self.pid_angle = PID(pid_parameters['angle'], dT, save_data=True)
        self.pid_rate = PID(pid_parameters['rate'], dT, save_data=True)
        self.pid_acceleration = None
        self.pids = [self.pid_angle, self.pid_rate, self.pid_acceleration]
class attitudeControler():
    def __init__(self, dT):
        roll_control = DoubleCascadeLoop(roll_pid, dT) ## pass parameters as arguments
        pitch_control = DoubleCascadeLoop(pitch_pid, dT)
        yaw_control = DoubleCascadeLoop(yaw_pid, dT)
        self.controlers = [roll_control, pitch_control, yaw_control]
    def __call__(self, attitude_setpoint, attitude, attitude_rate):
        u = np.zeros(3)
        for i, controller in enumerate(self.controlers):
            u[i] = controller(attitude_setpoint[i], attitude[i], attitude_rate[i])
        return (u[0], u[1], u[2])

class quadControler():
    def __init__(self, dT, PWM_RANGE, PWM0):
        self.attitude_control = attitudeControler(dT)
        self.pid_saturation = Saturation(lower_limit=-1, upper_limit=1)
        self.throttle_saturation = Saturation(lower_limit=0, upper_limit=1)
        self.pid_to_pwm_scaler = LinearScaler(min=-100, max=100)
        self.throttle_to_pwm_scaler = LinearScaler(min=1120, max=1920)
        self.motor_rate_limiter = RateOfChangeSaturation(rate_limit=2000, deltaT=dT)
        self.signal_to_ESC_prev = [PWM0, PWM0, PWM0, PWM0]
        self.pwm_limiter = Saturation(lower_limit=PWM_RANGE[0], upper_limit=PWM_RANGE[1])
    def __call__(self, attitude_setpoint, attitude, attitude_rate, throttle):
        '''throttle_setpoint should be in range (0, 1)'''
        throttle_limited = self.throttle_saturation(throttle)
        (roll_u, pitch_u, yaw_u)  = self.attitude_control(attitude_setpoint, attitude, attitude_rate)
        (roll_u_limited, pitch_u_limited, yaw_u_limited) = (self.pid_saturation(roll_u), self.pid_saturation(pitch_u), self.pid_saturation(yaw_u))
        (roll_PWM, pitch_PWM, yaw_PWM) = (self.pid_to_pwm_scaler(roll_u_limited/2 + 0.5), self.pid_to_pwm_scaler(pitch_u_limited/2 + 0.5), self.pid_to_pwm_scaler(yaw_u_limited/2 + 0.5))
        throttle_PWM = self.throttle_to_pwm_scaler(throttle_limited)
        signal_to_ESC = motor_mixing_algorithm(throttle_PWM, roll_PWM, pitch_PWM, yaw_PWM)
        signal_to_ESC = self.motor_rate_limiter(signal_to_ESC, self.signal_to_ESC_prev)
        signal_to_ESC = self.pwm_limiter(signal_to_ESC)
        self.signal_to_ESC_prev = signal_to_ESC
        return signal_to_ESC

def limits(motors):
    #currently not used as PWM's are scaled, might be deleted later
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