import numpy as np
class ElectronicSpeedControler():
    def __init__(self, pwm_range, angular_velocity_range):
        self.pwm_min = pwm_range[0]
        self.pwm_max = pwm_range[1]
        self.rot_vel_min = angular_velocity_range[0]
        self.rot_vel_max = angular_velocity_range[1]
    def __call__(self, PWMs):
        motor1 = self.pwm_to_angular_vel(PWMs[0])
        motor2 = self.pwm_to_angular_vel(PWMs[1])
        motor3 = self.pwm_to_angular_vel(PWMs[2])
        motor4 = self.pwm_to_angular_vel(PWMs[3])
        return [motor1, motor2, motor3, motor4]
    def pwm_to_angular_vel(self, PWM):
        normalized_signal = (PWM - self.pwm_min)/(self.pwm_max - self.pwm_min)
        angular_velocity = normalized_signal*(self.rot_vel_max - self.rot_vel_min) + self.rot_vel_min
        return angular_velocity