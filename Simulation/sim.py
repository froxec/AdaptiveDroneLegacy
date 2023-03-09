from model import quadcopterModel, loadPendulum, system
from Factories.ModelsFactory.model_parameters import pendulum_parameters, Z550_parameters
from Factories.ModelsFactory.general_models import ElectronicSpeedControler
import numpy as np
from plots import plotTrajectory
import attitude_control as control
from numpy import deg2rad
import time
from Factories.ControllersFactory.position_controllers.explicit_mpc_controller import ModelPredictiveController
from attitude_control import Normalizer, ThrustToAngularVelocity
FPS = 30
PAUSE_INCREMENT = 1e-5
INNER_LOOP_FREQ = 100
OUTER_LOOP_FREQ = 100
MODULO_FACTOR = INNER_LOOP_FREQ/OUTER_LOOP_FREQ
ANGULAR_VELOCITY_RANGE = [0, 800]
PWM_RANGE = [1120, 1920]
U_SS = [Z550_parameters['m']*Z550_parameters['g'], 0, 0, 0]
class MPC_output_converter():
    def __init__(self, u_ss, Kt, angular_velocity_range):
        self.angular_vel_min = angular_velocity_range[0]
        self.angular_vel_max = angular_velocity_range[1]
        self.u_ss = u_ss
        self.thrust_converter = ThrustToAngularVelocity(Kt)
        self.angular_vel_normalizer = Normalizer(min=self.angular_vel_min, max=self.angular_vel_max)
    def __call__(self, delta_u):
        u = delta_u + self.u_ss
        omega = self.thrust_converter(u[0])
        throttle = self.angular_vel_normalizer(omega)
        u[0] = throttle
        u[3] = 0
        return u


if __name__ == '__main__':
    load0 = np.zeros(4)
    quad0 = np.zeros(12)
    quad = quadcopterModel(quad0, Z550_parameters)
    load = loadPendulum(load0, pendulum_parameters, quad.translational_accelerations, quad.state[3:6])
    esc = ElectronicSpeedControler(pwm_range=PWM_RANGE, angular_velocity_range=ANGULAR_VELOCITY_RANGE)

    # visualizer = ParallelVisualizer()
    # plot_pipe, remote_end = mp.Pipe()
    # plot_process = mp.Process(
    #     target=visualizer,
    #     args=(remote_end,),
    #     daemon=True
    # )
    deltaT = 1/INNER_LOOP_FREQ
    attitude_controler = control.quadControler(deltaT)
    position_controler = ModelPredictiveController(quad_parameters=Z550_parameters, x0=np.array([0, 0, 0, 0, 0, 0]), xref=np.array([0, 0, 10, 0, 0, 0]))
    mpc_output_converter = MPC_output_converter(U_SS, Z550_parameters['Kt'], ANGULAR_VELOCITY_RANGE)
    state0 = np.concatenate([quad0, load0])
    #t = linspace(0, 25, 100)
    #x = odeint(odeSystem, state0, t, args=(quad, load))
    t = np.arange(0, 10, deltaT)
    x = np.zeros((t.size, 16))
    throttle = 0
    attitude_setpoint = np.array([deg2rad(0), deg2rad(0), deg2rad(0)])
    # plot_process.start()
    prev_stop_time = deltaT
    start = time.time()
    for i, t_i in enumerate(t):
        if (i % MODULO_FACTOR) == 0 and i != 0:
            ref = position_controler.update_state_control(x[i-1, :6])
            ref_converted = mpc_output_converter(ref)
            attitude_setpoint = np.array(ref_converted[1:])
            throttle = ref_converted[0]
        ESC_PWMs = attitude_controler(attitude_setpoint, quad.state[6:9], quad.state[9:12], throttle)
        motors = esc(ESC_PWMs)
        x[i] = system(np.array(motors), deltaT, quad, load)
        # if (i % int(1/(deltaT*FPS))) == 0:
        #     #print(i)
        #     #visualizer(quad.state[0:3], quad.state[6:9], t_i)
        #     # send = plot_pipe.send
        #     data_to_send = np.concatenate((quad.state[0:3], quad.state[6:9], np.array([t_i])))
            # send(data_to_send)
        #time.sleep(deltaT)
        # while(time.time() - start < t_i + deltaT):
        #     time.sleep(PAUSE_INCREMENT)

        #print(prev_stop_time)
        #print(time.time() - t1)
    # send(None)
    print(time.time() - start)
    plotTrajectory(t, x.transpose()[0:12], 4, 3)
    #plotDataPID(t, controler, 'roll')
    plotTrajectory(t, x.transpose()[12:16], 2, 2)
    time.sleep(1000)