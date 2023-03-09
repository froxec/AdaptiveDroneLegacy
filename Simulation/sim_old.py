from Factories.ToolsFactory.Converters import MPC_output_converter
from model import quadcopterModel, loadPendulum, system
from Factories.ModelsFactory.model_parameters import pendulum_parameters, Z550_parameters
from Factories.ModelsFactory.general_models import ElectronicSpeedControler
import numpy as np
from plots import plotTrajectory
import attitude_control as control
from numpy import deg2rad
import time
from Factories.ControllersFactory.position_controllers.explicit_mpc_controller import ModelPredictiveController
from Factories.ToolsFactory.AnalysisTools import ParametersPerturber
FPS = 30
PAUSE_INCREMENT = 1e-5
INNER_LOOP_FREQ = 100
OUTER_LOOP_FREQ = 10
MODULO_FACTOR = int(INNER_LOOP_FREQ/OUTER_LOOP_FREQ)
ANGULAR_VELOCITY_RANGE = [0, 800]
PWM_RANGE = [1120, 1920]
class SoftwareInTheLoop:
    def __init__(self, object, position_controller, attitude_controler, mpc_output_converter, esc, inner_loop_freq, outer_loop_freq):
        self.INNER_LOOP_FREQ = inner_loop_freq
        self.OUTER_LOOP_FREQ = outer_loop_freq
        self.MODULO_FACTOR = self.INNER_LOOP_FREQ / self.OUTER_LOOP_FREQ
        self.object = object
        self.position_controller = position_controller
        self.attitude_controller = attitude_controler
        self.mpc_output_converter = mpc_output_converter
        self.esc = esc
    def run(self, stop_time, deltaT, x0):
        t = np.arange(0, stop_time, deltaT)
        x = np.zeros((t.size, 12))
        x[0] = x0
        for i, t_i in enumerate(t[1:], 1):
            if (i % MODULO_FACTOR) == 1:
                ref = self.position_controller.update_state_control(x[i - 1, :6])
                ref_converted = self.mpc_output_converter(ref)
                attitude_setpoint = np.array(ref_converted[1:])
                throttle = ref_converted[0]
            ESC_PWMs = self.attitude_controller(attitude_setpoint, quad.state[6:9], quad.state[9:12], throttle)
            motors = self.esc(ESC_PWMs)
            x[i] = system(np.array(motors), deltaT, quad, load)[:12]
            ##TODO move animation into different module
            # if (i % int(1/(deltaT*FPS))) == 0:
            #     #print(i)
            #     #visualizer(quad.state[0:3], quad.state[6:9], t_i)
            #     # send = plot_pipe.send
            #     data_to_send = np.concatenate((quad.state[0:3], quad.state[6:9], np.array([t_i])))
            # send(data_to_send)
            # time.sleep(deltaT)
            # while(time.time() - start < t_i + deltaT):
            #     time.sleep(PAUSE_INCREMENT)

            # print(prev_stop_time)
            # print(time.time() - t1)
        return t, x


if __name__ == '__main__':
    load0 = np.zeros(4)
    quad0 = np.zeros(12)
    deltaT = 1 / INNER_LOOP_FREQ
    #quad0[2] = 10
    quad = quadcopterModel(quad0, Z550_parameters)
    load = loadPendulum(load0, pendulum_parameters, quad.translational_accelerations, quad.state[3:6])
    esc = ElectronicSpeedControler(pwm_range=PWM_RANGE, angular_velocity_range=ANGULAR_VELOCITY_RANGE)
    perturber = ParametersPerturber(Z550_parameters)
    perturber({'Kt': 0.05e-5})
    print(perturber.perturbed_parameters)
    position_controler = ModelPredictiveController(quad_parameters=perturber.perturbed_parameters, x0=np.array([0, 0, 0, 0, 0, 0]),
                                                   xref=np.array([10, 10, 10, 0, 0, 0]), Ts= 1 / OUTER_LOOP_FREQ)
    mpc_output_converter = MPC_output_converter([perturber.perturbed_parameters['m']*perturber.perturbed_parameters['g'], 0, 0, 0], perturber.perturbed_parameters['Kt'], ANGULAR_VELOCITY_RANGE)
    attitude_controler = control.quadControler(deltaT)
    simulator = SoftwareInTheLoop(quad, position_controler, attitude_controler, mpc_output_converter, esc, INNER_LOOP_FREQ, OUTER_LOOP_FREQ)
    # visualizer = ParallelVisualizer()
    # plot_pipe, remote_end = mp.Pipe()
    # plot_process = mp.Process(
    #     target=visualizer,
    #     args=(remote_end,),
    #     daemon=True
    # )
    state0 = np.concatenate([quad0, load0])
    # plot_process.start()
    prev_stop_time = deltaT
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
    t, x = simulator.run(20, deltaT, state0[0:12])
    plotTrajectory(t, x.transpose()[0:12], 4, 3)
    #plotDataPID(t, controler, 'roll')
    plotTrajectory(t[1::MODULO_FACTOR], np.vstack(position_controler.history).transpose(), 2, 2)
    time.sleep(1000)