from Factories.ConfigurationsFactory.configurations import QuadConfiguration, ControllerConfiguration, ControllerWithCompensatorConfiguration, GekkoConfiguration
from Factories.SimulationsFactory.SITL import SoftwareInTheLoop
from Factories.ModelsFactory.model_parameters import pendulum_parameters, Z550_parameters
import numpy as np
from plots import plotTrajectory, plotTrajectory3d
import time
from Factories.ToolsFactory.AnalysisTools import ParametersPerturber
from Factories.SimulationsFactory.TrajectoriesDepartment.trajectories import SpiralTrajectory
from Factories.ModelsFactory.linear_models import LinearizedQuadNoYaw

FPS = 30
PAUSE_INCREMENT = 1e-5
INNER_LOOP_FREQ = 100
OUTER_LOOP_FREQ = 10
MODULO_FACTOR = int(INNER_LOOP_FREQ/OUTER_LOOP_FREQ)
ANGULAR_VELOCITY_RANGE = [0, 800]
PWM_RANGE = [1120, 1920]
spiral_trajectory = SpiralTrajectory(15)
#trajectory = np.array([0, 10, 10])
trajectory = spiral_trajectory
if __name__ == '__main__':
    deltaT = 1 / INNER_LOOP_FREQ

    quad_conf = QuadConfiguration(Z550_parameters, pendulum_parameters, np.zeros(12), np.zeros(4), PWM_RANGE, ANGULAR_VELOCITY_RANGE)

    perturber = ParametersPerturber(Z550_parameters)
    perturber({'m': 0.0})
    print(perturber.perturbed_parameters)

    controller_compensator_conf = ControllerWithCompensatorConfiguration(perturber.perturbed_parameters, position0=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                                           trajectory=trajectory,
                                           u_ss=[perturber.perturbed_parameters['m']*perturber.perturbed_parameters['g'], 0.0, 0.0], prediction_model=LinearizedQuadNoYaw,
                                           INNER_LOOP_FREQ=INNER_LOOP_FREQ, OUTER_LOOP_FREQ=OUTER_LOOP_FREQ,
                                           ANGULAR_VELOCITY_RANGE=ANGULAR_VELOCITY_RANGE, PWM_RANGE=PWM_RANGE)
    gekko_controller_conf = GekkoConfiguration(perturber.perturbed_parameters, position0=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                                           trajectory=trajectory,
                                            x_ss = np.array([0, 0, 0, 0, 0, 0]),
                                           u_ss=np.array([perturber.perturbed_parameters['m']*perturber.perturbed_parameters['g'], 0.0, 0.0]), prediction_model=LinearizedQuadNoYaw,
                                           INNER_LOOP_FREQ=INNER_LOOP_FREQ, OUTER_LOOP_FREQ=OUTER_LOOP_FREQ,
                                           ANGULAR_VELOCITY_RANGE=ANGULAR_VELOCITY_RANGE, PWM_RANGE=PWM_RANGE, control_horizon=2)
    simulator = SoftwareInTheLoop(quad_conf.quadcopter, quad_conf.load, trajectory, gekko_controller_conf.position_controller, gekko_controller_conf.attitude_controller,
                                  [gekko_controller_conf.position_controller_input_converter, gekko_controller_conf.position_controller_output_converter]
                                  , quad_conf.esc, INNER_LOOP_FREQ, OUTER_LOOP_FREQ)
    # visualizer = ParallelVisualizer()
    # plot_pipe, remote_end = mp.Pipe()
    # plot_process = mp.Process(
    #     target=visualizer,
    #     args=(remote_end,),
    #     daemon=True
    # )
    state0 = np.concatenate([quad_conf.quad0, quad_conf.load0])
    u0 = np.array([0, 0, 0])
    setpoint = np.array([0, 0, 10, 0, 0, 0])
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
    t, x = simulator.run(20, deltaT, state0[0:12], u0, setpoint)
    #control_conf.thrust_compensator.plot_signals(t)
    plotTrajectory3d(x, spiral_trajectory.generated_trajectory)
    plotTrajectory(t, x.transpose()[0:12], 4, 3)
    #plotDataPID(t, controler, 'roll')
    #plotTrajectory(t[1::MODULO_FACTOR], np.vstack(control_conf.position_controller.history).transpose(), 3, 1)