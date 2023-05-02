from Factories.ConfigurationsFactory.configurations import QuadConfiguration, ControllerWithCompensatorConfiguration, CustomMPCConfig
from Factories.SimulationsFactory.SITL import SoftwareInTheLoop
from Factories.ModelsFactory.model_parameters import pendulum_parameters, Z550_parameters
import numpy as np
from plots import plotTrajectory, plotTrajectory3d
import time
from Factories.ToolsFactory.AnalysisTools import ParametersPerturber
from Factories.SimulationsFactory.TrajectoriesDepartment.trajectories import SpiralTrajectory, RectangularTrajectory, SinglePoint
from Factories.ModelsFactory.linear_models import LinearizedQuadNoYaw, AugmentedLinearizedQuadNoYaw
from Factories.ConfigurationsFactory.modes import MPCModes
FPS = 30
PAUSE_INCREMENT = 1e-5
INNER_LOOP_FREQ = 100
OUTER_LOOP_FREQ = 10
MODULO_FACTOR = int(INNER_LOOP_FREQ/OUTER_LOOP_FREQ)
ANGULAR_VELOCITY_RANGE = [0, 800]
PWM_RANGE = [1120, 1920]
spiral_trajectory = SpiralTrajectory(15)
rectangular_trajectory = RectangularTrajectory()
single_point_traj = SinglePoint(np.array([0, 0, 10]))
trajectory = single_point_traj
#trajectory = spiral_trajectory
#trajectory = rectangular_trajectory
if __name__ == '__main__':
    deltaT = 1 / INNER_LOOP_FREQ

    quad_conf = QuadConfiguration(Z550_parameters, pendulum_parameters, np.zeros(12), np.zeros(4), PWM_RANGE, ANGULAR_VELOCITY_RANGE)

    perturber = ParametersPerturber(Z550_parameters)
    perturber({'m': 0.0})
    print(perturber.perturbed_parameters)

    # controller_compensator_conf = ControllerWithCompensatorConfiguration(perturber.perturbed_parameters, position0=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    #                                        trajectory=trajectory,
    #                                        u_ss=[perturber.perturbed_parameters['m']*perturber.perturbed_parameters['g'], 0.0, 0.0],
    #                                        x_ss = np.array([0, 0, 0, 0, 0, 0]),
    #                                        prediction_model=LinearizedQuadNoYaw,
    #                                        INNER_LOOP_FREQ=INNER_LOOP_FREQ, OUTER_LOOP_FREQ=OUTER_LOOP_FREQ,
    #                                        ANGULAR_VELOCITY_RANGE=ANGULAR_VELOCITY_RANGE, PWM_RANGE=PWM_RANGE)
    prediction_model = LinearizedQuadNoYaw(perturber.perturbed_parameters, 1/OUTER_LOOP_FREQ)
    prediction_model2 = AugmentedLinearizedQuadNoYaw(perturber.perturbed_parameters, 1/OUTER_LOOP_FREQ)
    controller_conf = CustomMPCConfig(prediction_model, INNER_LOOP_FREQ, OUTER_LOOP_FREQ, ANGULAR_VELOCITY_RANGE, PWM_RANGE, horizon=10)
    controller_conf.position_controller.switch_modes(MPCModes.UNCONSTRAINED_WITH_SOLVER)
    # gekko_controller_conf = GekkoConfiguration(perturber.perturbed_parameters, position0=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    #                                        trajectory=trajectory,
    #                                         x_ss = np.array([0, 0, 0, 0, 0, 0]),
    #                                        u_ss=np.array([perturber.perturbed_parameters['m']*perturber.perturbed_parameters['g'], 0.0, 0.0]), prediction_model=LinearizedQuadNoYaw,
    #                                        INNER_LOOP_FREQ=INNER_LOOP_FREQ, OUTER_LOOP_FREQ=OUTER_LOOP_FREQ,
    #                                        ANGULAR_VELOCITY_RANGE=ANGULAR_VELOCITY_RANGE, PWM_RANGE=PWM_RANGE, control_horizon=5)
    simulator = SoftwareInTheLoop(quad_conf.quadcopter, quad_conf.load, trajectory, controller_conf.position_controller, controller_conf.attitude_controller,
                                  [controller_conf.position_controller_input_converter, controller_conf.position_controller_output_converter]
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
    t, x = simulator.run(250, deltaT, state0[0:12], u0, trajectory)
    #control_conf.thrust_compensator.plot_signals(t)
    controller_conf.position_controller.plot_history()
    plotTrajectory3d(x, trajectory.generated_trajectory)
    plotTrajectory(t, x.transpose()[0:12], 4, 3)
    #plotDataPID(t, controler, 'roll')
    #plotTrajectory(t[1::MODULO_FACTOR], np.vstack(control_conf.position_controller.history).transpose(), 3, 1)