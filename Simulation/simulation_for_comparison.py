from Factories.ConfigurationsFactory.configurations import QuadConfiguration, ControllerWithCompensatorConfiguration, CustomMPCConfig
from Factories.SimulationsFactory.SITL import SoftwareInTheLoopLegacy
from Factories.ModelsFactory.model_parameters import pendulum_parameters, Z550_parameters
import numpy as np
from plots import plotTrajectory, plotTrajectory3d
import time
from Factories.ToolsFactory.AnalysisTools import ParametersPerturber
from Factories.SimulationsFactory.TrajectoriesDepartment.trajectories import SpiralTrajectory, RectangularTrajectory, SinglePoint, \
    RectangularTrajectoryWithTerminals
from Factories.ModelsFactory.linear_models import LinearizedQuadNoYaw, AugmentedLinearizedQuadNoYaw
from Factories.ConfigurationsFactory.modes import MPCModes
import plotly.graph_objects as go
from plotly.subplots import make_subplots

FPS = 30
PAUSE_INCREMENT = 1e-5
INNER_LOOP_FREQ = 100
OUTER_LOOP_FREQ = 10
MODULO_FACTOR = int(INNER_LOOP_FREQ/OUTER_LOOP_FREQ)
ANGULAR_VELOCITY_RANGE = [0, 800]
PWM_RANGE = [1120, 1920]
spiral_trajectory = SpiralTrajectory(15)
rectangular_trajectory = RectangularTrajectory()
rectangular_trajectory_with_terminals = RectangularTrajectoryWithTerminals()
single_point_traj = SinglePoint(np.array([0, 0, 10]))
trajectory = single_point_traj
#trajectory = spiral_trajectory
#trajectory = rectangular_trajectory_with_terminals
#trajectory = rectangular_trajectory
mass_to_test = [-0.8,-0.5, 0.0, 0.5, 1.0]
colors = ['blue', 'red', 'orange', 'green', 'purple']
titles = ['x [m]', 'y[m]', 'z[m]', 'Vx [m/s]', 'Vy[m/s]', 'Vz[m/s]',
          'phi [rad]', 'theta [rad]', 'psi [rad]',
          'omega_x [rad/s]', 'omega_y [rad/s]', 'omega_z [rad/s]']
rows = 4
cols = 3
fig = make_subplots(rows, cols)
if __name__ == '__main__':
    deltaT = 1 / INNER_LOOP_FREQ

    for i, perturbation in enumerate(mass_to_test):
        quad_conf = QuadConfiguration(Z550_parameters, pendulum_parameters, np.zeros(12), np.zeros(4), PWM_RANGE,
                                      ANGULAR_VELOCITY_RANGE)
        perturber = ParametersPerturber(Z550_parameters)
        perturber({'m': perturbation})
        print(perturber.perturbed_parameters)

        prediction_model = LinearizedQuadNoYaw(perturber.perturbed_parameters, 1 / OUTER_LOOP_FREQ)
        prediction_model2 = AugmentedLinearizedQuadNoYaw(perturber.perturbed_parameters, 1/OUTER_LOOP_FREQ)
        controller_conf = CustomMPCConfig(prediction_model, INNER_LOOP_FREQ, OUTER_LOOP_FREQ, ANGULAR_VELOCITY_RANGE, PWM_RANGE, horizon=10)
        controller_conf.position_controller.switch_modes(MPCModes.UNCONSTRAINED)

        simulator = SoftwareInTheLoopLegacy(quad_conf.quadcopter, quad_conf.load, trajectory, controller_conf.position_controller, controller_conf.attitude_controller,
                                            [controller_conf.position_controller_input_converter, controller_conf.position_controller_output_converter]
                                            , quad_conf.esc, INNER_LOOP_FREQ, OUTER_LOOP_FREQ)

        state0 = np.concatenate([quad_conf.quad0, quad_conf.load0])
        u0 = np.array([0, 0, 0])

        t, x = simulator.run(20, deltaT, state0[0:12], u0, trajectory)

        fig.add_trace(go.Scatter(x=t, y=x[:, 0], name="m:{:.2f}".format(perturber.perturbed_parameters['m']), legendgroup=i, line=dict(color=colors[i]), showlegend=True), row=1, col=1)
        fig.add_trace(go.Scatter(x=t, y=x[:, 1], name="m:{:.2f}".format(perturber.perturbed_parameters['m']), legendgroup=i, line=dict(color=colors[i]), showlegend=False), row=1, col=2)
        fig.add_trace(go.Scatter(x=t, y=x[:, 2], name="m:{:.2f}".format(perturber.perturbed_parameters['m']), legendgroup=i, line=dict(color=colors[i]), showlegend=False), row=1, col=3)
        fig.add_trace(go.Scatter(x=t, y=x[:, 3], name="m:{:.2f}".format(perturber.perturbed_parameters['m']), legendgroup=i, line=dict(color=colors[i]), showlegend=False), row=2, col=1)
        fig.add_trace(go.Scatter(x=t, y=x[:, 4], name="m:{:.2f}".format(perturber.perturbed_parameters['m']), legendgroup=i, line=dict(color=colors[i]), showlegend=False), row=2, col=2)
        fig.add_trace(go.Scatter(x=t, y=x[:, 5], name="m:{:.2f}".format(perturber.perturbed_parameters['m']), legendgroup=i, line=dict(color=colors[i]), showlegend=False), row=2, col=3)
        fig.add_trace(go.Scatter(x=t, y=x[:, 6], name="m:{:.2f}".format(perturber.perturbed_parameters['m']), legendgroup=i, line=dict(color=colors[i]), showlegend=False), row=3, col=1)
        fig.add_trace(go.Scatter(x=t, y=x[:, 7], name="m:{:.2f}".format(perturber.perturbed_parameters['m']), legendgroup=i, line=dict(color=colors[i]), showlegend=False), row=3, col=2)
        fig.add_trace(go.Scatter(x=t, y=x[:, 8], name="m:{:.2f}".format(perturber.perturbed_parameters['m']), legendgroup=i, line=dict(color=colors[i]), showlegend=False), row=3, col=3)
        fig.add_trace(go.Scatter(x=t, y=x[:, 9], name="m:{:.2f}".format(perturber.perturbed_parameters['m']), legendgroup=i, line=dict(color=colors[i]), showlegend=False), row=4, col=1)
        fig.add_trace(go.Scatter(x=t, y=x[:, 10], name="m:{:.2f}".format(perturber.perturbed_parameters['m']), legendgroup=i, line=dict(color=colors[i]), showlegend=False), row=4, col=2)
        fig.add_trace(go.Scatter(x=t, y=x[:, 11], name="m:{:.2f}".format(perturber.perturbed_parameters['m']), legendgroup=i, line=dict(color=colors[i]), showlegend=False), row=4, col=3)

        plotTrajectory3d(x, trajectory.generated_trajectory)
    titles = np.array(titles).reshape((rows, cols))
    for i in range(rows):
        for j in range(cols):
            fig.update_xaxes(title_text='czas [s]', row=i+1, col=j+1)
            fig.update_yaxes(title_text=titles[i, j], row=i+1, col=j+1)
    fig.update_layout(font=dict(size=36))
    fig.show()
    # controller_conf.position_controller.plot_history()
    # plotTrajectory3d(x, trajectory.generated_trajectory)
    # plotTrajectory(t, x.transpose()[0:12], 4, 3)