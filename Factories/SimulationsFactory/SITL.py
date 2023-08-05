import numpy as np
from Simulation.model import system
import plotly.graph_objects as go
from plotly.subplots import make_subplots

from Simulation.model import quadcopterModel, loadPendulum, System
from Factories.RLFactory.Agents.BanditEstimatorAgent import BanditEstimatorAgent, BanditEstimatorAcceleration
from Factories.SimulationsFactory.TrajectoriesDepartment.trajectories import *
from Factories.ToolsFactory.GeneralTools import euclidean_distance
from Factories.ToolsFactory.Converters import RampSaturation
from Factories.ModelsFactory.uncertain_models import *
from Factories.ControllersFactory.position_controllers.position_controller import PositionController
from typing import Type
from copy import deepcopy

class SoftwareInTheLoop:
    def __init__(self, quad: Type[quadcopterModel],
                 trajectory,
                 position_controller: Type[PositionController],
                 attitude_controller, esc,
                 inner_loop_freq: int,
                 outer_loop_freq: int,
                 estimator=None,
                 adaptive_controller=None,
                 ramp_saturation_slope=np.array([np.Inf, np.Inf, np.Inf]),):
        self.INNER_LOOP_FREQ = inner_loop_freq
        self.OUTER_LOOP_FREQ = outer_loop_freq
        self.MODULO_FACTOR = self.INNER_LOOP_FREQ / self.OUTER_LOOP_FREQ
        self.quad = quad
        self.position_controller = position_controller
        self.attitude_controller = attitude_controller
        self.estimator = estimator
        self.trajectory = trajectory
        self.esc = esc
        self.adaptive_controller = adaptive_controller
        self.ramp_saturation = RampSaturation(slope_max=ramp_saturation_slope, Ts=1 / self.OUTER_LOOP_FREQ)
        if self.estimator is not None:
            self.system = System(self.quad, return_dstate=True)
        else:
            self.system = System(self.quad, return_dstate=True)

    def run(self, stop_time, deltaT, x0, u0, setpoint):
        import time
        t = np.arange(0, stop_time, deltaT)
        x = np.zeros((t.size, 12))
        x[0] = x0
        z_prev = x0[3:6]
        ref_prev = u0
        u_prev = ref_prev
        self.position_controller.change_trajectory(setpoint)
        for i, t_i in enumerate(t[1:], 0):
            if (i % self.MODULO_FACTOR) == 0:
                ref = self.position_controller(x[i, :6], convert_throttle=False)
                if self.adaptive_controller is None:
                    u_composite = ref
            if (self.adaptive_controller is not None and
                    isinstance(self.adaptive_controller.predictor.ref_model, QuadTranslationalDynamicsUncertain)):
                z = x[i, 3:6]
                z_prev = x[i - 1, 3:6]
                u = ref
                u_composite = self.adaptive_controller(z, z_prev,
                                                       np.concatenate([u, np.array([0])]),
                                                       np.concatenate([u_prev, np.array([0])]), t_i)
                u_prev = u
            elif (self.adaptive_controller is not None and
                  isinstance(self.adaptive_controller.predictor.ref_model, LinearQuadUncertain)):
                delta_x, delta_u = self.position_controller.input_converter(x[i, :6], ref)
                z = delta_x[3:6]
                u = delta_u
                delta_u_composite = self.adaptive_controller(z, z_prev,
                                                             u, u_prev, t_i)
                u_prev = u
                u_composite = self.position_controller.output_converter(delta_u_composite, throttle=False)
                z_prev = z
            mpc_u = self.position_controller.output_converter.convert_throttle(u_composite)
            attitude_setpoint = np.concatenate([mpc_u[1:], np.array([0.0])])
            throttle = mpc_u[0]
            ESC_PWMs = self.attitude_controller(attitude_setpoint, self.quad.state[6:9], self.quad.state[9:12],
                                                throttle)
            motors = self.esc(ESC_PWMs)
            x[i + 1], dstate = self.system(np.array(motors), deltaT, self.quad)[:12]
            if isinstance(self.estimator, BanditEstimatorAgent):
                self.estimator(x[i + 1, :6], ref_prev)
            elif isinstance(self.estimator, BanditEstimatorAcceleration):
                self.estimator(dstate[3:6], np.concatenate([u_composite, np.array([0.0])]))
            if isinstance(self.trajectory, type(TrajectoryWithTerminals())):
                terminal_ind = self.check_if_reached_terminal(x[i + 1, :6])
                if terminal_ind is not None:
                    self.quad.mass = self.quad.nominal_mass + self.trajectory.terminals_payload[terminal_ind]
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

    def check_if_reached_terminal(self, x):
        x = x[None, :]
        terminals = self.trajectory.terminals
        terminals = np.concatenate([terminals, np.zeros((3, 3))], axis=1)
        distances = euclidean_distance(x, terminals, axis=1)
        terminal_ind = None
        if True in [dist < 1 for dist in distances]:
            terminal_ind = np.argmin(distances)
            print(terminal_ind)
        return terminal_ind


class SoftwareInTheLoopLegacy:
    # TODO pass controllers as configuration
    def __init__(self, quad: Type[quadcopterModel],
                 load: Type[loadPendulum],
                 trajectory,
                 position_controller,
                 attitude_controller,
                 mpc_converters, esc,
                 inner_loop_freq: int,
                 outer_loop_freq: int,
                 thrust_compensator=None,
                 estimator=None,
                 adaptive_controller=None,
                 ramp_saturation_slope=np.array([np.Inf, np.Inf, np.Inf])):
        self.INNER_LOOP_FREQ = inner_loop_freq
        self.OUTER_LOOP_FREQ = outer_loop_freq
        self.MODULO_FACTOR = self.INNER_LOOP_FREQ / self.OUTER_LOOP_FREQ
        self.quad = quad
        self.load = load
        self.position_controller = position_controller
        self.attitude_controller = attitude_controller
        self.mpc_input_converter = mpc_converters[0]
        self.mpc_output_converter = mpc_converters[1]
        self.thrust_compensator = thrust_compensator
        self.estimator = estimator
        self.trajectory = trajectory
        self.esc = esc
        self.adaptive_controller = adaptive_controller
        self.ramp_saturation = RampSaturation(slope_max=ramp_saturation_slope, Ts=1/self.OUTER_LOOP_FREQ)
        self.system = System(self.quad)
    def run(self, stop_time, deltaT, x0, u0, setpoint):
        import time
        t = np.arange(0, stop_time, deltaT)
        x = np.zeros((t.size, 12))
        x[0] = x0
        z_prev = x0[3:6]
        ref_prev = u0
        u_prev = ref_prev
        u_saturated_prev = u_prev
        for i, t_i in enumerate(t[1:], 0):
            if (i % self.MODULO_FACTOR) == 0:
                delta_x0, delta_u0 = self.mpc_input_converter(x[i, :6], ref_prev)
                ref = self.position_controller.predict(delta_x0)
                ref_converted = self.mpc_output_converter(ref, throttle=False)
                u_saturated = self.ramp_saturation(ref_converted, u_saturated_prev)
                if self.adaptive_controller is not None:
                    u_saturated_prev = u_saturated
                else:
                    u_saturated_converted = self.mpc_output_converter.convert_throttle(u_saturated)
                    mpc_u = u_saturated_converted
            if (self.adaptive_controller is not None and
                    isinstance(self.adaptive_controller.predictor.ref_model, QuadTranslationalDynamicsUncertain)):
                z = x[i, 3:6]
                z_prev = x[i - 1, 3:6]
                u = u_saturated
                u_composite = self.adaptive_controller(z, z_prev,
                                                       np.concatenate([u, np.array([0])]),
                                                       np.concatenate([u_saturated_prev, np.array([0])]), t_i)
                mpc_u = self.mpc_output_converter.convert_throttle(u_composite)
            elif (self.adaptive_controller is not None and
                    isinstance(self.adaptive_controller.predictor.ref_model, LinearQuadUncertain)):
                z = delta_x0[3:6]
                u = ref
                delta_u_composite = self.adaptive_controller(z, z_prev,
                                                            u, u_prev, t_i)
                u_prev = u
                mpc_u = self.mpc_output_converter(delta_u_composite)
                z_prev = z
            attitude_setpoint = np.concatenate([mpc_u[1:], np.array([0.0])])
            throttle = mpc_u[0]
            ESC_PWMs = self.attitude_controller(attitude_setpoint, self.quad.state[6:9], self.quad.state[9:12], throttle)
            motors = self.esc(ESC_PWMs)
            x[i + 1] = self.system(np.array(motors), deltaT, self.quad)[:12]
            if (i % self.MODULO_FACTOR) == 0:
                ref_prev = self.mpc_output_converter(ref, throttle=False)
            if self.estimator is not None:
                self.estimator(x[i + 1, :6], ref_prev)
            if isinstance(self.trajectory, type(TrajectoryWithTerminals())):
                terminal_ind = self.check_if_reached_terminal(x[i+1, :6])
                if terminal_ind is not None:
                    self.quad.mass = self.quad.nominal_mass + self.trajectory.terminals_payload[terminal_ind]
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

    def check_if_reached_terminal(self, x):
        x = x[None, :]
        terminals = self.trajectory.terminals
        terminals = np.concatenate([terminals, np.zeros((3, 3))], axis=1)
        distances = euclidean_distance(x, terminals, axis=1)
        terminal_ind = None
        if True in [dist < 1 for dist in distances]:
            terminal_ind = np.argmin(distances)
            print(terminal_ind)
        return terminal_ind


class InnerLoopSITL(SoftwareInTheLoop):
    def __init__(self, quad, load, attitude_controler, esc, inner_loop_freq):
        self.INNER_LOOP_FREQ = inner_loop_freq
        self.deltaT = 1/inner_loop_freq
        self.quad = quad
        self.load = load
        self.attitude_controller = attitude_controler
        self.esc = esc
    def run(self,  attitude_ref, throttle, stop_time, x0, u0):
        t = np.arange(0, stop_time, self.deltaT)
        x = np.zeros((t.size, 12))
        u = np.zeros((t.size, 4))
        x[0] = x0
        u[0] = u0
        for i, t_i in enumerate(t[1:], 1):
            ESC_PWMs = self.attitude_controller(attitude_ref, self.quad.state[6:9], self.quad.state[9:12],
                                                throttle)
            motors = self.esc(ESC_PWMs)
            x[i] = system(np.array(motors), self.deltaT, self.quad, self.load)[:12]
            u[i] = motors
        self.u = u
        self.attitude_ref = attitude_ref
        self.trajectory = x
        self.time = t
    def plot_attitude_trajectory(self, tested_variable):
        fig = make_subplots(rows=3, cols=1,x_title='Czas [s]',
                            subplot_titles=('Przebieg czasowy położenia kątowego',  'Przebieg czasowy prędkości kątowej',
                                            'Przebieg czasowy sygnału sterującego'))
        if tested_variable==0:
            fig['layout']['yaxis']['title'] = 'φ [rad]'
            fig['layout']['yaxis2']['title'] = 'ω_x [rad/s]'
            fig['layout']['yaxis3']['title'] = 'ω_1,2,3,4 [rad/s]'
            fig.add_trace(go.Scatter(x=self.time, y=self.trajectory[:, 6], name='φ [rad]', line = dict(width=3)), row=1, col=1)
            fig.add_trace(go.Scatter(x=self.time, y=self.trajectory[:, 9], name='ω_x [rad/s]', line = dict(width=3)), row=2, col=1)
            fig.add_trace(go.Scatter(x=self.time, y=np.repeat([self.attitude_ref[0]], self.time.shape[0]), name='φ_ref [rad]', line = dict(dash = 'dash', color='rgb(255, 0, 0)', width=3)), row=1, col=1)
            fig.add_trace(go.Scatter(x=self.time, y=self.u[:, 0], name='ω_1 [rad/s]', line=dict(width=3, color='rgb(0, 0, 255)')), row=3, col=1)

            fig.add_trace(go.Scatter(x=self.time, y=self.u[:, 1], name='ω_2 [rad/s]', line=dict(width=3, dash='dash', color='rgb(255, 255, 0)')),
                          row=3, col=1)
            fig.add_trace(go.Scatter(x=self.time, y=self.u[:, 2], name='ω_3 [rad/s]', line=dict(width=3, color='rgb(255, 150, 0)')),
                          row=3, col=1)
            fig.add_trace(go.Scatter(x=self.time, y=self.u[:, 3], name='ω_4 [rad/s]', line=dict(width=3, dash='dash', color='rgb(100, 100, 100)')),
                          row=3, col=1)
        elif tested_variable==1:
            fig['layout']['yaxis']['title'] = 'θ [rad]'
            fig['layout']['yaxis2']['title'] = 'ω_y [rad/s]'
            fig['layout']['yaxis3']['title'] = 'ω_1,2,3,4 [rad/s]'
            fig.add_trace(go.Scatter(x=self.time, y=self.trajectory[:, 7], name='θ [rad]', line=dict(width=3)), row=1,
                          col=1)
            fig.add_trace(go.Scatter(x=self.time, y=self.trajectory[:, 10], name='ω_y [rad/s]', line=dict(width=3)),
                          row=2, col=1)
            fig.add_trace(go.Scatter(x=self.time, y=np.repeat([self.attitude_ref[1]], self.time.shape[0]), name='θ_ref [rad]', line = dict(dash = 'dash', color='rgb(255, 0, 0)', width=3)), row=1, col=1)
            fig.add_trace(go.Scatter(x=self.time, y=self.u[:, 0], name='ω_1 [rad/s]', line=dict(width=3, color='rgb(0, 0, 255)')), row=3, col=1)
            fig.add_trace(go.Scatter(x=self.time, y=self.u[:, 1], name='ω_2 [rad/s]', line=dict(width=3, color='rgb(255, 255, 0)')), row=3, col=1)
            fig.add_trace(go.Scatter(x=self.time, y=self.u[:, 2], name='ω_3 [rad/s]', line=dict(width=3,dash='dash', color='rgb(255, 150, 0)')),
                          row=3, col=1)
            fig.add_trace(go.Scatter(x=self.time, y=self.u[:, 3], name='ω_4 [rad/s]', line=dict(width=3, dash='dash', color='rgb(100, 100, 100)')),
                          row=3, col=1)
        elif tested_variable==2:
            fig['layout']['yaxis']['title'] = 'ψ [rad]'
            fig['layout']['yaxis2']['title'] = 'ω_z [rad/s]'
            fig['layout']['yaxis3']['title'] = 'ω_1,2,3,4 [rad/s]'
            fig.add_trace(go.Scatter(x=self.time, y=self.trajectory[:, 8], name='ψ [rad]', line=dict(width=3)), row=1,
                          col=1)
            fig.add_trace(go.Scatter(x=self.time, y=self.trajectory[:, 11], name='ω_z [rad/s]', line=dict(width=3)), row=2,
                          col=1)
            fig.add_trace(
                go.Scatter(x=self.time, y=np.repeat([self.attitude_ref[2]], self.time.shape[0]), name='ψ_ref [rad]',
                           line=dict(dash='dash', color='rgb(255, 0, 0)', width=3)), row=1, col=1)

            fig.add_trace(go.Scatter(x=self.time, y=self.u[:, 0], name='ω_1 [rad/s]', line=dict(width=3, color = 'rgb(0, 0, 255)')), row=3, col=1)
            fig.add_trace(go.Scatter(x=self.time, y=self.u[:, 1], name='ω_2 [rad/s]', line=dict(width=3, color='rgb(255, 255, 0)')), row=3, col=1)
            fig.add_trace(go.Scatter(x=self.time, y=self.u[:, 2], name='ω_3 [rad/s]', line=dict(width=3, dash='dash', color='rgb(255, 150, 0)')), row=3, col=1)
            fig.add_trace(go.Scatter(x=self.time, y=self.u[:, 3], name='ω_4 [rad/s]', line=dict(width=3, dash='dash', color='rgb(100, 100, 100)')), row=3, col=1)

        fig.write_image('../images/yaw_trajectory.jpeg')