import numpy as np
from Simulation.model import system
import plotly.graph_objects as go
from plotly.subplots import make_subplots
class SoftwareInTheLoop:
    def __init__(self, quad, load, position_controller, attitude_controler, mpc_output_converter, esc, inner_loop_freq, outer_loop_freq):
        self.INNER_LOOP_FREQ = inner_loop_freq
        self.OUTER_LOOP_FREQ = outer_loop_freq
        self.MODULO_FACTOR = self.INNER_LOOP_FREQ / self.OUTER_LOOP_FREQ
        self.quad = quad
        self.load = load
        self.position_controller = position_controller
        self.attitude_controller = attitude_controler
        self.mpc_output_converter = mpc_output_converter
        self.esc = esc
    def run(self, stop_time, deltaT, x0):
        t = np.arange(0, stop_time, deltaT)
        x = np.zeros((t.size, 12))
        x[0] = x0
        for i, t_i in enumerate(t[1:], 1):
            if (i % self.MODULO_FACTOR) == 1:
                ref = self.position_controller.update_state_control(x[i - 1, :6])
                ref_converted = self.mpc_output_converter(ref)
                attitude_setpoint =np.concatenate([ref_converted[1:], np.array([0])])
                throttle = ref_converted[0]
            ESC_PWMs = self.attitude_controller(attitude_setpoint, self.quad.state[6:9], self.quad.state[9:12], throttle)
            motors = self.esc(ESC_PWMs)
            x[i] = system(np.array(motors), deltaT, self.quad, self.load)[:12]
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

class InnerLoopSITL(SoftwareInTheLoop):
    def __init__(self, quad, load, attitude_controler, esc, inner_loop_freq):
        self.INNER_LOOP_FREQ = inner_loop_freq
        self.deltaT = 1/inner_loop_freq
        self.quad = quad
        self.load = load
        self.attitude_controller = attitude_controler
        self.esc = esc
    def run(self,  attitude_ref, throttle, stop_time, x0):
        t = np.arange(0, stop_time, self.deltaT)
        x = np.zeros((t.size, 12))
        x[0] = x0
        for i, t_i in enumerate(t[1:], 1):
            ESC_PWMs = self.attitude_controller(attitude_ref, self.quad.state[6:9], self.quad.state[9:12],
                                                throttle)
            motors = self.esc(ESC_PWMs)
            x[i] = system(np.array(motors), self.deltaT, self.quad, self.load)[:12]
        self.trajectory = x
        self.time = t
        self.plot_attitude_trajectory()
    def plot_attitude_trajectory(self):
        fig = make_subplots(rows=3, cols=2)

        fig.add_trace(go.Scatter(x=self.time, y=self.x[:, 6]), row=1, col=1)
        fig.add_trace(go.Scatter(x=self.time, y=self.x[:, 9]), row=1, col=2)

        fig.add_trace(go.Scatter(x=self.time, y=self.x[:, 7]), row=2, col=1)
        fig.add_trace(go.Scatter(x=self.time, y=self.x[:, 10]), row=2, col=2)

        fig.add_trace(go.Scatter(x=self.time, y=self.x[:, 8]), row=3, col=1)
        fig.add_trace(go.Scatter(x=self.time, y=self.x[:, 11]), row=3, col=2)