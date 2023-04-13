from gekko import GEKKO
import numpy as np
from Factories.ModelsFactory.linear_models import LinearizedQuadNoYaw
from Factories.ModelsFactory.model_parameters import Z550_parameters
import plotly.graph_objects as go
from plotly.subplots import make_subplots
class gekkoMPC():
    def __init__(self, model, control_horizon, outer_loop_freq):
        self.model = model
        self.model.discretize_model(1/outer_loop_freq)
        self.x_bounds = {'lower': [-np.Inf, -np.Inf, -np.Inf, -2.5, -2.5, -2.5],
                    'upper': [np.Inf, np.Inf, np.Inf, 2.5, 2.5, 2.5]}
        self.u_bounds = {'lower': [-model.parameters['m']*model.parameters['g'], -np.pi/6, -np.pi/6],
                         'upper': [model.parameters['m']*model.parameters['g'], np.pi/6, np.pi/6]} #poprawić maksymalne ograniczenie na ciąg
        self.setpoint = None
        self.costs = {'delta_mv_cost': [0, 0, 0],
                      'delta_mv_max': [np.Inf, np.Inf, np.Inf]}
        self.m = GEKKO(remote=False)
        self.x, self.y, self.u = self.m.state_space(self.model.Ad, self.model.Bd, self.model.C, D=None, discrete=True)
        self.m.time = np.arange(0, control_horizon, 1/outer_loop_freq)
        #Manipulated variable
        # self.u = [self.m.MV(value=self.u0[i], lb=self.u_bounds['lower'][i], ub=self.u_bounds['upper'][i]) for i in range(len(self.u0))]
        # for i, var in enumerate(self.u):
        #     var.STATUS = 1
        #     var.DCOST = self.costs['delta_mv_cost'][i]
        #     var.DMAX = self.costs['delta_mv_max'][i]
        # #Controled variable
        # self.x = [self.m.CV(value=self.x0[i], lb=self.x_bounds['lower'][i], ub=self.x_bounds['upper'][i]) for i in range(len(self.x0))]
        # for i, var in enumerate(self.x):
        #     var.STATUS = 1
        #     var.DCOST = self.costs['delta_mv_cost'][i]
        #     var.DMAX = self.costs['delta_mv_max'][i]
        #     var.SP = self.setpoint[i]

        for i, var in enumerate(self.u):
            var.LOWER = self.u_bounds['lower'][i]
            var.UPPER = self.u_bounds['upper'][i]
            var.DCOST = self.costs['delta_mv_cost'][i]
            var.DMAX = self.costs['delta_mv_max'][i]
            var.STATUS = 1

        for i, var in enumerate(self.y):
            var.STATUS = 1
        self.m.options.CV_TYPE = 2
        self.m.options.imode = 6
        self.m.options.nodes = 2
    def predict(self, y0, u0, setpoint):
        self.setpoint = setpoint
        self.y0 = y0
        self.u0 = u0
        for i, var in enumerate(self.x):
            var.VALUE = y0[i]
        for i, var in enumerate(self.y):
            var.SP = self.setpoint[i]
            var.VALUE= y0[i]
            var.TR_INIT = y0[i]
        for i, var in enumerate(self.u):
            var.VALUE = self.u0[i]
        self.m.solve()
        return self.u, self.x, self.y
    def plot(self):
        u_fig = make_subplots(rows=3, cols=1)
        u_fig.add_trace(go.Scatter(x=self.m.time, y=list(self.u[0]), name='Thrust [N]'), row=1, col=1)
        u_fig.add_trace(go.Scatter(x=self.m.time, y=list(self.u[1]), name='Roll [rad]'), row=2, col=1)
        u_fig.add_trace(go.Scatter(x=self.m.time, y=list(self.u[2]), name='Pitch [rad]'), row=3, col=1)

        y_fig = make_subplots(rows=3, cols=2)
        y_fig.add_trace(go.Scatter(x=self.m.time, y=list(self.y[0]), name='delta_x [N]'), row=1, col=1)
        y_fig.add_trace(go.Scatter(x=self.m.time, y=list(self.y[1]), name='delta_y [rad]'), row=2, col=1)
        y_fig.add_trace(go.Scatter(x=self.m.time, y=list(self.y[2]), name='delta_z [rad]'), row=3, col=1)
        y_fig.add_trace(go.Scatter(x=self.m.time, y=list(self.y[3]), name='deltaVx [N]'), row=1, col=2)
        y_fig.add_trace(go.Scatter(x=self.m.time, y=list(self.y[4]), name='deltaVy [rad]'), row=2, col=2)
        y_fig.add_trace(go.Scatter(x=self.m.time, y=list(self.y[5]), name='deltaBz [rad]'), row=3, col=2)
        u_fig.show()
        y_fig.show()

if __name__ == "__main__":
    model = LinearizedQuadNoYaw(Z550_parameters)
    mpc = gekkoMPC(model, 2, 5)
    mpc.predict([0, 0, 50, 0, 0, 0], [0, 0, 0], [0, 0, 100, 0, 0, 0])
    mpc.plot()