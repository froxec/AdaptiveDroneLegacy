import numpy as np
import plotly.graph_objects as go
class ThrustCompensator():
    def __init__(self, parameters, u_ss, model_parameters, prediction_model, state0, thrust0, deltaT):
        self.Kp = parameters['Kp']
        self.Ki = parameters['Ki']
        self.Kd = parameters['Kd']
        self.prediction_model = prediction_model(model_parameters)
        self.state_prev = state0
        self.deltaT = deltaT
        self.u_prev = thrust0 + u_ss
        self.u_ss = u_ss
        self.error_prev = 0
        self.P = 0
        self.I = 0
        self.D = 0
        self.history = {'P': [],
                        'I': [],
                        'D': []}
    def __call__(self, state, u):
        state_predicted = self.prediction_model.discrete_prediction(self.state_prev, self.u_prev, self.deltaT)
        acceleration = self.calculate_acceleration(state)
        acceleration_hat = self.calculate_acceleration(state_predicted)
        error = acceleration_hat - acceleration
        self.state_prev = state
        self.u_prev = u + self.u_ss
        self.P = self.Kp * error
        self.I = self.I + error*self.Ki*self.deltaT
        self.D = ((error - self.error_prev)/self.deltaT)*self.Kd
        u[0] = u[0] + self.P + self.I + self.D
        self.history['P'].append(self.P)
        self.history['I'].append(self.I)
        self.history['D'].append(self.D)
        self.error_prev = error
        return u

    def calculate_acceleration(self, state):
        vz_prev = self.state_prev[5]
        vz = state[5]
        acceleration = (vz - vz_prev)/self.deltaT
        return acceleration

    def plot_signals(self, t):
        fig = go.Figure()
        fig.add_trace(go.Scatter(x=t ,y=self.history['P']))
        fig.add_trace(go.Scatter(x=t, y=self.history['I']))
        fig.add_trace(go.Scatter(x=t, y=self.history['D']))
        fig.show()