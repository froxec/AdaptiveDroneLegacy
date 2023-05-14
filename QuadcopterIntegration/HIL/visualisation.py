import dash.dependencies
from dash import Dash, html, dcc, Input, Output
from simulation_parameters import DB_PARAMETERS
from Factories.CommunicationFactory.redis_db_commands import redis_stream_read_last
import pickle
import plotly.express as px
from plotly.subplots import make_subplots
import plotly.graph_objects as go
import numpy as np
import redis

class Dashboard():
    app = Dash(__name__)
    app.layout = html.Div([
        html.H3('Quadcopter dashboard'),
        dcc.Dropdown(
            id='my-dropdown',
            options=[{'label': 'State', 'value': 'STATE'},
                     {'label': 'Control', 'value': 'CONTROL'}],
            value='STATE'
        ),
        dcc.Graph(id="state_graph"),
        dcc.Graph(id='3d-view'),
        dcc.Interval(
            id='interval-component',
            interval=1000,
            n_intervals=0
        ),
        html.P(id='placeholder')
    ])
    def __init__(self):
        self.database = redis.Redis(host=DB_PARAMETERS['ip'], port=DB_PARAMETERS['port'],
                               decode_responses=DB_PARAMETERS['decode_responses'])
        self.data_x = []
        self.data_u = []

    @app.callback(
        dash.dependencies.Output("placeholder", "title"),
        [dash.dependencies.Input("interval-component", "n_intervals")],
    )
    def get_data(self):
        x = redis_stream_read_last(redis_db=self.database, stream='x')
        u = redis_stream_read_last(redis_db=self.database, stream='u')
        self.data_x.append(x)
        self.data_u.append(u)

        return None
    @app.callback(
        dash.dependencies.Output("state_graph", "figure"),
        [dash.dependencies.Input("interval-component", "n_intervals")],
    )
    def update_graph(self):
        fig = self.draw_subplots(self.data_x, 4, 3)

        return fig

    def draw_subplots(self, data, rows, cols):
        fig = make_subplots(rows, cols)
        data = np.array(data).reshape((-1, 4, 3))

        for i in range(rows):
            for j in range(cols):
                fig.add_trace(go.Scatter(x=list(range(data.shape[0])), y=data[:, i, j]),row=i+1, col=j+1)

        return fig

if __name__ == '__main__':
    dashboard = Dashboard()
    dashboard.app.run_server(debug=True)