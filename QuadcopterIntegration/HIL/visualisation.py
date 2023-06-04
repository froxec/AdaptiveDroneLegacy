import dash.dependencies
from dash import Dash, html, dcc, Input, Output
from simulation_parameters import DB_PARAMETERS
from Factories.CommunicationFactory.redis_db_commands import redis_stream_read_last, redis_stream_add
import pickle
import plotly.express as px
from plotly.subplots import make_subplots
import plotly.graph_objects as go
import numpy as np
import redis

input_names = {'xref', 'yref', 'zref'}

dashboard : object = None

## dash app instantiation
app = Dash(__name__)
app.layout = html.Div([
    html.H3('Quadcopter dashboard'),
    html.Div([
        dcc.Input(
            id="{}".format(input_name),
            type="number",
            value=0.0,
            placeholder="{}".format(input_name)
        ) for input_name in input_names]),
    html.Button('Update reference', id='submit-val', n_clicks=0),
    html.Div([html.H3('Current setpoint'), html.P(id="ref_holder")]),
    # dcc.Dropdown(
    #     id='my-dropdown',
    #     options=[{'label': 'State', 'value': 'STATE'},
    #              {'label': 'Control', 'value': 'CONTROL'}],
    #     value='STATE'
    # ),
    dcc.Graph(id="state_graph"),
    dcc.Graph(id="control_graph"),
    dcc.Graph(id='3d-view'),
    dcc.Interval(
        id='interval-component',
        interval=1000,
        n_intervals=0
    ),
    html.P(id='placeholder')
])


@app.callback(
    dash.dependencies.Output("placeholder", "title"),
    [dash.dependencies.Input("interval-component", "n_intervals")],
)
def get_data(value):
    dashboard.get_data()

@app.callback(
    dash.dependencies.Output("state_graph", "figure"),
    [dash.dependencies.Input("interval-component", "n_intervals")],
)
def update_graph(value):
    fig = dashboard.update_graph('state')
    return fig

@app.callback(
    dash.dependencies.Output("control_graph", "figure"),
    [dash.dependencies.Input("interval-component", "n_intervals")],
)
def update_graph(value):
    fig = dashboard.update_graph('control')
    return fig

@app.callback(dash.dependencies.Output("ref_holder", "children"),
              Input('submit-val', 'n_clicks'),
              [dash.dependencies.State("xref", "value"),
               dash.dependencies.State("yref", "value"),
               dash.dependencies.State("zref", "value")])
def update_ref(n_clicks, xref, yref, zref):
    dashboard.update_ref([xref, yref, zref])
    return "x: {}, y: {}, z :{}".format(xref, yref, zref)

class Dashboard():
    def __init__(self):
        self.database = redis.Redis(host=DB_PARAMETERS['ip'], port=DB_PARAMETERS['port'],
                               decode_responses=DB_PARAMETERS['decode_responses'])
        self.data_x = []
        self.data_u = []

    def run(self, debug=True):
        # set global module as the Dashboard instance
        global dashboard
        assert dashboard is None, "Cannot instantiate more than one dashboard."
        dashboard = self

        app.run_server(debug=debug)

    def get_data(self):
        x = redis_stream_read_last(redis_db=self.database, stream='x')
        u = redis_stream_read_last(redis_db=self.database, stream='u')
        if x is not None:
            self.data_x.append(x)
        if u is not None:
            self.data_u.append(u)
        return None
    def update_graph(self, mode):
        if mode == 'state' and len(self.data_x) > 0:
            fig = self.draw_subplots(self.data_x, 4, 3)
        elif mode == 'control' and len(self.data_u) > 0:
            fig = self.draw_subplots(self.data_u, 3, 1)
        else:
            fig = None
        return fig

    def draw_subplots(self, data, rows, cols):
        fig = make_subplots(rows, cols)
        data = np.array(data).reshape((-1, rows, cols))

        for i in range(rows):
            for j in range(cols):
                fig.add_trace(go.Scatter(x=list(range(data.shape[0])), y=data[:, i, j]),row=i+1, col=j+1)

        return fig
    def update_ref(self, ref):
        redis_stream_add(redis_db=self.database,
                         stream='ref',
                         data=ref)
if __name__ == '__main__':
    dashboard_primitive = Dashboard()
    dashboard_primitive.run()