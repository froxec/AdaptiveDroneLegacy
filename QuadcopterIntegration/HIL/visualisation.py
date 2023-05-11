import dash.dependencies
from dash import Dash, html, dcc, Input, Output
from simulation_parameters import DB_PARAMETERS
from Factories.CommunicationFactory.redis_db_commands import redis_stream_read_last
import pickle
import plotly.express as px
from plotly.subplots import make_subplots
import numpy as np
import redis

database = redis.Redis(host=DB_PARAMETERS['ip'], port=DB_PARAMETERS['port'], decode_responses=DB_PARAMETERS['decode_responses'])
data_x = []
data_u = []
app = Dash(__name__)

app.layout = html.Div([
    html.H3('Quadcopter dashboard'),
    dcc.Dropdown(
        id='my-dropdown',
        options=[{'label': 'quad_data', 'value': '0'},
                 {'label': 'Tesla', 'value': 'TSLA'}],
        value='0'
    ),
    dcc.Graph(id="graph"),
    dcc.Interval(
        id='interval-component',
        interval=500,
        n_intervals=0
    )
])

@app.callback(
    dash.dependencies.Output("graph", "figure"),
    [dash.dependencies.Input("interval-component", "n_intervals")],
)
def update_graph(file):
    x = redis_stream_read_last(redis_db=database, stream='x')
    u = redis_stream_read_last(redis_db=database, stream='u')
    data_x.append(x)
    data_u.append(u)

#     return fig
#
# def draw_subplots(data, rows, cols):
#     fig = make_subplots(4, 3)
#
#     for i in range(rows):
#         for j in range (cols):

if __name__ == '__main__':
    app.run_server(debug=True)