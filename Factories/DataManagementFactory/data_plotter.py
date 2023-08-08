import plotly.graph_objects as go
from plotly.subplots import make_subplots
import pandas as pd
import datetime
class DataPlotter():
    def __init__(self, path):
        self.path = path
        self.df = pd.read_csv(path)
        self.font_dict = dict(family='Arial',
                         size=26,
                         color='black'
                         )
        self.layout = {'font': self.font_dict,  # font formatting
                          'plot_bgcolor':'white',  # background color
                          'width' :850,  # figure width
                          'height':700,  # figure height
                          'margin': dict(r=20, t=20, b=10)  # remove white space
                       }
        self.axeslayout = { # axis label
                         'showline':True,  # add line at x=0
                         'linecolor':'black',  # line color
                         'linewidth':2.4,  # line size
                         'ticks':'outside',  # ticks outside axis
                         'tickfont':self.font_dict,  # tick label font
                         'mirror':'allticks',  # add ticks to top/right axes
                         'tickwidth':2.4,  # tick width
                         'tickcolor':'black',  # tick color
                         }
    def plot_velocity(self):
        column_names = ['VELOCITY:X', 'VELOCITY:Y', 'VELOCITY:Z']
        self._plotting_3rows(column_names)

    def plot_position_local(self):
        column_names = ['POSITION_LOCAL:X', 'POSITION_LOCAL:Y', 'POSITION_LOCAL:Z']
        self._plotting_3rows(column_names)

    def plot_attitude(self):
        column_names = ['ATTITUDE:X', 'ATTITUDE:Y', 'ATTITUDE:Z']
        self._plotting_3rows(column_names)

    def plot_output_control(self):
        column_names = ['U_OUTPUT:X', 'U_OUTPUT:Y', 'U_OUTPUT:Z']
        self._plotting_3rows(column_names)

    def plot_u_l1(self):
        column_names = ['u_l1:X', 'u_l1:Y', 'u_l1:Z']
        self._plotting_3rows(column_names)

    def plot_sigma(self):
        column_names = ['SIGMA:X', 'SIGMA:Y', 'SIGMA:Z']
        self._plotting_3rows(column_names)

    def plot_u_ref(self):
        column_names = ['U_REF:X', 'U_REF:Y', 'U_REF:Z']
        self._plotting_3rows(column_names)

    def _plotting_3rows(self, column_names):
        t = pd.to_datetime(self.df['TIME'])
        t = t - t[0]
        t = t.dt.total_seconds()
        data = self.df[column_names]
        fig = make_subplots(rows=3, cols=1)
        fig.update_layout(**self.layout)
        fig.update_xaxes(**self.axeslayout)
        fig.update_yaxes(**self.axeslayout)
        for i, col in enumerate(data, 1):
            fig.add_trace(go.Scatter(x=t, y=data[col]), row=i, col=1)
        fig.show()
if __name__ == "__main__":
    import os
    TEST_NAME = 'TEST3.csv'
    cwd = os.getcwd()
    dir = os.listdir()
    candidates = []
    for i in range(len(dir)):
        name = dir[i].split("_")[1]
        if name == TEST_NAME:
            candidates.append(dir[i])
    print(candidates)
    path = candidates[0]
    print(os.listdir(os.getcwd()))
    data_plotter = DataPlotter(path)
    data_plotter.plot_position_local()
    data_plotter.plot_output_control()
    data_plotter.plot_u_l1()
    data_plotter.plot_u_ref()