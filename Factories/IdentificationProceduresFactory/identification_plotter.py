import pandas as pd
import numpy as np
import ast
import plotly.graph_objects as go
from scipy import stats
import os
import csv
from datetime import datetime
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

    def get_acceleration(self):
        t = self.df['time']
        x = self.df['x']
        throttle = self.df['throttle'][10]
        data = np.array([ast.literal_eval(row) for row in x])
        velocity = data[:, 3:]
        velocity_norm = np.linalg.norm(velocity, axis=1) * np.sign(velocity[:, 2])
        slope, intercept, r_value, p_value, std_err = stats.linregress(t, velocity_norm)
        reg_y = slope*t + intercept
        fig = go.Figure()
        fig.add_trace(go.Scatter(x=t, y=velocity_norm))
        fig.add_trace(go.Scatter(x=t, y=reg_y))
        fig.show()
        return slope, throttle

class ThrottleThrustCharacteristics:
    def __init__(self, data_path, mass, max_idx=11):
        self.data_path = data_path
        self.mass = mass
        self.g = 9.81
        self.df = pd.read_csv(data_path)
        self.max_idx = max_idx

    def calculate_thrust(self, acceleration):
        thrust = list(self.mass * (np.array(acceleration) + self.g))
        return thrust

    def calculate_characteristics(self):
        acceleration = list(self.df['acceleration'])[:self.max_idx]
        throttle = list(self.df['throttle'])[:self.max_idx]
        thrust = self.calculate_thrust(acceleration)
        slope, intercept, r_value, p_value, std_err = stats.linregress(throttle, thrust)
        reg_y = slope * np.array(throttle) + intercept
        fig = go.Figure()
        fig.add_trace(go.Scatter(x=throttle, y=thrust))
        fig.add_trace(go.Scatter(x=throttle, y=reg_y))
        fig.show()
        print("Equation: thrust = {} * throttle + {}".format(slope, intercept))
if __name__ == "__main__":
    print(os.getcwd())
    throttle_thrust_csv_path = './identification_data/throttle_thrust_data/iris_rpi/'

    # collect throttle - acceleration data

    csv_files_path = './identification_logs/rpi_iris/'
    files = os.listdir(csv_files_path)
    throttle_list = []
    acceleration_list = []
    for file in files:
        path = csv_files_path + file
        data_plotter = DataPlotter(path)
        a, throttle = data_plotter.get_acceleration()
        throttle_list.append(throttle)
        acceleration_list.append(a)
    data_pairs = [(throttle, acceleration) for throttle, acceleration in sorted(zip(throttle_list, acceleration_list))]
    throttle_list = [throttle for throttle, acceleration in data_pairs]
    acceleration_list = [acceleration for throttle, acceleration in data_pairs]
    print(throttle_list)
    print(acceleration_list)
    fig = go.Figure()
    fig.add_trace(go.Scatter(x=throttle_list, y=acceleration_list))
    fig.show()
    with open(throttle_thrust_csv_path + datetime.now().strftime("%m-%d-%Y-%H:%M:%S") + '_thrust_throttle.csv', 'w') as f:
        writer = csv.writer(f)
        writer.writerow(['throttle', 'acceleration'])
        for data in data_pairs:
            writer.writerow(data)

    # calculate characteristics

    # throttle_thrust = ThrottleThrustCharacteristics(data_path=throttle_thrust_csv_path + 'thrust_throttle.csv',
    #                                                 mass=1.5,
    #                                                 max_idx=6)
    # throttle_thrust.calculate_characteristics()