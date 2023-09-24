import pandas as pd
import numpy as np
import ast
import plotly.graph_objects as go
from scipy import stats
import os
import csv
from datetime import datetime
import plotly.express as px
class DataPlotter():
    def __init__(self, path,
                 beggining_id=0):
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
        self.beggining_id = beggining_id

    def get_acceleration(self):
        t = self.df['time'][self.beggining_id:]
        x = self.df['x'][self.beggining_id:]
        throttle = self.df['throttle'][10]
        data = np.array([ast.literal_eval(row) for row in x])
        velocity = data[:, 3:]
        velocity_norm = np.linalg.norm(velocity, axis=1) * np.sign(velocity[:, 2])
        slope, intercept, r_value, p_value, std_err = stats.linregress(t, velocity_norm)
        reg_y = slope*t + intercept
        fig = go.Figure(layout={'title': self.path})
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

class ThrottleThrustCharacteristicsMastTest:
    def __init__(self, data_path, max_idx=11):
        self.data_path = data_path
        self.g = 9.81
        self.df = pd.read_csv(data_path, delimiter=';', decimal=',')
        self.max_idx = max_idx

    def calculate_thrust(self, mass):
        thrust = list(np.array(mass) * self.g)
        return thrust

    def calculate_characteristics(self):
        mass = list(self.df['mass'])[:self.max_idx]
        throttle = list(self.df['throttle'])[:self.max_idx]
        thrust = self.calculate_thrust(mass)
        slope, intercept, r_value, p_value, std_err = stats.linregress(throttle, thrust)
        reg_y = slope * np.array(throttle) + intercept
        fig = go.Figure()
        fig.add_trace(go.Scatter(x=throttle, y=thrust))
        fig.add_trace(go.Scatter(x=throttle, y=reg_y))
        fig.show()
        print("Equation: thrust = {} * throttle + {}".format(slope, intercept))

class ThrottleThrustTestStand:
    def __init__(self, data_path, min_idx=4, max_idx=18):
        self.data_path = data_path
        self.g = 9.81
        self.df = pd.read_csv(data_path, delimiter=';', decimal=',')
        self.max_idx = max_idx
        self.min_idx = min_idx

    def calculate_thrust(self, thrust_in_g):
        thrust = list((np.array(thrust_in_g) * self.g / 1000))
        return thrust

    def calculate_characteristics(self):
        thrust_g = list(self.df['thrust'])[self.min_idx:self.max_idx]
        throttle = list(self.df['throttle'])[self.min_idx:self.max_idx]
        thrust = self.calculate_thrust(thrust_g)
        slope, intercept, r_value, p_value, std_err = stats.linregress(throttle, thrust)
        reg_y = slope * np.array(throttle) + intercept
        fig = go.Figure()
        fig.add_trace(go.Scatter(x=throttle, y=thrust))
        fig.add_trace(go.Scatter(x=throttle, y=reg_y))
        fig.show()
        print("Equation: thrust = {} * throttle + {}".format(slope, intercept))

if __name__ == "__main__":
    # print(os.getcwd())
    #throttle_thrust_csv_path = './identification_data/test_stand/z550.csv'
    throttle_thrust_csv_path = './identification_data/throttle_mass_equillibrium_data/iris.csv'
    #
    # collect throttle - acceleration data

    # csv_files_path = './identification_logs/all_zd550/'
    # files = os.listdir(csv_files_path)
    # throttle_list = []
    # acceleration_list = []
    # for file in files:
    #     path = csv_files_path + file
    #     data_plotter = DataPlotter(path, beggining_id=30)
    #     a, throttle = data_plotter.get_acceleration()
    #     throttle_list.append(throttle)
    #     acceleration_list.append(a)
    # data_pairs = [(throttle, acceleration) for throttle, acceleration in sorted(zip(throttle_list, acceleration_list))]
    # throttle_list = [throttle for throttle, acceleration in data_pairs]
    # acceleration_list = [acceleration for throttle, acceleration in data_pairs]
    # print(throttle_list)
    # print(acceleration_list)
    # fig = go.Figure()
    # fig.add_trace(go.Scatter(x=throttle_list, y=acceleration_list))
    # fig.show()
    # with open(throttle_thrust_csv_path + datetime.now().strftime("%m-%d-%Y-%H:%M:%S") + '_thrust_throttle.csv', 'w') as f:
    #     writer = csv.writer(f)
    #     writer.writerow(['throttle', 'acceleration'])
    #     for data in data_pairs:
    #         writer.writerow(data)

    # calculate characteristics
    # #
    # throttle_thrust = ThrottleThrustCharacteristics(data_path=throttle_thrust_csv_path + 'thrust_throttle.csv',
    #                                                 mass=1.628,
    #                                                 max_idx=10)
    throttle_thrust = ThrottleThrustCharacteristicsMastTest(data_path=throttle_thrust_csv_path)
    #throttle_thrust = ThrottleThrustTestStand(data_path=throttle_thrust_csv_path)
    throttle_thrust.calculate_characteristics()