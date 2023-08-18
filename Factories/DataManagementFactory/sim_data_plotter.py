import plotly.graph_objects as go
from plotly.subplots import make_subplots
import pandas as pd
import plotly
import datetime
class DataPlotter():
    def __init__(self, path):
        self.path = path
        self.df = pd.read_csv(path)
        self.font_dict = dict(family='Arial',
                         size=18,
                         color='#2f323f'
                         )
        self.layout = {'font': self.font_dict,  # font formatting
                          'plot_bgcolor':'white',  # background color
                          'width' :1400,  # figure width
                          'height':1100,  # figure height
                          'margin': dict(r=10, t=0, b=0),  # remove white space
                          #'showlegend': False
                       }
        self.axeslayout = { # axis label
                         'showline':True,  # add line at x=0
                         'linecolor':'#75787B',  # line color
                         'linewidth':2.4,  # line size
                         'ticks':'outside',  # ticks outside axis
                         'tickfont':self.font_dict,  # tick label font
                         'mirror':'allticks',  # add ticks to top/right axes
                         'tickwidth':2.4,  # tick width
                         'tickcolor':'#75787B',  # tick color
                         }
    def plot_velocity(self):
        column_names = ['Vx', 'Vy', 'Vz']
        self._plotting_3rows(column_names)

    def plot_position_local(self):
        column_names = ['x', 'y', 'z']
        self._plotting_3rows(column_names)

    def plot_attitude(self):
        column_names = ['theta', 'phi', 'psi']
        self._plotting_3rows(column_names)

    def plot_output_control(self):
        column_names = ['u', 'u', 'u']
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
        t = pd.to_datetime(self.df['time'])
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

    def _plotting_states(self):
        state_columns = ['x', 'y', 'z', 'Vx', 'Vy', 'Vz',
                         'phi', 'theta', 'psi', 'omega_x', 'omega_y', 'omega_z']
        y_labels = ['x [m]', 'y [m]', 'z [m]', 'Vx [m/s]', 'Vy [m/s]', 'Vz [m/s]',
                    'φ [rad]', 'θ [rad]', 'ψ [rad]', 'ωx [rad/s]', 'ωy [rad/s]', 'ωz [rad/s]']
        plot_indices = [(1, 1), (1, 2), (1, 3),
                        (2, 1), (2, 2), (2, 3),
                        (3, 1), (3, 2), (3, 3),
                        (4, 1), (4, 2), (4, 3)]
        t = self.df['time']
        data = self.df[state_columns]
        fig = make_subplots(rows=4, cols=3, horizontal_spacing=0.1)
        fig.update_layout(**self.layout)
        fig.update_xaxes(**self.axeslayout)
        fig.update_yaxes(**self.axeslayout)
        k = 0
        for col, (i, j) in zip(state_columns, plot_indices):
            fig.add_trace(go.Scatter(x=t, y=data[col], line=dict(color="#4d6e89", width=3)), row=i, col=j)
            if k > 0:
                fig['layout']['xaxis' + str(k+1)]['title'] = 't [s]'
                fig['layout']['yaxis' + str(k+1)]['title'] = y_labels[k]
                fig['layout']['yaxis' + str(k+1)]['title']['standoff'] = 0
                if  k < 7:
                    fig['layout']['yaxis' + str(k + 1)]['title']['standoff'] = 25
            else:
                fig['layout']['xaxis']['title'] = 't [s]'
                fig['layout']['yaxis']['title'] = y_labels[k]
                fig['layout']['yaxis']['title']['standoff'] = 25
            k += 1
        fig.update_layout(yaxis9=(dict(range=[-0.5, 0.5])))
        fig.update_layout(yaxis12=(dict(range=[-0.5, 0.5])))
        folders = self.path.split('/')
        path = ''
        filename = folders[-1].split('.')[0]
        for folder in folders[:-1]:
            path += folder + '/'
        fig.write_image(path + filename + '.png')

    def _plotting_controls(self):
        layout = {'font': self.font_dict,  # font formatting
                          'plot_bgcolor':'white',  # background color
                          'width' :1400,  # figure width
                          'height':900,  # figure height
                          'margin': dict(r=10, t=0, b=0),  # remove white space
                          'showlegend': False
                       }
        state_columns = ['F', 'phi_ref', 'theta_ref']
        y_labels = ['Fzad [N]', 'φzad [rad]', 'θzad [rad]']
        plot_indices = [(1, 1), (2, 1), (3, 1)]
        t = self.df['time']
        data = self.df[state_columns]
        fig = make_subplots(rows=3, cols=1, horizontal_spacing=0.1)
        fig.update_layout(**layout)
        fig.update_xaxes(**self.axeslayout)
        fig.update_yaxes(**self.axeslayout)
        k = 0
        for col, (i, j) in zip(state_columns, plot_indices):
            fig.add_trace(go.Scatter(x=t, y=data[col], line=dict(color="#4d6e89", width=3)), row=i, col=j)
            if k > 0:
                fig['layout']['xaxis' + str(k + 1)]['title'] = 't [s]'
                fig['layout']['yaxis' + str(k + 1)]['title'] = y_labels[k]
                fig['layout']['yaxis' + str(k + 1)]['title']['standoff'] = 0
            else:
                fig['layout']['xaxis']['title'] = 't [s]'
                fig['layout']['yaxis']['title'] = y_labels[k]
            k += 1
        folders = self.path.split('/')
        path = ''
        filename = folders[-1].split('.')[0]
        for folder in folders[:-1]:
            path += folder + '/'
        fig.write_image(path + filename + '_controls.png')

    def plot_mass_sensitivity_tests(self, path):
        df_min05 = pd.read_csv(path + "perturbed-05.csv")
        df_min02 = pd.read_csv(path + "perturbed-02.csv")
        df_0 = pd.read_csv(path + "perturbed-00.csv")
        df_plus02 = pd.read_csv(path + "perturbed+02.csv")
        df_plus05 = pd.read_csv(path + "perturbed+05.csv")
        tests = [df_min05, df_min02, df_0, df_plus02, df_plus05]
        legend_names = ['Δm = -0.5 kg', 'Δm = -0.2 kg', 'Δm = 0.0 kg', 'Δm = 0.2 kg', 'Δm = 0.5 kg']
        self.plot_tests_state(tests, legend_names, path)
        self.plot_tests_control(tests, legend_names, path)

    def plot_wind_tests(self, path):
        wind1 = pd.read_csv(path + "wind_100_0.csv")
        wind2 = pd.read_csv(path + "wind_100_2.csv")
        wind3 = pd.read_csv(path + "wind_100_5.csv")
        tests = [wind1, wind2, wind3]
        legend_names = ['||w|| = 0 N', '||w|| = 2 N', '||w|| = 5 N']
        self.plot_tests_state(tests, legend_names, path)
        self.plot_tests_control(tests, legend_names, path)

    def plot_tests_state(self, tests, legend_names, path):
        colors = plotly.colors.DEFAULT_PLOTLY_COLORS
        state_fig = make_subplots(rows=4, cols=3, horizontal_spacing=0.1)
        plot_indices = [(1, 1), (1, 2), (1, 3),
                        (2, 1), (2, 2), (2, 3),
                        (3, 1), (3, 2), (3, 3),
                        (4, 1), (4, 2), (4, 3)]
        state_fig.update_layout(**self.layout)
        state_fig.update_xaxes(**self.axeslayout)
        state_fig.update_yaxes(**self.axeslayout)
        state_columns = ['x', 'y', 'z', 'Vx', 'Vy', 'Vz',
                         'phi', 'theta', 'psi', 'omega_x', 'omega_y', 'omega_z']
        y_labels = ['x [m]', 'y [m]', 'z [m]', 'Vx [m/s]', 'Vy [m/s]', 'Vz [m/s]',
                    'φ [rad]', 'θ [rad]', 'ψ [rad]', 'ωx [rad/s]', 'ωy [rad/s]', 'ωz [rad/s]']
        t = tests[0]['time']
        for l, test in enumerate(tests):
            data = test
            k = 0
            for col, (i, j) in zip(state_columns, plot_indices):
                if i == 1 and j == 1:
                    legend = True
                else:
                    legend = False
                state_fig.add_trace(
                    go.Scatter(x=t, y=data[col], line=dict(color=colors[l], width=3), name=legend_names[l],
                               showlegend=legend), row=i, col=j)
                if k > 0:
                    state_fig['layout']['xaxis' + str(k + 1)]['title'] = 't [s]'
                    state_fig['layout']['yaxis' + str(k + 1)]['title'] = y_labels[k]
                    state_fig['layout']['yaxis' + str(k + 1)]['title']['standoff'] = 0
                    if k < 7:
                        state_fig['layout']['yaxis' + str(k + 1)]['title']['standoff'] = 25
                else:
                    state_fig['layout']['xaxis']['title'] = 't [s]'
                    state_fig['layout']['yaxis']['title'] = y_labels[k]
                    state_fig['layout']['yaxis']['title']['standoff'] = 25
                k += 1
            state_fig.update_layout(yaxis9=(dict(range=[-0.5, 0.5])))
            state_fig.update_layout(yaxis12=(dict(range=[-0.5, 0.5])))
        state_fig.write_image(path + '/' + 'wind_sensitivity.png')
    def plot_tests_control(self, tests, legend_names, path):
        colors = plotly.colors.DEFAULT_PLOTLY_COLORS
        control_columns = ['F', 'phi_ref', 'theta_ref']
        y_labels = ['Fzad [N]', 'φzad [rad]', 'θzad [rad]']
        plot_indices = [(1, 1), (2, 1), (3, 1)]
        layout = {'font': self.font_dict,  # font formatting
                  'plot_bgcolor': 'white',  # background color
                  'width': 1400,  # figure width
                  'height': 900,  # figure height
                  'margin': dict(r=10, t=0, b=0),  # remove white space
                  }
        control_fig = make_subplots(rows=3, cols=1, horizontal_spacing=0.1)
        control_fig.update_layout(**layout)
        control_fig.update_xaxes(**self.axeslayout)
        control_fig.update_yaxes(**self.axeslayout)
        t = tests[0]['time']
        for l, test in enumerate(tests):
            data = test
            k = 0
            for col, (i, j) in zip(control_columns, plot_indices):
                if i == 1 and j == 1:
                    legend = True
                else:
                    legend = False
                control_fig.add_trace(
                    go.Scatter(x=t, y=data[col], line=dict(color=colors[l], width=3), name=legend_names[l],
                               showlegend=legend), row=i, col=j)
                if k > 0:
                    control_fig['layout']['xaxis' + str(k + 1)]['title'] = 't [s]'
                    control_fig['layout']['yaxis' + str(k + 1)]['title'] = y_labels[k]
                    control_fig['layout']['yaxis' + str(k + 1)]['title']['standoff'] = 0
                else:
                    control_fig['layout']['xaxis']['title'] = 't [s]'
                    control_fig['layout']['yaxis']['title'] = y_labels[k]
                k += 1
        control_fig.write_image(path + '/' + '_wind_sensitivity_control.png')
if __name__ == "__main__":
    path = './ResearchTests/MPCTestResults/BasicTest/2023:08:18:14:56:32mpc_basic_test.csv'
    mass_sensitivity_tests_path = './ResearchTests/MPCTestResults/MassSensitivity/'
    wind_tests_path = './ResearchTests/MPCTestResults/WindTests/'
    data_plotter = DataPlotter(path)
    #data_plotter._plotting_states()
    #data_plotter._plotting_controls()
    #data_plotter.plot_mass_sensitivity_tests(mass_sensitivity_tests_path)
    data_plotter.plot_wind_tests(wind_tests_path)
