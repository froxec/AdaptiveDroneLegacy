import plotly.graph_objects as go
from plotly.subplots import make_subplots
import pandas as pd
import plotly
import datetime
import matplotlib.pyplot as plt
class DataPlotter():
    def __init__(self, path):
        self.path = path
        self.df = pd.read_csv(path)
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
        plt.style.use('../../Factories/PlottingFactory/plotstyle.mplstyle')
        fig, ax = plt.subplots(nrows=3, ncols=1, sharex=True, dpi=100)
        t = pd.to_datetime(self.df['time'])
        t = t - t[0]
        t = t.dt.total_seconds()
        data = self.df[column_names]
        for i, col in enumerate(data, 1):
            ax[i].plot(t, col)
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
        legend_names = [r'$$\Delta m = -0.5 kg$$', r'$$\Delta m = -0.2 kg$$', r'$$\Delta m = 0.0 kg$$', r'$$\Delta m = 0.2 kg$$', r'$$\Delta m = 0.5 kg$$']
        self.plot_tests_state(tests, legend_names, path, 'mass_sensitivity')
        self.plot_tests_control(tests, legend_names, path, 'mass_sensitivity_control')

    def plot_wind_tests(self, path):
        wind1 = pd.read_csv(path + "wind_100_0.csv")
        wind2 = pd.read_csv(path + "wind_100_2.csv")
        wind3 = pd.read_csv(path + "wind_100_5.csv")
        tests = [wind1, wind2, wind3]
        legend_names = [r'$$||w|| = 0 [N]$$', r'$$||w|| = 2 [N]$$', r'$$||w|| = 5 [N]$$']
        self.plot_tests_state(tests, legend_names, path, 'wind_sensitivity')
        self.plot_tests_control(tests, legend_names, path, 'wind_sensitivity_control')
    def plot_adaptive_mass_sensitivity_tests(self, path):
        test1 = pd.read_csv(path + "adaptive_mass-05.csv")
        test2 = pd.read_csv(path + "adaptive_mass-02.csv")
        test3 = pd.read_csv(path + "adaptive_mass-00.csv")
        test4 = pd.read_csv(path + "adaptive_mass+02.csv")
        test5 = pd.read_csv(path + "adaptive_mass+05.csv")
        legend_names = [r'$$\Delta m = -0.5 kg$$', r'$$\Delta m = -0.2 kg$$', r'$$\Delta m = 0.0 kg$$', r'$$\Delta m = 0.2 kg$$', r'$$\Delta m = 0.5 kg$$']
        tests = [test1, test2, test3, test4, test5]
        self.plot_tests_state(tests, legend_names, path, 'adaptive_mass_sensitivity')
        self.plot_tests_control(tests, legend_names, path, 'adaptive_mass_sensitivity_control')
        self.plot_tests_sigma(tests, legend_names, path, 'adaptive_mass_sensitivity_sigma')
        self.plot_tests_u_adapt(tests, legend_names, path, 'adaptive_mass_sensitivity_u_adapt')
    def plot_adaptive_wind_tests(self, path):
        wind1 = pd.read_csv(path + "adaptive_wind_100_00.csv")
        wind2 = pd.read_csv(path + "adaptive_wind_100_02.csv")
        wind3 = pd.read_csv(path + "adaptive_wind_100_05.csv")
        tests = [wind1, wind2, wind3]
        legend_names = [r'$$||w|| = 0 [N]$$', r'$$||w|| = 2 [N]$$', r'$$||w|| = 5 [N]$$']
        self.plot_tests_state(tests, legend_names, path, 'adaptive_wind_sensitivity')
        self.plot_tests_control(tests, legend_names, path, 'adaptive_wind_sensitivity_control')
        self.plot_tests_sigma(tests, legend_names, path, 'adaptive_wind_sensitivity_sigma')
        self.plot_tests_u_adapt(tests, legend_names, path, 'adaptive_wind_sensitivity_u_adapt')
    def plot_tests_state(self, tests, legend_names, path, filename):
        plt.style.use('./Factories/PlottingFactory/plotstyle.mplstyle')
        fig, ax = plt.subplots(nrows=4, ncols=3, sharex=True, dpi=100)
        plot_indices = [(0, 0), (0, 1), (0, 2),
                        (1, 0), (1, 1), (1, 2),
                        (2, 0), (2, 1), (2, 2),
                        (3, 0), (3, 1), (3, 2)]
        state_columns = ['x', 'y', 'z', 'Vx', 'Vy', 'Vz',
                         'phi', 'theta', 'psi', 'omega_x', 'omega_y', 'omega_z']
        y_labels = [r'$$x [m]$$', r'$$y [m]$$', r'$$z [m]$$', r'$$V_x [m/s]$$', r'$$V_y [m/s]$$', r'$$V_z [m/s]$$',
                    r'$$\phi [rad]$$', r'$$\theta [rad]$$', r'$$\psi [rad]$$', r'$$\omega_{\phi} [rad/s]$$', r'$$\omega_{\theta} [rad/s]$$', r'$$\omega_{\psi} [rad/s]$$']
        t = tests[0]['time']
        for l, test in enumerate(tests):
            data = test
            k = 0
            for m, (col, (i, j)) in enumerate(zip(state_columns, plot_indices)):
                ax[i][j].plot(t, data[col], label=legend_names[l])
                ax[i][j].set_ylabel(y_labels[m])
                ax[i][j].legend()
        plt.savefig(path + '/' + filename + '.png')
    def plot_tests_control(self, tests, legend_names, path, filename):
        plt.style.use('./Factories/PlottingFactory/plotstyle.mplstyle')
        fig, ax = plt.subplots(nrows=3, ncols=1, sharex=True, dpi=100)
        control_columns = ['F', 'phi_ref', 'theta_ref']
        y_labels = [r'$$T_{zad} [N]$$', r'$$\phi_{zad} [rad]$$', r'$$\psi_{zad} [rad]$$']
        plot_indices = [(0, 0), (1, 0), (2, 0)]
        t = tests[0]['time']
        for l, test in enumerate(tests):
            data = test
            k = 0
            for m, (col, (i, j)) in enumerate(zip(control_columns, plot_indices)):
                ax[i].plot(t, data[col], label=legend_names[l])
                ax[i].set_ylabel(y_labels[m])
                ax[i].legend()
        plt.savefig(path + '/' + filename + '.png')

    def plot_tests_sigma(self, tests, legend_names, path, filename):
        plt.style.use('./Factories/PlottingFactory/plotstyle.mplstyle')
        fig, ax = plt.subplots(nrows=3, ncols=1, sharex=True, dpi=100)
        control_columns = ['sigma_x', 'sigma_y', 'sigma_z']
        y_labels = [r'$$\hat{\sigma}_0 [N]$$',
                    r'$$\hat{\sigma}_1 [rad]$$',
                    r'$$\hat{\sigma}_2 [rad]$$']
        plot_indices = [(0, 0), (1, 0), (2, 0)]
        t = tests[0]['time']
        for l, test in enumerate(tests):
            data = test
            k = 0
            for m, (col, (i, j)) in enumerate(zip(control_columns, plot_indices)):
                ax[i].plot(t, data[col], label=legend_names[l])
                ax[i].set_ylabel(y_labels[m])
                ax[i].legend()
        plt.savefig(path + '/' + filename + '.png')

    def plot_tests_u_adapt(self, tests, legend_names, path, filename):
        plt.style.use('./Factories/PlottingFactory/plotstyle.mplstyle')
        fig, ax = plt.subplots(nrows=3, ncols=1, sharex=True, dpi=100)
        control_columns = ['u_l1_x', 'u_l1_y', 'u_l1_z']
        y_labels = [r"$$ u_{adapt, 0} [N]$$",
                    r'$$ u_{adapt, 1} [rad]$$',
                    r'$$ u_{adapt, 2} [rad]$$']
        plot_indices = [(0, 0), (1, 0), (2, 0)]
        t = tests[0]['time']
        for l, test in enumerate(tests):
            data = test
            k = 0
            for m, (col, (i, j)) in enumerate(zip(control_columns, plot_indices)):
                ax[i].plot(t, data[col], label=legend_names[l])
                ax[i].set_ylabel(y_labels[m])
                ax[i].legend()
        plt.savefig(path + '/' + filename + '.png')

if __name__ == "__main__":
    path = './ResearchTests/MPCTestResults/BasicTest/2023:08:18:14:56:32mpc_basic_test.csv'
    mass_sensitivity_tests_path = './ResearchTests/MPCTestResults/MassSensitivity/'
    wind_tests_path = './ResearchTests/MPCTestResults/WindTests/'
    adaptive_mass_sensitivity = './ResearchTests/MPCTestResults/AdaptiveMassSensitivity/'
    adaptive_wind_sensitivity = './ResearchTests/MPCTestResults/AdaptiveWindSensitivity/'
    data_plotter = DataPlotter(path)
    #data_plotter._plotting_states()
    #data_plotter._plotting_controls()
    data_plotter.plot_mass_sensitivity_tests(mass_sensitivity_tests_path)
    data_plotter.plot_wind_tests(wind_tests_path)
    data_plotter.plot_adaptive_mass_sensitivity_tests(adaptive_mass_sensitivity)
    data_plotter.plot_adaptive_wind_tests(adaptive_wind_sensitivity)
