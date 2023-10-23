import os

import plotly.graph_objects as go
from plotly.subplots import make_subplots
import pandas as pd
import plotly
import datetime
import matplotlib.pyplot as plt
class DataPlotter():
    def __init__(self, path=None):
        self.path = path
        if path is not None:
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

    def plot_basic_mpc_test(self, path):
        df = pd.read_csv(path + 'mpc_basic_tests.csv')
        tests = [df]
        colors = ['#196E85']
        self.plot_tests_state(tests, legend_names=None, path=path, filename='mpc_basic_tests', colors=colors)
        self.plot_tests_control(tests, legend_names=None, path=path, filename='mpc_basic_tests_control', colors=colors)

    def plot_mass_sensitivity_tests(self, path):
        df_min05 = pd.read_csv(path + "mpc-05kg.csv")
        df_min02 = pd.read_csv(path + "mpc-02kg.csv")
        df_0 = pd.read_csv(path + "mpc+00kg.csv")
        df_plus02 = pd.read_csv(path + "mpc+02kg.csv")
        df_plus05 = pd.read_csv(path + "mpc+05kg.csv")
        tests = [df_min05, df_min02, df_0, df_plus02, df_plus05]
        colors = ['#808080', '#e89b26', '#326024', '#ff0000', '#196E85']
        legend_names = [r'$$\Delta m = -0.5 kg$$', r'$$\Delta m = -0.2 kg$$', r'$$\Delta m = 0.0 kg$$', r'$$\Delta m = 0.2 kg$$', r'$$\Delta m = 0.5 kg$$']
        self.plot_tests_state(tests, legend_names, path, 'mass_sensitivity', colors=colors)
        self.plot_tests_control(tests, legend_names, path, 'mass_sensitivity_control', colors=colors)

    def plot_wind_tests(self, path):
        wind1 = pd.read_csv(path + "wind_100_0.csv")
        wind2 = pd.read_csv(path + "wind_100_2.csv")
        wind3 = pd.read_csv(path + "wind_100_5.csv")
        tests = [wind1, wind2, wind3]
        colors = ['#808080', '#e89b26', '#326024', '#ff0000', '#196E85']
        legend_names = [r'$$||w|| = 0 [N]$$', r'$$||w|| = 2 [N]$$', r'$$||w|| = 5 [N]$$']
        self.plot_tests_state(tests, legend_names, path, 'wind_sensitivity', colors=colors)
        self.plot_tests_control(tests, legend_names, path, 'wind_sensitivity_control', colors=colors)
    def plot_adaptive_mass_sensitivity_tests(self, path):
        test1 = pd.read_csv(path + "adaptive-05kg.csv")
        test2 = pd.read_csv(path + "adaptive-02kg.csv")
        test3 = pd.read_csv(path + "adaptive+00kg.csv")
        test4 = pd.read_csv(path + "adaptive+02kg.csv")
        test5 = pd.read_csv(path + "adaptive+05kg.csv")
        legend_names = [r'$$\Delta m = -0.5 kg$$', r'$$\Delta m = -0.2 kg$$', r'$$\Delta m = 0.0 kg$$', r'$$\Delta m = 0.2 kg$$', r'$$\Delta m = 0.5 kg$$']
        tests = [test1, test2, test3, test4, test5]
        colors = ['#808080', '#e89b26', '#326024', '#ff0000', '#196E85']
        self.plot_tests_state(tests, legend_names, path, 'adaptive_mass_sensitivity', colors=colors)
        self.plot_tests_control(tests, legend_names, path, 'adaptive_mass_sensitivity_control', colors=colors)
        self.plot_tests_sigma(tests, legend_names, path, 'adaptive_mass_sensitivity_sigma', colors=colors)
        self.plot_tests_u_adapt(tests, legend_names, path, 'adaptive_mass_sensitivity_u_adapt', colors=colors)
    def plot_adaptive_wind_tests(self, path):
        wind1 = pd.read_csv(path + "adaptive_wind_100_00.csv")
        wind2 = pd.read_csv(path + "adaptive_wind_100_02.csv")
        wind3 = pd.read_csv(path + "adaptive_wind_100_05.csv")
        tests = [wind1, wind2, wind3]
        colors = ['#808080', '#e89b26', '#326024', '#ff0000', '#196E85']
        legend_names = [r'$$||w|| = 0 [N]$$', r'$$||w|| = 2 [N]$$', r'$$||w|| = 5 [N]$$']
        self.plot_tests_state(tests, legend_names, path, 'adaptive_wind_sensitivity', colors=colors)
        self.plot_tests_control(tests, legend_names, path, 'adaptive_wind_sensitivity_control', colors=colors)
        self.plot_tests_sigma(tests, legend_names, path, 'adaptive_wind_sensitivity_sigma', colors=colors)
        self.plot_tests_u_adapt(tests, legend_names, path, 'adaptive_wind_sensitivity_u_adapt', colors=colors)
    def plot_tests_state(self, tests, legend_names, path, filename, colors):
        plt.style.use('./Factories/PlottingFactory/plotstyle.mplstyle')
        plt.rcParams['axes.prop_cycle'] = plt.cycler(color=colors)
        nrows = 4
        fig, ax = plt.subplots(nrows=nrows, ncols=3, sharex=True, dpi=100)
        fix_range = [(3, 2), (2, 2)]
        plot_indices = [(0, 0), (0, 1), (0, 2),
                        (1, 0), (1, 1), (1, 2),
                        (2, 0), (2, 1), (2, 2),
                        (3, 0), (3, 1), (3, 2)]
        state_columns = ['x', 'y', 'z', 'Vx', 'Vy', 'Vz',
                         'phi', 'theta', 'psi', 'omega_x', 'omega_y', 'omega_z']
        y_labels = [r'$$x [m]$$', r'$$y [m]$$', r'$$z [m]$$', r'$$V_x [m/s]$$', r'$$V_y [m/s]$$', r'$$V_z [m/s]$$',
                    r'$$\phi [rad]$$', r'$$\theta [rad]$$', r'$$\psi [rad]$$', r'$$\omega_{\phi} [rad/s]$$', r'$$\omega_{\theta} [rad/s]$$', r'$$\omega_{\psi} [rad/s]$$']
        xlabel = 't [s]'
        t = tests[0]['time']
        for l, test in enumerate(tests):
            data = test
            k = 0
            for m, (col, (i, j)) in enumerate(zip(state_columns, plot_indices)):
                ax[i][j].plot(t, data[col])
                ax[i][j].set_ylabel(y_labels[m])
                if i == nrows - 1:
                    ax[i][j].set_xlabel(xlabel)
                if (i, j) in fix_range:
                    ax[i][j].set_ylim(-1, 1)
        plt.tight_layout()
        if legend_names is not None:
            plt.subplots_adjust(right=0.85)
            legend_ax = fig.add_axes([0.9, 0.6, 0.1, 0.05])
            legend_ax.legend(ax[0, 0].get_lines(), legend_names, loc='upper right')
            legend_ax.axis('off')
        plt.savefig(path + '/' + filename + '.png')
    def plot_tests_control(self, tests, legend_names, path, filename, colors):
        plt.style.use('./Factories/PlottingFactory/plotstyle.mplstyle')
        plt.rcParams['axes.prop_cycle'] = plt.cycler(color=colors)
        nrows = 3
        fig, ax = plt.subplots(nrows=nrows, ncols=1, sharex=True, dpi=100)
        control_columns = ['F', 'phi_ref', 'theta_ref']
        y_labels = [r'$$T_{zad} [N]$$', r'$$\phi_{zad} [rad]$$', r'$$\theta_{zad} [rad]$$']
        xlabel = 't [s]'
        plot_indices = [(0, 0), (1, 0), (2, 0)]
        t = tests[0]['time']
        for l, test in enumerate(tests):
            data = test
            k = 0
            for m, (col, (i, j)) in enumerate(zip(control_columns, plot_indices)):
                ax[i].plot(t, data[col])
                ax[i].set_ylabel(y_labels[m])
                if i == nrows-1:
                    ax[i].set_xlabel(xlabel)
        plt.tight_layout()
        if legend_names is not None:
            plt.subplots_adjust(right=0.85)
            legend_ax = fig.add_axes([0.9, 0.6, 0.1, 0.05])
            legend_ax.legend(ax[0].get_lines(), legend_names, loc='upper right')
            legend_ax.axis('off')
        plt.savefig(path + '/' + filename + '.png')

    def plot_tests_sigma(self, tests, legend_names, path, filename, colors):
        plt.style.use('./Factories/PlottingFactory/plotstyle.mplstyle')
        plt.rcParams['axes.prop_cycle'] = plt.cycler(color=colors)
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
                ax[i].plot(t, data[col])
                ax[i].set_ylabel(y_labels[m])
        plt.tight_layout()
        if legend_names is not None:
            plt.subplots_adjust(right=0.85)
            legend_ax = fig.add_axes([0.9, 0.6, 0.1, 0.05])
            legend_ax.legend(ax[0].get_lines(), legend_names, loc='upper right')
            legend_ax.axis('off')
        plt.savefig(path + '/' + filename + '.png')

    def plot_tests_u_adapt(self, tests, legend_names, path, filename, colors):
        plt.style.use('./Factories/PlottingFactory/plotstyle.mplstyle')
        plt.rcParams['axes.prop_cycle'] = plt.cycler(color=colors)
        fig, ax = plt.subplots(nrows=3, ncols=1, sharex=True, dpi=100)
        plt.tight_layout()
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
                ax[i].plot(t, data[col])
                ax[i].set_ylabel(y_labels[m])
        plt.tight_layout()
        if legend_names is not None:
            plt.subplots_adjust(right=0.85)
            legend_ax = fig.add_axes([0.9, 0.6, 0.1, 0.05])
            legend_ax.legend(ax[0].get_lines(), legend_names, loc='upper right')
            legend_ax.axis('off')
        plt.savefig(path + '/' + filename + '.png')

if __name__ == "__main__":
    basic_test_path = './ResearchTests/SimulationResultsNew/MPC_BASIC_TEST/'
    mass_sensitivity_tests_path = './ResearchTests/SimulationResultsNew/MPC_MASS_PERTURBATION_TESTS/'
    wind_tests_path = './ResearchTests/SimulationResultsNew/MPC_WIND_TESTS/'
    adaptive_mass_sensitivity = './ResearchTests/SimulationResultsNew/ADAPTIVE_MASS_PERTURBATION_TESTS/'
    adaptive_wind_sensitivity = './ResearchTests/SimulationResultsNew/ADAPTIVE_WIND_TESTS/'
    data_plotter = DataPlotter()
    #data_plotter.plot_basic_mpc_test(basic_test_path)
    #data_plotter.plot_mass_sensitivity_tests(mass_sensitivity_tests_path)
    #data_plotter.plot_wind_tests(wind_tests_path)
    #data_plotter.plot_adaptive_mass_sensitivity_tests(adaptive_mass_sensitivity)
    data_plotter.plot_adaptive_wind_tests(adaptive_wind_sensitivity)
