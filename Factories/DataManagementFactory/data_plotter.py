import plotly.graph_objects as go
from plotly.subplots import make_subplots
import pandas as pd
import matplotlib.pyplot as plt
import datetime
import matplotlib.animation as animation
import io
from PIL import Image
import numpy as np
from matplotlib.lines import Line2D
from Factories.ToolsFactory.GeneralTools import euclidean_distance

def plotly_fig2array(fig):
    #convert Plotly fig to  an array
    fig_bytes = fig.to_image(format="png")
    buf = io.BytesIO(fig_bytes)
    img = Image.open(buf)
    return np.asarray(img)

class DataPlotter():
    def __init__(self, paths, save_path, timeshifts, legend=None, use_plt = True):
        self.paths = paths
        if isinstance(paths, list):
            self.df = [pd.read_csv(path) for path in paths]
        else:
            raise ValueError("paths should be a list")
        self.save_path = save_path
        self.font_dict = dict(family='Arial',
                         size=26,
                         color='black'
                         )
        self.layout = {'font': self.font_dict,  # font formatting
                          'plot_bgcolor':'white',  # background color
                          'width' :850,  # figure width
                          'height':700,  # figure height
                          'margin': dict(r=20, t=50, b=10),  # remove white space
                          'showlegend': False
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
        self.use_plt = use_plt
        if legend is not None:
            self.legend=legend
        else:
            self.legend=['none']*len(self.paths)
        self.timeshifts = timeshifts
    def plot_velocity(self):
        column_names = ['VELOCITY:X', 'VELOCITY:Y', 'VELOCITY:Z']
        ylabels = [r'$V_x [m/s]$', r'$V_y [m/s]$', r'$V_z [m/s]$']
        xlabel = '$t [s]$'
        if self.use_plt:
            self._plotting_3_rows_matplotlib(column_names, 'Przebiegi czasowe predkości', xlabel, ylabels)
        else:
            self._plotting_3rows(column_names, 'velocity')

    def plot_position_local(self, reference_points=None, reference_shift=None):
        column_names = ['POSITION_LOCAL:X', 'POSITION_LOCAL:Y', 'POSITION_LOCAL:Z']
        ylabels = [r'$x [m]$', r'$y [m]$', r'$z [m]$']
        xlabel = '$t [s]$'
        if self.use_plt:
            self._plotting_3_rows_matplotlib(column_names, 'Przebiegi czasowe położenia', xlabel, ylabels, reference_points, reference_shift)
        else:
            self._plotting_3rows(column_names, 'position_local')

    def plot_attitude(self):
        column_names = ['ATTITUDE:X', 'ATTITUDE:Y', 'ATTITUDE:Z']
        ylabels = [r'$\phi [rad]$', r'$\theta [rad]$', r'$\psi [rad]$']
        xlabel = '$t [s]$'
        if self.use_plt:
            self._plotting_3_rows_matplotlib(column_names, 'Przebiegi czasowe orientacji', xlabel, ylabels)
        else:
            self._plotting_3rows(column_names, 'attitude')

    def plot_output_control(self):
        column_names = ['U_OUTPUT:X', 'U_OUTPUT:Y', 'U_OUTPUT:Z']
        ylabels = [r'$T_{out} [N]$', r'$\phi_{out} [rad]$', r'$\theta_{out} [rad]$']
        xlabel = '$t [s]$'
        if self.use_plt:
            self._plotting_3_rows_matplotlib(column_names, 'Przebiegi czasowe sterowań wyjściowych', xlabel, ylabels)
        else:
            self._plotting_3rows(column_names, 'u_output')

    def plot_u_l1(self):
        column_names = ['u_l1:X', 'u_l1:Y', 'u_l1:Z']
        ylabels = [r'$u_{l1,0} [N]$', r'$u_{l1,1}[rad]$', r'$u_{l1,2} [rad]$']
        xlabel = '$t [s]$'
        if self.use_plt:
            self._plotting_3_rows_matplotlib(column_names, 'Przebiegi czasowe sterowań adaptacyjnych', xlabel, ylabels)
        else:
            self._plotting_3rows(column_names, 'u_l1')

    def plot_sigma(self):
        column_names = ['sigma_hat:X', 'sigma_hat:Y', 'sigma_hat:Z']
        ylabels = [r'$\hat{\sigma}_{0} [N]$', r'$\hat{\sigma}_{1}[rad]$', r'$\hat{\sigma}_{2} [rad]$']
        xlabel = '$t [s]$'
        if self.use_plt:
            self._plotting_3_rows_matplotlib(column_names, 'Przebiegi czasowe estymowanej niepewności', xlabel, ylabels)
        else:
            self._plotting_3rows(column_names, 'sigma_hat')

    def plot_u_ref(self):
        column_names = ['U_REF:X', 'U_REF:Y', 'U_REF:Z']
        ylabels = [r'$T_{zad} [N]$', r'$\phi_{zad} [rad]$', r'$\theta_{zad} [rad]$']
        xlabel = '$t [s]$'
        if self.use_plt:
            self._plotting_3_rows_matplotlib(column_names, 'Przebiegi czasowe sterowań bazowych', xlabel, ylabels)
        else:
            self._plotting_3rows(column_names, 'u_reference')

    def animate_position_local(self):
        column_names = ['POSITION_LOCAL:X', 'POSITION_LOCAL:Y', 'POSITION_LOCAL:Z']
        self._animate_3_rows(column_names, title='position_local')

    def _plotting_3rows(self, column_names, title):
        t = pd.to_datetime(self.df['TIME'])
        t = t - t[0]
        t = t.dt.total_seconds()
        data = self.df[column_names]
        fig = make_subplots(rows=3, cols=1)
        fig.update_layout(**self.layout)
        fig.update_xaxes(**self.axeslayout)
        fig.update_yaxes(**self.axeslayout)
        fig.update_layout(title_text=title)
        for i, col in enumerate(data, 1):
            fig.add_trace(go.Scatter(x=t, y=data[col]), row=i, col=1)
        fig.show()

    def _plotting_3_rows_matplotlib(self, column_names, title, xlabel, ylabels, reference_points=None, reference_shift=None, linestyles=['solid', 'dashdot', 'dotted'], markers=['o', '*', 'P']):
        plt.style.use('../../Factories/PlottingFactory/plotstyle.mplstyle')
        fig, ax = plt.subplots(nrows=3, ncols=1, sharex=True, dpi=100)
        for k, df in enumerate(self.df):
            t = pd.to_datetime(df['TIME'])
            t = t[timeshifts[k]:]
            t = t - t[timeshifts[k]]
            t = t.dt.total_seconds()
            data = df[column_names][timeshifts[k]:]
            for i, col in enumerate(data):
                ax[i].plot(t, data[col], label=self.legend[k], linestyle=linestyles[k])
                ax[i].set_ylabel(ylabels[i])
                ax[i].legend()
        if reference_points is not None:
            reference_points = np.array(reference_points)
            ref = np.empty_like(data)
            for i, point in enumerate(np.array(data)):
                closest_ref_id = np.argmin(np.abs(reference_points - point), axis=0)
                indices = zip(closest_ref_id, range(3))
                closest_ref = [reference_points[id] for id in indices]
                ref[i] = closest_ref
            ref = np.roll(ref, -reference_shift, axis=0)
            for i, line in enumerate(ref.T):
                ax[i].plot(t, line, label='trajektoria referencyjna', color='red', linestyle='dashed')
                ax[i].legend()
        ax[-1].set_xlabel(xlabel)
        fig.suptitle(title)
        plt.savefig(self.save_path+title+'.png')
    # def _animate_3_rows(self, column_names, title):
    #     duration=20
    #     animation_steps = np.arange(0, 20, step=0.02)
    #     animation_mapping = {}
    #     df = self.df.iloc[::10, :]
    #     df = df.reset_index()
    #     for i in range(animation_steps.shape[0]):
    #         animation_mapping[animation_steps[i]] = i
    #     t = pd.to_datetime(df['TIME'])
    #     t = t - t[0]
    #     t = t.dt.total_seconds()
    #     n = t.size
    #     data = df[column_names]
    #     fig = make_subplots(rows=3, cols=1)
    #     fig.update_layout(**self.layout)
    #     fig.update_xaxes(**self.axeslayout)
    #     fig.update_yaxes(**self.axeslayout)
    #     fig.update_layout(title_text=title)
    #     traces = [go.Scatter(x=t, y=data[col]) for col in data]
    #     for i, trace in enumerate(traces, 1):
    #         fig.add_trace(trace=trace, row=i, col=1)
    #     def mpy_make_frame(time_stamp):
    #         fig.data = fig.data[:3]
    #         time_index = animation_mapping[time_stamp]
    #         for i, trace in enumerate(traces, 1):
    #             fig.add_scatter(x=[t[time_index]],
    #                                        y=[data[column_names[i-1]][time_index]],
    #                                        mode="markers",
    #                                        marker=dict(color="red", size=10), row=i, col=1)
    #         return plotly_fig2array(fig)
    #     animation = mpy.VideoClip(mpy_make_frame, duration=duration)
    #     animation.write_gif(title + ".gif", fps=50)

    def _animate_3_rows(self, column_names, title):
        # get data
        df = self.df.iloc[::10, :]
        df = df.reset_index()
        t = pd.to_datetime(df['TIME'])
        t = t - t[0]
        t = t.dt.total_seconds()
        intervals = np.array(t.tolist())[1:] - np.array(t.to_list())[:-1]
        mean_interval = intervals.mean()*1000
        global scats
        scats = []
        data = df[column_names]
        indices_array = np.arange(t.size)
        # create figure
        fig, ax = plt.subplots(nrows=3, ncols=1, sharex=True)
        for i, axis in enumerate(ax):
            axis.plot(t, data[column_names[i]], linewidth=2)

        def update(time_step):
            global scats
            for scat in scats:
                scat.remove()
            scats = []
            mask = indices_array==time_step
            for i, axis in enumerate(ax):
                scats.append(axis.scatter(t, data[column_names[i]], marker="o", c='red', zorder=2, alpha=mask))

        ani = animation.FuncAnimation(fig=fig, func=update, frames=int(indices_array.shape[0]), interval=mean_interval)
        writervideo = animation.FFMpegWriter(fps=1000/mean_interval)
        ani.save(title + '.mp4', writer=writervideo)
        plt.close()




if __name__ == "__main__":
    import os
    TEST_NAME = 'TEST WITH ADDITIONAL DATA RPI.csv'
    base_path = '/home/pete/PycharmProjects/AdaptiveDrone/logs/field_07_10/'
    save_path = '/home/pete/PycharmProjects/AdaptiveDrone/images/test_plots/'
    path = [base_path+'testbaro1.csv']
    # path = ['/home/pete/PycharmProjects/AdaptiveDrone/logs/sim_official_new/load0_1762.csv',
    #         '/home/pete/PycharmProjects/AdaptiveDrone/logs/sim_official_new/load300_2066.csv',
    #         '/home/pete/PycharmProjects/AdaptiveDrone/logs/sim_official_new/load500_2268.csv']
    timeshifts = [0]
    # reference_points =  [[-10, -10, 6],
    #                     [-10, 10, 6],
    #                     [10, 10, 6],
    #                     [-10, 10, 6],
    #                     [-10, -10, 6]]
    reference_points= None
    reference_shift = 220
    legend = [r'm_{load} = 0.0 kg', r'm_{load} = 0.3 kg', r'm_{load} = 0.5 kg']
    # cwd = os.getcwd()
    # dir = os.listdir()
    # candidates = []
    # for i in range(len(dir)):
    #     print(dir[i])
    #     name = dir[i].split("_")[1]
    #     if name == TEST_NAME:
    #         candidates.append(dir[i])
    # print(candidates)
    # path = candidates[0]
    #print(os.listdir(os.getcwd()))
    data_plotter = DataPlotter(path, save_path, timeshifts, legend)
    data_plotter.plot_position_local(reference_points, reference_shift)
    data_plotter.plot_velocity()
    data_plotter.plot_output_control()
    data_plotter.plot_u_l1()
    data_plotter.plot_u_ref()
    data_plotter.plot_sigma()
    data_plotter.plot_attitude()
    plt.show(block=True)
    # data_plotter.animate_position_local()