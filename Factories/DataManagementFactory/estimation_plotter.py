import os

import pandas as pd
import matplotlib.pyplot as plt
import datetime
import matplotlib.animation as animation
import numpy as np
from scipy.ndimage import gaussian_filter

class EstimationPlotter:
    def __init__(self, dirs, save_path, stats_save_path, freq, buffer_lengths, convergence_n, realvalue, params, titles, max_t=50):
        if isinstance(dirs, list):
            self.dfs = []
            for dir in dirs:
                files = os.listdir(dir)
                paths = [dir + file for file in files]
                self.dfs.append([pd.read_csv(path) for path in paths])
        else:
            raise ValueError("paths should be a list")
        self.freq = freq
        self.dt = 1 / freq
        self.buffer_lengths = buffer_lengths
        self.sample_dt = list(self.dt * np.array(self.buffer_lengths))
        self.save_path = save_path
        self.stats_save_paths = stats_save_path
        self.realvalue = realvalue
        self.convergence_n = convergence_n
        self.params = params
        self.titles=titles
        self.max_t = max_t

    def plot_estimation_result(self, realvalue, estimatedvalue):
        column_names = ['action', 'penalty']
        self._plotting_2_rows_matplotlib(column_names, title='Przebiegi czasowe akcji oraz kary',
                                         xlabel='t [s]', ylabels=['Akcja', 'Kara'],
                                         realvalue=realvalue, estimated_value=estimatedvalue)

    def _plotting_2_rows_matplotlib(self, column_names, title, xlabel, ylabels, realvalue=None, estimated_value=None, linestyles=['solid', 'dashdot', 'dotted']):
        plt.style.use('../../Factories/PlottingFactory/plotstyle.mplstyle')
        fig, ax = plt.subplots(nrows=2, ncols=1, sharex=True, dpi=100)
        for k, df in enumerate(self.df):
            data = df[column_names]
            t = np.arange(0, len(data))*self.sample_dt
            for i, col in enumerate(data):
                ax[i].scatter(t, data[col])
                ax[i].set_ylabel(ylabels[i])
        if realvalue is not None:
            realvalues = np.ones_like(t)*realvalue
            ax[0].plot(t, realvalues, label='Wartość rzeczywista', linestyle=linestyles[0])
            ax[0].legend()
        if estimated_value is not None:
            estimated_values = np.ones_like(t)*estimated_value
            ax[0].plot(t, estimated_values, label='Estymata', linestyle=linestyles[2], color='r')
            ax[0].legend()


        ax[-1].set_xlabel(xlabel)
        fig.suptitle(title)
        plt.savefig(self.save_path+title+'.png')
        plt.show()

    def plot_tests_heatmap(self):
        plt.style.use('../../Factories/PlottingFactory/plotstyle.mplstyle')
        plt.tight_layout = True
        plt.rcParams['figure.figsize'] = (16, 3*len(self.dfs))
        heatmaps = []
        extents = []
        data = []
        t_max = 0
        #generate heatmaps
        for i, dfs in enumerate(self.dfs):
            x, y = self.collect_data_for_hm(dfs, 'action', test_num=i)
            t_max = max(t_max, np.max(x))
            data.append((x,y))
        if t_max > self.max_t:
            t_max = self.max_t
        x_bins = np.linspace(0, t_max, 100)
        for i, data_pair in enumerate(data):
            x, y = data_pair[0], data_pair[1]
            heatmap, xedges, yedges = np.histogram2d(x, y, bins=(x_bins, 50))
            heatmap = gaussian_filter(heatmap, sigma=2)
            heatmaps.append(heatmap)
            extents.append([xedges[0], xedges[-1], yedges[0], yedges[-1]])
        # draw heatmaps
        if len(heatmaps) > 1:
            fig, ax = plt.subplots(nrows=len(heatmaps), ncols=1, sharex=True, gridspec_kw={'hspace': 0.3})
            for i, hm in enumerate(heatmaps):
                ax[i].imshow(hm.T, extent=list(extents[i]), origin='lower', cmap='jet', aspect='auto')
                ax[i].set_ylabel(r'$\hat{m}$ [kg]')
                ax[i].axhline(y=self.realvalue, color='white', linestyle='--', linewidth=2, label=r'$\hat{m}$' + '= {} [kg]'.format(self.realvalue))
                ax[i].set_title(titles[i])
                #ax[i].colorbar()
        else:
            plt.imshow(heatmaps[0], extent=list(extents[0]), origin='lower', cmap='jet', aspect='auto')
            plt.colorbar()
        plt.xlabel(r't [s]')
        plt.savefig(self.save_path+'.png')
        plt.show()
    def collect_data_for_hm(self, dfs, column_names, test_num):
        data_for_hm = []
        for df in dfs:
            data = df[column_names]
            t = np.arange(0, len(data)) * self.sample_dt[test_num]
            for i in range(len(data)):
                data_for_hm.append((t[i], data[i]))
        data_for_hm = list(zip(*data_for_hm))
        return data_for_hm

    def calculate_statistics(self):
        for i, dfs in enumerate(self.dfs):
            column_names = ['action']
            conv_times = []
            estimated_values = []
            for df in dfs:
                data = df[column_names]
                t = np.arange(0, len(data)) * self.sample_dt[i]
                conv_times.append(t[-1])
                influencers = np.array(data[-self.convergence_n[i]:]).flatten()
                estimation = np.mean(influencers)
                estimated_values.append(estimation)


            # mean_convergence_time, max_convergence_time, min_convergence_time, conv_time_std
            mean_convergence_time = np.mean(conv_times)
            max_convergence_time = np.max(conv_times)
            min_convergence_time = np.min(conv_times)
            conv_time_std = np.std(conv_times)

            # mean_error, minimum_error, maximum_error, error_variance
            estimated_values = np.array(estimated_values).flatten()
            errors = np.abs(estimated_values - self.realvalue)
            mean_error = np.mean(errors)
            minimum_error = np.min(errors)
            maximum_error = np.max(errors)
            std_error = np.std(errors)

            stats = {'mean_convergence_time':[mean_convergence_time],
                     'min_convergence_time': [min_convergence_time],
                     'max_convergence_time': [max_convergence_time],
                     'conv_time_std': [conv_time_std],
                     'mean_error': [mean_error],
                     'minimum_error': [minimum_error],
                     'maximum_error': [maximum_error],
                     'std_error': [std_error]
                     }

            stats = pd.DataFrame(stats)
            with pd.option_context('display.max_rows', None, 'display.max_columns',None):
                print(self.params[i] + "STATS:")
                print(stats)
            if self.stats_save_paths == None:
                continue
            if os.path.isfile(self.stats_save_paths):
                header = False
            else:
                header = True
            stats['params'] = self.params[i]
            stats.to_csv(self.stats_save_paths, mode='a', index=False, header=header)









if __name__ == "__main__":
    #path = ['/home/pete/PycharmProjects/AdaptiveDrone/logs/sim_official_new/load0_estimation.csv']
    save_path = '/home/pete/PycharmProjects/AdaptiveDrone/images/test_plots/'
    test_name = 'lque_changes'
    stats_save_path = '/home/pete/PycharmProjects/AdaptiveDrone/logs/estimation_tests/' + test_name + '.csv'
    estim_base_dir = '/home/pete/PycharmProjects/AdaptiveDrone/logs/estimation_tests/' + test_name +'/'
    folders = ['ns0.08len0.7con_smpl15eps0.15atm_smpls3/',
               'ns0.08len0.7con_smpl15eps0.15atm_smpls10/',
               'ns0.08len0.7con_smpl15eps0.15atm_smpls35/',
               'ns0.08len0.7con_smpl15eps0.15atm_smpls50/']
    titles = [r'$l_{que} = 3$',
              r'$l_{que} = 10$',
              r'$l_{que} = 35$',
              r'$l_{que} = 50$']
    dirs = [estim_base_dir + folder_name for folder_name in folders]
    plotter = EstimationPlotter(dirs=dirs, save_path=save_path, stats_save_path=stats_save_path,
                                freq=100, buffer_lengths=[3, 10, 35, 50], convergence_n=[15, 15, 15, 15], realvalue=2.25, params=folders, titles=titles, max_t=50)
    plotter.calculate_statistics()
    plotter.plot_tests_heatmap()
    # plotter.plot_estimation_result(realvalue=1.75, estimatedvalue=1.766)

