from Factories.GaussianProcessFactory.kernels import RBF_Kernel
from Factories.GaussianProcessFactory.gaussian_process import GaussianProcess, EfficientGaussianProcess

import numpy as np
import matplotlib.pyplot as plt
plt.style.use('../../Factories/PlottingFactory/plotstyle.mplstyle')

def plot_round(gp, domain, true_values, img_path, i):
    plt.cla()
    plt.rcParams['figure.figsize'] = (16, 2)
    plt.rcParams['figure.titlesize'] = 36
    plt.rcParams['legend.fontsize'] = 28
    plt.rcParams['xtick.labelsize'] = 36
    plt.rcParams['ytick.labelsize'] = 36
    plt.plot(domain, true_values, color='red', label='Funkcja kary', linestyle='--')
    plt.errorbar(gp.X.flatten(), gp.mean, gp.std, linestyle=None, capsize=10, label='Model próbkowanej funkcji', color='grey')
    plt.scatter(gp.memory['obs_x'], gp.memory['obs_y'], label='Obserwacje')
    plt.title('Runda {}'.format(i))
    plt.xlabel(r"$\hat{p}$")
    plt.ylabel(r"$e_{pred}(\hat{p})$")
    plt.legend(loc='upper left')
    plt.xticks(gp.X.flatten())
    ax.tick_params(axis='x', colors='red', width=5, length=10)
    n = 10
    [l.set_visible(False) for (i,l) in enumerate(ax.xaxis.get_ticklabels()) if i % n != 0]
    for label in ax.xaxis.get_ticklabels():
        label.set_color('#2c2d2e')
    plt.tight_layout()
    plt.savefig(img_path + "gp_plot{}.jpg".format(i))


def plot_sampled_function(gp, img_path, i):
    plt.cla()
    plt.rcParams['figure.figsize'] = (16, 2)
    plt.rcParams['figure.titlesize'] = 36
    plt.rcParams['legend.fontsize'] = 28
    plt.rcParams['xtick.labelsize'] = 36
    plt.rcParams['ytick.labelsize'] = 36
    plt.plot(gp.sampled_function['x'], gp.sampled_function['y'], color='green', label='Realizacja GP', linestyle='--')
    plt.scatter(gp.sampled_function['best_action'], gp.sampled_function['predicted_reward'], color='red', label='Minimum')
    plt.errorbar(gp.X.flatten(), gp.mean, gp.std, linestyle=None, capsize=10, label='Model próbkowanej funkcji',
                 color='grey')
    plt.title('Przykład próbkowania Thompsona {}'.format(i))
    plt.xlabel(r"$\hat{p}$")
    plt.ylabel(r"$e_{pred}(\hat{p})$")
    plt.legend(loc='upper left')
    plt.xticks(gp.X.flatten())
    ax.tick_params(axis='x', colors='red', width=5, length=10)
    n = 10
    [l.set_visible(False) for (i, l) in enumerate(ax.xaxis.get_ticklabels()) if i % n != 0]
    for label in ax.xaxis.get_ticklabels():
        label.set_color('#2c2d2e')
    plt.tight_layout()
    plt.savefig(img_path + "ts_plot{}.jpg".format(i))



if __name__ == "__main__":
    img_path = '../../images/ilustracje4.6/'
    samples_num = 100
    observations = 10
    domain = (1, 2*3)
    step = 0.1
    X0 = np.arange(domain[0], domain[1] + step, step=step).reshape(-1, 1)

    noise_strength = 0.5
    noise = lambda: np.random.random() * noise_strength
    function = lambda x: np.cos(2*x) * x
    sample_function = lambda x: function(x) + noise()

    y_true = function(X0)

    ax = plt.gca()

    ax.plot(X0, y_true)
    plt.title(r"Kształt przykładowej funkcji $e_{pred}(\hat{p})$ ")
    plt.xlabel(r"$\hat{p}$")
    plt.ylabel(r"$e_{pred}(\hat{p})$")
    plt.savefig(img_path + 'original_function.png')

    plt.xticks(X0)
    ax.tick_params(axis='x', colors='red', width=5, length=10)
    n = 10
    [l.set_visible(False) for (i,l) in enumerate(ax.xaxis.get_ticklabels()) if i % n != 0]
    for label in ax.xaxis.get_ticklabels():
        label.set_color('#2c2d2e')
    plt.savefig(img_path + 'original_function_with_search_space.png')


    # observations
    x1 = np.random.choice(X0.flatten()).reshape((1, 1))
    y1 = np.sin(x1).flatten()

    rbf_kernel = RBF_Kernel(length=0.7, uncertainty=15)
    gp = EfficientGaussianProcess(X0, rbf_kernel, noise_std=0.3)
    plot_round(gp, X0, y_true, img_path, 0)

    # first_round
    action = np.random.choice(X0.flatten()).reshape((1, 1))
    y = sample_function(action.flatten()).flatten()
    gp(np.array([action]).reshape(-1, 1), y)
    plot_round(gp, X0, y_true, img_path, 1)
    for i in range(2, observations + 2):
        best = gp.Thompson_sampling(mode='min')
        plot_sampled_function(gp, img_path, i)
        action = best['best_action']
        y = sample_function(action).flatten()
        gp(np.array([action]).reshape(-1, 1), y)
        plot_round(gp, X0, y_true, img_path, i)
