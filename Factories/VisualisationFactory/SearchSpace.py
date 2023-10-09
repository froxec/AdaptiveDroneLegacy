import matplotlib.pyplot as plt
import numpy as np
from Factories.ToolsFactory.GeneralTools import euclidean_distance
class SearchSpace:
    def __init__(self, N, q, scale=1, shift=0):
        self.N = N
        self.q = q
        self.scale = scale
        self.shift = shift

    def create_space(self):
        coords = []
        for dim in range(self.q):
            coords.append(np.random.random(self.N)*self.scale + self.shift)
        points = list(zip(*coords))
        return points

    def plot_3dspace(self, space):
        plt.style.use('../../Factories/PlottingFactory/plotstyle.mplstyle')
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        x, y, z = list(zip(*space))
        ax.scatter(x, y, z)
        fig.suptitle(r'N = {}'.format(self.N))
        plt.savefig('search_spaceN{}'.format(self.N) + '.png', bbox_inches='tight')

    def exploration_exploitation(self, space):
        plt.style.use('../../Factories/PlottingFactory/plotstyle.mplstyle')
        chosen = np.random.choice(range(len(space)), replace=False, size=20)
        chosen_one = np.random.choice(chosen)
        epsilon = 3
        space = np.array(space)
        in_region = (euclidean_distance(space[chosen_one], space, axis=1) < epsilon)
        in_region_indices = [i for i, x in enumerate(in_region) if x]
        in_region_chosen = np.random.choice(in_region_indices, size=int(len(in_region_indices)/4), replace=False)
        color_map = np.array(['#196E85']*len(space), dtype='object')
        color_map[chosen] = 'red'
        color_map[in_region_chosen] = '#FFD700'
        markers_map = np.array(['.']*len(space), dtype='object')
        markers_map[chosen] = 'X'
        markers_map[in_region_chosen] = '*'
        markersize_map = np.array([10] * len(space), dtype='object')
        markersize_map[chosen] = 200
        markersize_map[in_region_chosen] = 200
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        x, y, z = list(zip(*space))
        for i in range(len(x)):
            ax.scatter(x[i], y[i], z[i], [markersize_map[i]], c=list(color_map)[i], marker=markers_map[i])
        fig.suptitle(r'N = {}'.format(self.N))
        plt.savefig('evse{}'.format(self.N) + '.png', bbox_inches='tight')


if __name__ == "__main__":
    search_space = SearchSpace(1000, 3, 10, 10)
    space = search_space.create_space()
    search_space.plot_3dspace(space)
    search_space.exploration_exploitation(space)
