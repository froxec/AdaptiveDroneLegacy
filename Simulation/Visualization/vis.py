import copy
import time
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import numpy as np

# MATPLOTLIB MULTIPROCESSING
# https://matplotlib.org/3.1.0/gallery/misc/multiprocess_sgskip.html

class Visualizer:

    def __init__(self, do_blit=True, azimuth_init=45, elevation_init=45):
        # TODO make quadcopter read-only, immutable
        self.u_base = np.array([-1, 0, 0], dtype=np.float32)
        self.v_base = np.array([0, 1, 0], dtype=np.float32)
        self.w_base = np.array([0, 0, 1], dtype=np.float32)
        self.u = np.array([1, 0, 0], dtype=np.float32)
        self.v = np.array([0, 1, 0], dtype=np.float32)
        self.w = np.array([0, 0, 1], dtype=np.float32)
        self.azimuth = azimuth_init
        self.elevation = elevation_init
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.quiver = self.ax.quiver(0, 0, 0, self.u, self.v, self.w, color=['b', 'b', 'r'], animated='True')
        self.time_text = self.ax.text(0, 0, 0, 'time=%s' % 0.0)
        self.background = self.init_fig(do_blit)
        self.do_blit = do_blit
        self.font = {
            'family': 'serif',
            'color': 'green',
            'weight': 'normal',
            'size': 9
        }

    def init_fig(self, do_blit, x_lim=[-10, 10], y_lim=[-10, 10], z_lim=[-10, 10], x_label='x', y_label='y',
                 z_label='z'):
        self.ax.set_xlim(x_lim)
        self.ax.set_ylim(y_lim)
        self.ax.set_zlim(z_lim)
        self.ax.set_xlabel(x_label, fontsize=20)
        self.ax.set_ylabel(y_label, fontsize=20)
        self.ax.set_zlabel(z_label, fontsize=20)
        self.ax.view_init(elev=self.elevation, azim=self.azimuth)
        # self.azimuth_slider = Slider(
        #     ax=self.sliders,
        #     label="Azimuth",
        #     valmin=0,
        #     valmax=360,
        #     valinit=self.azimuth,
        #     orientation="vertical"
        # )
        # self.elevation_slider = Slider(
        #     ax=self.sliders,
        #     label="Elevation",
        #     valmin=0,
        #     valmax=360,
        #     valinit=self.elevation,
        #     orientation="vertical"
        # )
        plt.show(block=False)
        plt.ion()
        plt.pause(0.1)  # wait for a while so we have background to render on
        if do_blit:
            # cache the background
            background = self.fig.canvas.copy_from_bbox(self.ax.bbox)
            return background

    def __call__(self, position, angles, t, azimuth=45, elevation=45, stop=False):
        self.transform_base_vectors(angles)
        self.visualize_body_frame(position, t)
        #self.update_limits(position)
        if azimuth != self.azimuth or elevation != self.elevation:
            self.azimuth = azimuth
            self.elevation = elevation
            self.update_view()
        # self.azimuth_slider.on_changed(self.update_view)
        # self.elevation_slider.on_changed(self.update_view)

        if stop == True:
            NotImplementedError
            self.end_job()

    def update_view(self):
        self.ax.view_init(elev=self.elevation, azim=self.azimuth)

    def update_limits(self, position):
        left_lim = position - 10
        right_lim = position + 10
        x_lim = [left_lim[0], right_lim[0]]
        y_lim = [left_lim[1], right_lim[1]]
        z_lim = [left_lim[2], right_lim[2]]
        self.ax.set_xlim(x_lim)
        self.ax.set_ylim(y_lim)
        self.ax.set_zlim(z_lim)

    def transform_base_vectors(self, angles):
        cR = np.cos(angles[0])
        cP = np.cos(angles[1])
        cY = np.cos(angles[2])
        sR = np.sin(angles[0])
        sP = np.sin(angles[1])
        sY = np.sin(angles[2])
        R = np.array([[cP * cY, sY * cP, sP],
                      [sP * sR * cY - sY * cR, sP * sR * sY + cR * cY, -sR * cP],
                      [-sP * cR * cY - sR * sY, -sP * sY * cR + sR * cY, cP * cR]], dtype=np.float32)
        self.u = np.matmul(R, self.u_base)
        self.v = np.matmul(R, self.v_base)
        self.w = np.matmul(R, self.w_base)
        # TODO - check if original model transformation row/column interpretation is correct
        # TODO - check precision problem -> np.linalg.norm(self.w) is not 1
        return self.u, self.v, self.w

    def visualize_body_frame(self, position, t):
        x = position[0]
        y = position[1]
        z = position[2]

        if self.do_blit:
            self.fig.canvas.restore_region(self.background)
            self.time_text.set_position((x, y, z))
            self.time_text.set_text('time=%s' % t)
            self.quiver.remove()
            self.quiver = self.ax.quiver(x, y, z, self.u, self.v, self.w, color=['b', 'b', 'r'])
            self.ax.draw_artist(self.quiver)
            self.ax.draw_artist(self.time_text)
            self.fig.canvas.blit(self.ax.bbox)
        else:
            self.ax.quiver(x, y, z, self.u, self.v, self.w)
        plt.draw()

    def end_job(self):
        plt.show(block=True)


class ParallelVisualizer(Visualizer):
    def __init__(self, do_blit=True, azimuth_init=45, elevation_init=45):
        super().__init__(self)

    def call_back(self):
        while self.pipe.poll():
            command = self.pipe.recv()
            if command is None:
                print("No data yet")
            else:
                self.transform_base_vectors([command[3], command[4], command[5]])
                self.visualize_body_frame([command[0], command[1], command[2]], command[6])
        return True

    def __call__(self, pipe):
        self.pipe = pipe
        self.timer = self.fig.canvas.new_timer(interval=10)
        self.timer.add_callback(self.call_back())
        self.timer.start()
    def terminate(self):
        plt.close('all')

# visualizer = Visualizer()
# visualizer(np.array([0, 0, 0]), np.array([0, 0, np.pi / 4]))
