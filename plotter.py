from operator import attrgetter as get
from types import SimpleNamespace
import multiprocessing as mp

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import style, use


class Plotter(object):
    def __init__(self, rocket, time_series_plots=[], show_rotation=False, show_traj=False, custom_plot=None):
        self.obj = rocket
        self.time_series_plots = time_series_plots
        self.show_rotation = show_rotation
        self.show_traj = show_traj
        self.custom_plot = custom_plot

        self.total_plots = len(self.time_series_plots) + \
            self.show_rotation + self.show_traj

        self.counter = 0
        self.axs, self.fig = None, None
        self.d = {}
        for p in self.time_series_plots:
            self.d[p] = []
        if self.show_rotation:
            self.d['Simulation'] = None
        if self.show_traj:
            self.d['Trajectory'] = None
        if self.custom_plot is not None:
            if type(self.custom_plot) not in [list, tuple, np.ndarray]:
                raise("custom_plot: pass list of len()=2 of property names")
            self.total_plots += 1
            self.d['Custom'] = [[],[]]

        self.configParams()

        plt.ion()
        # use('Qt5agg')
        style.use('Solarize_Light2')

        self.fig = plt.figure()
        self.fig.canvas.mpl_connect('close_event', self.handle_close)
        self.fig.canvas.mpl_connect('motion_notify_event', self.handle_move)
        self.fig.canvas.set_window_title('Reentry simulator')
        self.fig.set_size_inches(9, 9, forward=True)

        def createOrigin(ax):
            x = ax.quiver([0], [0], [0], [0], [0], [0], color='r', linewidth=3)
            y = ax.quiver([0], [0], [0], [0], [0], [0], color='g', linewidth=3)
            z = ax.quiver([0], [0], [0], [0], [0], [0], color='b', linewidth=3)
            return [x,y,z]

        def createRotation(ax):
            self.vel_dir = ax.quiver([0], [0], [0], [0], [0], [0], color='c', linewidth=1.5)
            self.rot_o = createOrigin(ax)
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.set_xlim(-1, 1)
            ax.set_ylim(-1, 1)
            ax.set_zlim(-1, 1)
            self.rot_ax = ax

        def createTraj(ax):
            self.traj, = ax.plot([], [], [], 'k-', linewidth=1, alpha=1)
            self.pos_marker, = ax.plot([], [], [], 'm.', markersize=8)
            u, v = np.mgrid[0:2*np.pi:10j, 0:np.pi:10j]     # u lon, v lat
            earth_x = self.obj.planet.R*np.cos(u)*np.sin(v)
            earth_y = self.obj.planet.R*np.sin(u)*np.sin(v)
            earth_z = self.obj.planet.R*np.cos(v)
            self.planet = ax.plot_surface(earth_x, earth_y, earth_z, color=self.obj.planet.colour)
            self.zoom = 1
            self.fig.canvas.mpl_connect('scroll_event', self.handle_scroll)
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            self.traj_ax = ax

        def createCustom(ax):
            self.custom, = ax.plot([], [], 'r-')
            ax.set_xlabel(self.params[self.custom_plot[0]]['title'])
            ax.set_ylabel(self.params[self.custom_plot[1]]['title'])
            
        rows = np.floor(self.total_plots**0.5)
        cols = np.ceil(self.total_plots / rows)
        self.fig.set_size_inches(4*cols, 4*rows, forward=True)

        self.axs, self.lines = [], []
        done, done1, done2 = False, False, False
        for i in range(self.total_plots):
            if self.show_rotation and not done:
                ax = plt.subplot(rows, cols, i+1, projection='3d')
                createRotation(ax)
                done = True
            elif self.show_traj and not done1:
                ax = plt.subplot(rows, cols, i+1, projection='3d')
                createTraj(ax)
                done1 = True
            elif self.custom_plot is not None and not done2:
                ax = plt.subplot(rows, cols, i+1)
                createCustom(ax)
                done2 = True
            else:
                _property = self.time_series_plots[i-self.show_rotation-self.show_traj-bool(self.custom_plot)]
                ax = plt.subplot(rows, cols, i+1, projection=None)
                if 'num' in self.params[_property]:
                    l = [ax.plot([], [])[0] for i in range(self.params[_property]['num'])]
                else:
                    l = ax.plot([], [])
                self.lines.append(l)
                ax.set_title(_property)
                ax.set_xlabel('time /s')
                # self.modifyAx(ax, self.params[_property])
            self.axs.append(ax)
        self.fig.tight_layout()

    def update(self, cmd):

        if self.axs is None:
            exit()
            return

        done, done1, done2 = False, False, False
        for i, ax in enumerate(self.axs, start=-self.show_rotation-self.show_traj-bool(self.custom_plot)):
            if self.show_rotation and not done:
                for l, val in zip(self.rot_o, cmd['rotation'][0]):
                    l.set_segments([[[0, 0, 0], [val[0], val[1], val[2]]]])
                self.vel_dir.set_segments([[[0,0,0], cmd['rotation'][1]/self.obj.speed]])
                done = True
            elif self.show_traj and not done1:
                pos = cmd['traj'][0]
                trajs = cmd['traj'][1]

                self.traj.set_data_3d(trajs[:, 0], trajs[:, 1], trajs[:, 2])
                self.pos_marker.set_data_3d([pos[0]], [pos[1]], [pos[2]])

                _min, _max = np.amin(trajs, axis=0), np.amax(trajs, axis=0)
                max_range = np.array(_max-_min).max() * 0.5
                _min = pos - max_range/2/self.zoom
                _max = pos + max_range/2/self.zoom

                if self.counter % 30 == 0:
                    la, lo = self.obj.latitude*np.pi/180, self.obj.longitude*np.pi/180
                    res = 30 * np.pi/180
                    u, v = np.mgrid[lo-res:lo+res:10j, 
                                    np.pi/2-la-res:np.pi/2-la+res:10j]     # u lon, v lat
                    earth_x = self.obj.planet.R*np.cos(u)*np.sin(v)
                    earth_y = self.obj.planet.R*np.sin(u)*np.sin(v)
                    earth_z = self.obj.planet.R*np.cos(v)
                    self.planet.remove()
                    self.planet = ax.plot_surface(earth_x, earth_y, earth_z, color=self.obj.planet.colour)
                ax.set_xlim(_min[0], _max[0])
                ax.set_ylim(_min[1], _max[1])
                ax.set_zlim(_min[2], _max[2])
                done1 = True
            elif self.custom_plot and not done2:
                self.d['Custom'][0].append(cmd['custom_plot'][0])
                self.d['Custom'][1].append(cmd['custom_plot'][1])     
                self.custom.set_data(self.d['Custom'][0],self.d['Custom'][1])          
                ax.relim()
                ax.autoscale_view(True,True,True)
                if 'ylim' in self.params[self.custom_plot[0]]:
                    ax.set_xlim(self.params[self.custom_plot[0]]['ylim'])
                if 'ylim' in self.params[self.custom_plot[1]]:
                    ax.set_ylim(self.params[self.custom_plot[1]]['ylim'])
                done2 = True
            else:
                p = self.time_series_plots[i]
                val = cmd['time_series'][i]
                if type(val) == np.ndarray:
                    if self.d[p] == []:
                        self.d[p] = [[v] for v in val]
                    else:
                        [self.d[p][i].append(v) for i, v in enumerate(val)]
                    ts = np.linspace(0, self.obj.clock, len(self.d[p][0]))
                    for j, line in enumerate(self.lines[i]):
                        line.set_data(ts, self.d[p][j])
                else:
                    self.d[p].append(val)
                    ts = np.linspace(0, self.obj.clock, len(self.d[p]))
                    for line in self.lines[i]:
                        line.set_data(ts, self.d[p])
                ax.relim()
                ax.autoscale_view(True,True,True)
                self.modifyAx(ax, self.params[p])

        self.counter += 1
        plt.draw()
        self.fig.canvas.start_event_loop(0.0001)

    def modifyAx(self, ax, dct):
        if 'xlim' in dct:
            ax.set_xlim(dct['xlim'])
        if 'ylim' in dct:
            ax.set_ylim(dct['ylim'])
        if 'zlim' in dct:
            ax.set_zlim(dct['zlim'])
        if 'ylab' in dct:
            ax.set_zlim(dct['ylab'])
        if 'label' in dct:
            ax.legend(dct['label'], loc=0)
        if 'title' in dct:
            ax.set_title(dct['title'])

    def handle_close(self, evt):
        print('closed plotting window')
        plt.ioff()
        del self.axs, self.fig
        plt.close('all')
        self.axs, self.fig = None, None

    def handle_scroll(self, evt):
        if evt.inaxes == self.traj_ax:
            self.zoom += evt.step
            self.zoom = np.clip(self.zoom, 0.1, 50)

    def handle_move(self, evt):
        if not (self.show_rotation and self.show_traj):
            return
        if evt.inaxes == self.traj_ax:
            if self.traj_ax.button_pressed in self.traj_ax._rotate_btn:
                self.rot_ax.view_init(elev=self.traj_ax.elev, azim=self.traj_ax.azim)
        elif evt.inaxes == self.rot_ax:
            if self.rot_ax.button_pressed in self.rot_ax._rotate_btn:
                self.traj_ax.view_init(elev=self.rot_ax.elev, azim=self.rot_ax.azim)

    def configParams(self):
        self.params = {
            'altitude': {
                'ylim': (0, None),
                'title': 'Altitude',
            },
            'attackAngle': {
                'ylim': (0, np.pi+0.01),
                'title': 'Angle of Attack'
            },
            'ddrot': {
                'num': 3,
                'title': 'Angular Accelerations',
                'label': ['pitch', 'yaw', 'roll']
            },
            'deltaV': {
                'ylim': (0, None),
                'title': 'Delta V'
            },
            'eulerAngles': {
                'num': 3,
                'ylim': (-180, 180),
                'title': 'Euler Angles',
                'label': ['pitch', 'yaw', 'roll']
            },
            'eulerVels': {
                'num': 3,
                'title': 'Euler Angles',
                'label': ['pitch', 'yaw', 'roll']
            },
            'Gs': {
                'ylim': (0, 10),
                'title': 'G-force'
            },
            'inertia': {
                # TODO
            },
            'latitude': {
                'ylim': (-90, 90),
                'title': 'Latitude',
            },
            'longitude': {
                'ylim': (-180, 180),
                'title': 'Longitude'
            },
            'machNum': {
                'ylim': (0, None),
                'title': 'Mach Number'
            },
            'mass': {
                'ylim': (0, None),
                'title': 'Mass'
            },
            'orbitalAngMomentum': {
                'num': 3,
                'title': 'Angular Momentum Vector',
                'label': ['x', 'y', 'z']
            },
            'radialSpeed': {
                'ylim': (0, None),
                'title': 'Radial Speed'
            },
            'speed': {
                'ylim': (0, None),
                'title': 'Speed'
            },
            'Simulation': {
                'label': ['x', 'y', 'z']
            },
            'Trajectory': {
            }
        }

    def reqDataFormat(self):
        info = {
            'time_series': self.time_series_plots,
        }
        if self.show_traj:
            info['traj'] = ['pos', 'currentTraj']
        if self.show_rotation:
            info['rotation'] = ['localAxes', 'vel']
        if self.custom_plot:
            info['custom_plot'] = self.custom_plot
        return info
