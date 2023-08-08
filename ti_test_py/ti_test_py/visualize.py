import os
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import juggle_axes
from TI import get_data

FLOOR = -10
CEILING = 10

class AnimatedScatter(object):
    def __init__(self):
        self.detected_points=get_data(command_port="/dev/ttyUSB0", data_port="/dev/ttyUSB1", cfg_path=os.getcwd()+'/src/ti_mmwave_sensor/mmwave_sensor/ti_test_py/cfg/staticRetention.cfg')
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111,projection = '3d')
        self.ani = animation.FuncAnimation(self.fig, self.update, interval=int(self.detected_points._ms_per_frame/10), frames=np.arange(0,20),
                                           init_func=self.setup_plot, blit=True)

    def setup_plot(self):
        self.ax.set_xlim3d(FLOOR, CEILING)
        self.ax.set_ylim3d(FLOOR, CEILING)
        self.ax.set_zlim3d(FLOOR, CEILING)
        self.scat = self.ax.scatter([],[],[])
        return self.scat,

    def update(self, i):
        action = self.detected_points._read()
        try:
            data=next(action)
            time.sleep(float(self.detected_points._ms_per_frame/2000))
            print(data.shape)
            x, y, z, i = np.transpose(data)
            colors = self.color_map(i)
            self.scat = self.ax.scatter(x, y, z, c=colors, s=3, animated=True)
            self.scat._offsets3d = juggle_axes(x, y, z, 'z')
        except Exception as exception:
            print(exception)
        plt.draw()
        return self.scat,

    def save(self, path=os.getcwd()+"/src/ti_mmwave_sensor/mmwave_sensor/ti_test_py/animation.gif"):
        self.ani.save(path, dpi=300, fps=30, writer="imagemagick")
        self.detected_points.close()

    def show(self):
        plt.show()
    
    @staticmethod
    def color_map(channel): 
        dmin, dmax = np.nanmin(channel), np.nanmax(channel)
        cmo = plt.cm.get_cmap("Blues")
        cs = list()
        for i in range(len(channel)):
            j = int(256*channel[i]/(dmax-dmin))
            c = cmo(j)
            cs.append(c)
        cs = np.array(cs)
        return cs

if __name__ == "__main__":
    a = AnimatedScatter()
    a.save()
