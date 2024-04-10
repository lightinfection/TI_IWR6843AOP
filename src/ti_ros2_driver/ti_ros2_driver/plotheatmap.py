import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as pat
import scipy.interpolate as spi
import time

class plotHM():
    def __init__(self, if_ra, if_rd, hang=100):
        self.figure = plt.figure(figsize=(10,5))
        self.num_hm = [if_ra, if_rd].count(True)
        self.if_ra_ = if_ra
        self.if_rd_ = if_rd
        self.periodic = hang
        self.wait_times = 0
        self.ax_1 = self.figure.add_subplot(121)
        if self.num_hm == 2:
            self.ax_2 = self.figure.add_subplot(122)
        self.animation = animation.FuncAnimation(self.figure, self.update, interval=self.periodic)

    def get(self, RA, RD):
        self.rd_data = RD
        self.ra_data = RA

    def init_hm(self, range_bins, doppler_bins, range_max, doppler_max, range_res, doppler_bias, angle_bins=None, grid_res=None):
        if self.if_ra_:
            t = np.array(range(-angle_bins//2 + 1, angle_bins//2)) * (2 / angle_bins)
            t = np.arcsin(t)
            r = np.array(range(range_bins)) * range_res
            self.x = np.array([r]).T * np.sin(t)
            self.y = np.array([r]).T * np.cos(t)
            xi = np.linspace(-range_max/2, range_max/2, grid_res)
            yi = np.linspace(0, range_max, grid_res)
            self.xi, self.yi = np.meshgrid(xi, yi)
        if self.num_hm == 2:
            self.plot_RD(range_bins, doppler_bins, range_max, doppler_max, doppler_bias)
            self.plot_RA(range_bins, range_max, grid_res)
        else:
            if self.if_rd_: self.plot_RD(range_bins, doppler_bins, range_max, doppler_max, doppler_bias)
            if self.if_ra_: self.plot_RA(range_bins, range_max, grid_res)

    def plot_RD(self, range_bins, doppler_bins, range_max, doppler_max, doppler_bias):
        self.ax_1.set_title('Range-Doppler FFT Heatmap', fontsize=10)
        self.ax_1.set_xlabel('Radial velocity [m/s]')
        self.ax_1.set_ylabel('Longitudinal distance [m]')
        self.ax_1.grid(color='white', linestyle=':', linewidth=0.5)
        self.im1 = self.ax_1.imshow(np.reshape([0,]*range_bins*doppler_bins, (range_bins, doppler_bins)),
                    cmap=plt.cm.jet,
                    interpolation="quadric",
                    aspect= doppler_max/range_max,
                    extent=[0-doppler_bias, doppler_max-doppler_bias, 0, 0+range_max],
                    alpha=.95
                    )
        self.ax_1.plot([0,0],[0,0+range_max], color='white', linestyle=':', linewidth=0.5, zorder=1)

    def plot_RA(self, range_bins, range_max, grid_res):
        range_width = range_max/2
        if self.if_rd_:
            self.ax_2.set_title('Range-Azimuth FFT Heatmap in cartesian coordinates at 0 doppler', fontsize=10)
            self.ax_2.set_xlabel('Lateral distance along [m]')
            self.ax_2.set_ylabel('Longitudinal distance along [m]')
            self.im2 = self.ax_2.imshow(((0,)*grid_res,) * grid_res, 
                    cmap=plt.cm.jet, 
                    extent=[-range_width, +range_width, 0, range_max], 
                    alpha=.95
                    )
            self.ax_2.plot([0, 0], [0, range_max], color='white', linewidth=0.5, linestyle=':', zorder=1)
            self.ax_2.plot([0, -range_width], [0, range_width], color='white', linewidth=0.5, linestyle=':', zorder=1)
            self.ax_2.plot([0, +range_width], [0, range_width], color='white', linewidth=0.5, linestyle=':', zorder=1)
            self.ax_2.set_xlim(-range_width, range_width)
            self.ax_2.set_ylim(0, range_max)
            for i in range(1, int(range_max)+1):
                self.ax_2.add_patch(pat.Arc((0, 0), width=i*2, height=i*2, angle=90, theta1=-90, theta2=90, color='white', linewidth=0.5, linestyle=':', zorder=1))
        else:
            self.ax_1.set_title('Range-Azimuth FFT Heatmap in cartesian coordinates at 0 doppler', fontsize=10)
            self.ax_1.set_xlabel('Lateral distance along [m]')
            self.ax_1.set_ylabel('Longitudinal distance along [m]')
            self.im1 = self.ax_1.imshow(((0,)*range_bins,) * range_bins, 
                    cmap=plt.cm.jet, 
                    extent=[-range_width, +range_width, 0, range_max], 
                    alpha=.95
                    )
            self.ax_1.plot([0, 0], [0, range_max], color='white', linewidth=0.5, linestyle=':', zorder=1)
            self.ax_1.plot([0, -range_width], [0, range_width], color='white', linewidth=0.5, linestyle=':', zorder=1)
            self.ax_1.plot([0, +range_width], [0, range_width], color='white', linewidth=0.5, linestyle=':', zorder=1)
            self.ax_1.set_xlim(-range_width, range_width)
            self.ax_1.set_ylim(0, range_max)
            for i in range(1, int(range_max)+1):
                self.ax_1.add_patch(pat.Arc((0, 0), width=i*2, height=i*2, angle=90, theta1=-90, theta2=90, color='white', linewidth=0.5, linestyle=':', zorder=1))

    def update(self, i):
        global shutdown
        if self.wait_times > 5:
            print("no available fft array captured.")
            self.close()
            return
        try:
            if self.if_rd_:
                rd_fft = np.fft.fftshift(self.rd_data, axes=(1,))
                self.im1.set_array(np.flip(rd_fft[:,1:], axis=0))
                self.im1.autoscale()
            if self.if_ra_:
                ra_fft  = np.fft.fftshift(self.ra_data, axes=(1,))
                ra_fft = ra_fft[:,1:]
                zi = spi.griddata((self.x.ravel(), self.y.ravel()), ra_fft.ravel(), (self.xi, self.yi), method='linear')
                if self.if_rd_:
                    self.im2.set_array(zi[::-1,::-1])
                    self.im2.autoscale()
                else: 
                    self.im1.set_array(zi[::-1,::-1])
                    self.im1.autoscale()
            self.wait_times = 0
        except Exception as e:
            print(e)
            self.wait_times += 1
            time.sleep(self.periodic)
    
    def show(self):
        plt.show()

    def close(self):
        self.animation.pause()
        plt.clf()
        plt.close()

    #### to do: add a signal-induced stopping mechanism for animation, otherwise, got stuck when stopping ros.