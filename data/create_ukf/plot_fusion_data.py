from ukf import UKF
import csv
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import


class plot_fusion_data():
    
    def __init__(self):

        self.data = []

        self.read_data('../sensor_fusion_data.txt')
        self.mx = np.array([item[0] for item in self.data])
        self.my = np.array([item[1] for item in self.data])
        self.mz = np.array([item[2] for item in self.data])

        self.macc_x = np.array([item[3] for item in self.data])
        self.macc_y = np.array([item[4] for item in self.data])

        self.mroll = np.array([item[5] for item in self.data])
        self.mpitch = np.array([item[6] for item in self.data])
        self.myaw = np.array([item[7] for item in self.data])

        self.mgyro_x = np.array([item[8] for item in self.data])
        self.mgyro_y = np.array([item[9] for item in self.data])
        self.mgyro_z = np.array([item[10] for item in self.data])
        
        self.mbaro = np.array([item[11] for item in self.data])
        self.mbaro_corrected = np.array([item[12] for item in self.data])
        self.mbaro_bias = np.array([item[13] for item in self.data])
        
        self.mvision_x = np.array([item[14] for item in self.data])
        self.mvision_y = np.array([item[15] for item in self.data])
        self.mvision_z = np.array([item[16] for item in self.data])
        
        self.mtime = np.array([item[17] for item in self.data])
        
        self.g_roll = np.array([item[18] for item in self.data])
        self.g_pitch = np.array([item[19] for item in self.data])
        self.g_yaw = np.array([item[20] for item in self.data])
        
        self.g_x = np.array([item[21] for item in self.data])
        self.g_y = np.array([item[22] for item in self.data])
        self.g_z = np.array([item[23] for item in self.data])
         
    def read_data(self, file_name):
        init = True
        seq = 0
        with open(file_name,"r") as text_file:
            for line in text_file:
                items = line.split(' ')
                items[-1] = items[-1].strip()
                str_to_float = [float(item) for item in items]
                self.data.append(str_to_float)

    def plot_position(self):

        fig, ax = plt.subplots()
        ax.set_axisbelow(True)
        ax.set_facecolor('#E6E6E6')
        plt.grid(color='w', linestyle='solid')

        for spine in ax.spines.values():
            spine.set_visible(False)

        ax.xaxis.tick_bottom()
        ax.yaxis.tick_left()
        ax.tick_params(colors='gray', direction='out')

        for tick in ax.get_xticklabels():
            tick.set_color('gray')
        for tick in ax.get_yticklabels():
            tick.set_color('gray')

        plt.scatter(self.mvision_x, self.mvision_y, s=10, label='GPS Measurements')
        plt.scatter(self.mx, self.my, s=2, label='EKF Position', c='k')

        # Start/Goal
        plt.scatter(self.mx[0], self.my[0], s=60, label='Start', c='g')
        plt.scatter(self.mx[-1], self.my[-1], s=60, label='Goal', c='r')

        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.title('Position_xy')
        plt.legend(loc='best')
        plt.axis('equal')

        plt.savefig('Positionxy.png')

    def plot_position_2d(self):

        fig, ax = plt.subplots()
        ax.set_axisbelow(True)
        ax.set_facecolor('#E6E6E6')
        plt.grid(color='w', linestyle='solid')

        for spine in ax.spines.values():
            spine.set_visible(False)

        ax.xaxis.tick_bottom()
        ax.yaxis.tick_left()
        ax.tick_params(colors='gray', direction='out')

        for tick in ax.get_xticklabels():
            tick.set_color('gray')
        for tick in ax.get_yticklabels():
            tick.set_color('gray')

        plt.scatter(self.mtime, self.mbaro_corrected, s=2, label='Corrected altimeter')
        plt.scatter(self.mtime, self.mbaro, s=2, label='Altimeter')
        plt.scatter(self.mtime, self.mvision_z, s=2, label='Vision altitude')
        plt.scatter(self.mtime, self.g_z, s=2, label='Ground truth')
        plt.scatter(self.mtime, self.mz, s=2, label='EKF altitude')

        plt.xlabel('Z [m]')
        plt.title('Positionz')
        plt.legend(loc='best')
        #plt.axis('equal')

        plt.ylim(ymax = 5, ymin = 0)
        plt.savefig('Positionz.png')

    def plot_data_vs_ground_truth(self, title, x_label, y_label, fig_name, x, y, g_y, labels, g_labels):

        fig, ax = plt.subplots()
        ax.set_axisbelow(True)
        ax.set_facecolor('#E6E6E6')
        plt.grid(color='w', linestyle='solid')

        for spine in ax.spines.values():
            spine.set_visible(False)

        ax.xaxis.tick_bottom()
        ax.yaxis.tick_left()
        ax.tick_params(colors='gray', direction='out')

        for tick in ax.get_xticklabels():
            tick.set_color('gray')
        for tick in ax.get_yticklabels():
            tick.set_color('gray')

        for label, y in zip(labels, y):
            plt.plot(x, y, linewidth=0.5, label=label)

        for label, y in zip(g_labels, g_y):
            plt.plot(x, y, linewidth=0.5,linestyle='-.', label=label)
        
        plt.xlabel(x_label)
        plt.ylabel(y_label)
        plt.title(title)
        plt.legend(loc='best')
        plt.savefig(fig_name)
        
    def plot_data(self, title, x_label, y_label, fig_name, x, y, labels):

        fig, ax = plt.subplots()
        ax.set_axisbelow(True)
        ax.set_facecolor('#E6E6E6')
        plt.grid(color='w', linestyle='solid')

        for spine in ax.spines.values():
            spine.set_visible(False)

        ax.xaxis.tick_bottom()
        ax.yaxis.tick_left()
        ax.tick_params(colors='gray', direction='out')

        for tick in ax.get_xticklabels():
            tick.set_color('gray')
        for tick in ax.get_yticklabels():
            tick.set_color('gray')

        for label, y in zip(labels, y):
            plt.plot(x, y, linewidth=0.5, label=label)
        
        plt.xlabel(x_label)
        plt.ylabel(y_label)
        plt.title(title)
        plt.legend(loc='best')
        plt.savefig(fig_name)

if __name__ == "__main__":

    pfd = plot_fusion_data()

    #Position vs ground truth
    labels = ['x','y','z']
    g_labels = ['g_x', 'g_y', 'g_z']
    ys = [pfd.mx, pfd.my, pfd.mz]
    g_ys = [pfd.g_x, pfd.g_y, pfd.g_z]
    pfd.plot_data_vs_ground_truth ('Position', 'Time [s]', r'Position [$m$]', 'position.png', pfd.mtime, ys, g_ys, labels, g_labels)

    #Acceleration from IMU
    labels = ['Acceleration in x','Acceleration in y']
    ys = [pfd.macc_x, pfd.macc_y]
    pfd.plot_data('Acceleration [IMU]', 'Time [s]', r'Acceleration [$(\frac{m}{s})$]', 'acceleration.png', pfd.mtime, ys, labels)
    
    labels = ['Roll','Pitch']
    g_labels = ['g_roll','g_pitch']
    ys = [pfd.mroll, pfd.mpitch]
    g_ys = [pfd.g_roll, pfd.g_pitch]
    pfd.plot_data_vs_ground_truth('Angle [IMU and vision fusion]', 'Time [s]', r'Angle [Degress]', 'orientation.png', pfd.mtime, ys, g_ys, labels, g_labels)
    
    labels = ['x','y','z']
    ys = [pfd.mgyro_x, pfd.mgyro_y, pfd.mgyro_z]
    pfd.plot_data('Angular velocity [IMU]', 'Time [s]', r'Angular velocity [$(\frac{r}{s})$]', 'angular_velocity.png', pfd.mtime, ys, labels)
    
    pfd.plot_position()
    pfd.plot_position_2d()
    
    labels = ['Altimeter bias']
    ys = [pfd.mbaro_bias]
    pfd.plot_data('Baro bias [Altimeter]', 'Time [s]', r'Altitude [Meters]', 'baro_bias.png', pfd.mtime, ys, labels) 
