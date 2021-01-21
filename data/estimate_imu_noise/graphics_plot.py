#!/usr/bin/python
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import griddata

class graphics_plot:

    def __init__(self):

        self.data = []
        pass

    def read_data(self, file_name):
        
        with open(file_name,"r") as text_file:
            self.data = []
            for line in text_file:
                items = line.split(' ')
                items[-1] = items[-1].strip()
                str_to_float = [float(item) for item in items]
                self.data.append(str_to_float)

    
    #A function to plot the Kalman filter against data points    
    def plot_kf_data(self, file_name, title, xlabel, ylabel, fig_path_name, pos_index, time_index):
        
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

        self.read_data(file_name)
        data = np.array([item[pos_index] for item in self.data])
        time = np.array([item[time_index] for item in self.data])

        ax.plot(time, data, label='Sensor noise',color='red',markersize=2)
        ax.set_title(title)
        ax.legend()
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.savefig(fig_path_name)


if __name__ == "__main__":
    gp = graphics_plot()

    #Plot estimated ArUco positions vs Kalman filter to see the difference and avantage of using this filter
    gp.plot_kf_data('data.txt','Estimated sensor noise for accelerometer','Time [s]', 'x [m]','accelerometer_noise_x.png',0,6)
    gp.plot_kf_data('data.txt','Estimated sensor noise for accelerometer','Time [s]', 'y [m]','accelerometer_noise_y.png',1,6)
    gp.plot_kf_data('data.txt','Estimated sensor noise for accelerometer','Time [s]', 'z [m]','accelerometer_noise_z.png',2,6)


