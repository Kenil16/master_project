#!/usr/bin/python
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import griddata
from scipy.interpolate import interp1d

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

    
    def plot_data(self, file_name, title, xlabel, ylabel, fig_path_name, pos_index, set_pos_index, time_index):
        
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
        pos = np.array([item[pos_index] for item in self.data])
        setpoint = np.array([item[set_pos_index] for item in self.data])
        time = np.array([item[time_index] for item in self.data])

        i = interp1d(time, pos, kind='cubic')
        ax.plot(time,setpoint,'--',label='Setpoint',color='blue',markersize=1)
        ax.plot(time, i(time), label='Estimated ArUco marker pos',color='red',markersize=1)
        ax.set_title(title)
        ax.legend()
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.savefig(fig_path_name)

if __name__ == "__main__":
    gp = graphics_plot()
    gp.plot_data('data_route3.txt','Setpoint vs estimated ArUco marker position','Time [s]', 'x [m]','error_x.png',0,1,6)
    gp.plot_data('data_route3.txt','Setpoint vs estimated ArUco marker position','Time [s]', 'y [m]','error_y.png',2,3,6)
    gp.plot_data('data_route3.txt','Setpoint vs estimated ArUco marker position','Time [s]', 'z [m]','error_z.png',4,5,6)

