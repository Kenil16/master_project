#!/usr/bin/python
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

class graphics_plot:

    def __init__(self):
        pass


    #A function to plot the Kalman filter against data points
    
    def plot_kf_data(self,without_kf,with_kf,time,title,xlabel,ylabel,fig_path_name):
        
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

        ax.scatter(time,without_kf,label='Data points')
        ax.plot(time,with_kf,label='Kalman filter',color='red',markersize=2)
        ax.set_title(title)
        ax.legend()
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.savefig(fig_path_name)

    def plot_aruco_pose_est(self, file_name):
        
        data = []
        with open(file_name,"r") as text_file:
            for line in text_file:
                items = line.split(' ')
                items[-1] = items[-1].strip()
                #[float(item) for item in items]
                data.append(items)

        x = np.array([item[0] for item in data])
        std_x = np.array([item[1] for item in data])
        error_x = np.array([item[2] for item in data])

        y = np.array([item[3] for item in data])
        std_y = [item[4] for item in data]
        error_y = [item[5] for item in data]
        
        std_z = [item[7] for item in data]
        error_z = [item[8] for item in data]

        fig, ax = plt.subplots(constrained_layout=True)
        ax.pcolormesh(x, y, error_x, shading='gouraud', vmin=error_x.min(), vmax=error_x.max())
        _annotate(ax, x, y, "shading='gouraud'; X, Y same shape as Z")
        
        plt.show()


if __name__ == "__main__":

    gp = graphics_plot()
    gp.plot_aruco_pose_est('aruco_pose_estimation_test/test1.txt')
