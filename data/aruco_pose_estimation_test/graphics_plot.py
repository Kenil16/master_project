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
    def plot_kf_data(self, file_name, title, xlabel, ylabel, fig_path_name, pos_index, kf_index, time_index):
        
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
        kf = np.array([item[kf_index] for item in self.data])
        time = np.array([item[time_index] for item in self.data])

        ax.scatter(time, pos, label='Data points')
        ax.plot(time, kf, label='Kalman filter',color='red',markersize=2)
        ax.set_title(title)
        ax.legend()
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.savefig(fig_path_name)

    def plot_aruco_pose_est(self, fig_name, file_name, x_label, y_label, title, index):
        
        data = []
        with open(file_name,"r") as text_file:
            for line in text_file:
                items = line.split(' ')
                items[-1] = items[-1].strip()
                str_to_float = [float(item) for item in items]
                data.append(str_to_float)


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


        METHODS = [ 'nearest', 'bilinear', 'bicubic', 'hermite' ]
        FORMATS = [ 'png', 'pdf', 'svg' ]
        COLORS  = 'viridis'
        
        x = np.array([item[0] for item in data])
        y = np.array([item[3] for item in data])
        z = np.array([item[index] for item in data])

        mesh= np.meshgrid(x, y)
        
        min_z, max_z = np.amin(z), np.amax(z)
        z = np.reshape(z, (-1, 9))
        print(z)
        
        #ax = fig.add_axes([0.125, 0.175, 0.75, 0.75])
        plt.imshow(z, interpolation=METHODS[2], cmap=COLORS, vmin=min_z, vmax=max_z)
        #ax.plot(mesh[0], mesh[1], marker='.', ms=4, color='k', lw=0)
        
        #plt.xlim(x.min(), x.max()+10.5)
        
        #plt.ylim(y.min()-0.5, y.max()+0.5)
        plt.xlabel(x_label)
        plt.ylabel(y_label)
        #cax = fig.add_axes([0.125, 0.075, 0.75, 0.03])
        plt.colorbar(orientation='vertical')        
        plt.savefig(fig_name)

if __name__ == "__main__":
    gp = graphics_plot()

    #Plot estimated ArUco positions vs Kalman filter to see the difference and avantage of using this filter
    gp.plot_kf_data('aruco_pos_kf/data.txt','Estimated ArUco marker position','Time [s]', 'x [m]','aruco_pos_kf/kf_pos_x.png',0,3,6)
    gp.plot_kf_data('aruco_pos_kf/data.txt','Estimated ArUco marker position','Time [s]', 'y [m]','aruco_pos_kf/kf_pos_y.png',1,4,6)
    gp.plot_kf_data('aruco_pos_kf/data.txt','Estimated ArUco marker position','Time [s]', 'z [m]','aruco_pos_kf/kf_pos_z.png',2,5,6)

    #Plot estimated AruCo positions from test 
    gp.plot_aruco_pose_est('aruco_pose_estimation_test/std_x.png','aruco_pose_estimation_test/test1.txt','y [m]', 'x [m]','STD x in ArUco position',1)
    gp.plot_aruco_pose_est('aruco_pose_estimation_test/std_y.png','aruco_pose_estimation_test/test1.txt','y [m]', 'x [m]','STD y in ArUco position',4)
    gp.plot_aruco_pose_est('aruco_pose_estimation_test/std_z.png','aruco_pose_estimation_test/test1.txt','y [m]', 'x [m]','STD z in ArUco position',7)
    gp.plot_aruco_pose_est('aruco_pose_estimation_test/error_x.png','aruco_pose_estimation_test/test1.txt','y [m]', 'x [m]','Error x in ArUco position',2)
    gp.plot_aruco_pose_est('aruco_pose_estimation_test/error_y.png','aruco_pose_estimation_test/test1.txt','y [m]', 'x [m]','Error y in ArUco position',5)
    gp.plot_aruco_pose_est('aruco_pose_estimation_test/error_z.png','aruco_pose_estimation_test/test1.txt','y [m]', 'x [m]','Error z in ArUco position',8)
