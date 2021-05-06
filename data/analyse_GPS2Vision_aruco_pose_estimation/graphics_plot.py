#!/usr/bin/python
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import griddata
from mpl_toolkits.axes_grid1 import ImageGrid
import matplotlib as mpl
from mpl_toolkits.axes_grid1 import make_axes_locatable
from matplotlib.patches import Circle, Wedge, Polygon

class graphics_plot:

    def __init__(self):

        self.data = []

    def read_data(self, file_name):
        
        with open(file_name,"r") as text_file:
            self.data = []
            for line in text_file:
                items = line.split(' ')
                items[-1] = items[-1].strip()
                str_to_float = [float(item) for item in items]
                self.data.append(str_to_float)

    def plot_aruco_pose_est(self, fig_name, file_name, x_label, y_label, title, index):
        
        
        plt.rcParams.update({
        "text.usetex": True})
        
        data = []
        with open(file_name,"r") as text_file:
            for line in text_file:
                items = line.split(' ')
                items[-1] = items[-1].strip()
                str_to_float = [float(item) for item in items]
                data.append(str_to_float)


        fig, ax = plt.subplots(figsize=(7.4,5),constrained_layout=True)
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
        y = np.array([item[1] for item in data])
        z = np.array([item[index] for item in data])
        
        print(z)
        min_z, max_z = np.amin(z), np.amax(z)
        
        z = np.reshape(z, (-1, 9)) #15
        print(z)
        
        im =plt.imshow(z, interpolation=METHODS[2], cmap=COLORS, vmin=min_z, vmax=max_z)

        x_ = range(0,9, 1)
        y_ = range(0,7, 1)
       
        xticks = ['-4', '-3', '-2', '-1', '0', '1', '2', '3','4']
        yticks = ['-2', '-3', '-4', '-5', '-6', '-7', '-8']
        
        plt.xticks(x_, xticks,fontsize=20)
        plt.yticks(y_, yticks,fontsize=20)

        #Plot black circles to illustrate where the stimations where made
        for x in range(9):
            for y in range(8):
                cir1=plt.Circle((x,y), 0.03, color='k')
                #cir2=plt.Circle((4, 3.0), 0.1, color='black')
                
                ax.add_patch(cir1)
                #ax.add_patch(cir2)
                
        #Show safe area to change from GPS to vision
        wedge = Wedge((4, -2), 5, 0, 180, linestyle='--', fill=False, color='xkcd:sky blue')

        #Safe GPS2Vision transitiom
        ax.add_patch(wedge)
        
        plt.xlabel(y_label,fontsize=20)
        plt.ylabel(x_label,fontsize=20)
        plt.title(title,fontsize=20)
        cbar = plt.colorbar(im, cax = fig.add_axes([0.88, 0.1525, 0.03, 0.78]))
        for t in cbar.ax.get_yticklabels():
            t.set_fontsize(20)

        plt.savefig(fig_name)

if __name__ == "__main__":
    gp = graphics_plot()

    file_ = 'test1_aruco_board_width_0.2_space_0.1/GPS2Vision_aruco_pose_estimation.txt'

    #Plot estimated AruCo positions from test 
    gp.plot_aruco_pose_est('aruco_pose_estimation_error_x.png',file_,'y [m]', 'x [m]','Error in x (ArUco pose estimation) in meters',2)
    gp.plot_aruco_pose_est('aruco_pose_estimation_error_y.png',file_,'y [m]', 'x [m]','Error in y (ArUco pose estimation) in meters',3)
    gp.plot_aruco_pose_est('aruco_pose_estimation_error_z.png',file_,'y [m]', 'x [m]','Error in z (ArUco pose estimation) in meters',4)
    gp.plot_aruco_pose_est('aruco_pose_estimation_error_roll.png',file_,'y [m]', 'x [m]','Error in roll (ArUco pose estimation) in degress ',5)
    gp.plot_aruco_pose_est('aruco_pose_estimation_error_pitch.png',file_,'y [m]', 'x [m]','Error in pitch (ArUco pose estimation) in degress ',6)
    gp.plot_aruco_pose_est('aruco_pose_estimation_error_yaw.png',file_,'y [m]', 'x [m]','Error in yaw (ArUco pose estimation) in degress ',7)
