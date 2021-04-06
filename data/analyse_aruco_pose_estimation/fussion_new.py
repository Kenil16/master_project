import csv
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import


class ukf():
    
    def __init__(self):

        self.data = []

        self.read_data('../aruco_pose_estimation.txt')
        self.mx = np.array([item[0] for item in self.data])
        self.my = np.array([item[1] for item in self.data])
        self.mz = np.array([item[2] for item in self.data])
        
        self.mpsi = np.array([item[3] for item in self.data])
        self.mphi = np.array([item[4] for item in self.data])
        self.mtheta = np.array([item[5] for item in self.data])
        
        self.g_x = np.array([item[6] for item in self.data])
        self.g_y = np.array([item[7] for item in self.data])
        self.g_z = np.array([item[8] for item in self.data])
        
        self.g_roll = np.array([item[9] for item in self.data])
        self.g_pitch = np.array([item[10] for item in self.data])
        self.g_yaw = np.array([item[11] for item in self.data])
        
        self.mtime = np.array([item[12] for item in self.data])
    
    def read_data(self, file_name):
        init = True
        seq = 0
        with open(file_name,"r") as text_file:
            for line in text_file:
                items = line.split(' ')
                items[-1] = items[-1].strip()
                str_to_float = [float(item) for item in items]
                self.data.append(str_to_float)


    def plot_state(self, title, x_label, y_label, fig_name, x, y, g_y, labels, g_labels):

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

    def create_ground_truth(self):
        
        route = [[-3.65,-1.10,1.5], [-3.65,-4.25,1.5], [-3.65,-4.25,1.5], [-7.1,-4.25,1.5], [-7.1,-4.25,1.5], [-7.1,-7.1,1.5], [-7.1,-7.1, 1.0,]]
        waypoints = 335
        
        int_points = waypoints/(len(route)-1)
        if int(int_points) < int_points:
            int_points = int_points + 1
        
        #print(int_points)
        
        for i in range(1, len(route)):
            point1 = route[i-1]
            point2 = route[i]
    
            new_points = np.linspace(point1[0],point2[0],int(int_points))
            for i in new_points:    
                self.new_route_x.append(i)
        
            new_points = np.linspace(point1[1],point2[1],int(int_points))
            for i in new_points:    
                self.new_route_y.append(i)
            
            new_points = np.linspace(point1[2],point2[2],int(int_points))
            for i in new_points:    
                self.new_route_z.append(i)
        
        while len(self.new_route_x) > waypoints:
            self.new_route_x.pop()
            self.new_route_y.pop()
            self.new_route_z.pop()
        
if __name__ == "__main__":

    ukf = ukf()
    labels = ['x','y','z']
    g_labels = ['g_x', 'g_y', 'g_z']
    ys = [ukf.mx, ukf.my, ukf.mz]
    g_ys = [ukf.g_x, ukf.g_y, ukf.g_z]
    ukf.plot_state('Position', 'Time [s]', r'Position [$m$]', 'position.png', ukf.mtime, ys, g_ys, labels, g_labels)
    
    """
    labels = ['Roll','Pitch', 'g_roll', 'g_pitch']
    ys = [ukf.mpsi, ukf.mphi, ukf.g_roll, ukf.g_pitch]
    ukf.plot_state('Angle [Ground truth]', 'Time [s]', r'Angle [Degress]', 'orientation_ground_truth.png', ukf.mtime, ys)
    """
    labels = ['Roll','Pitch','Yaw', 'g_roll', 'g_pitch', 'g_yaw']
    ys = [ukf.mpsi, ukf.mphi, ukf.mtheta]
    g_ys = [ukf.g_roll, ukf.g_pitch,ukf.g_yaw]
    g_labels = ['g_roll', 'g_pitch', 'g_yaw']
    
    ukf.plot_state('Angle [Ground truth]', 'Time [s]', r'Angle [Degress]', 'orientation_ground_truth.png', ukf.mtime, ys, g_ys, labels, g_labels)
