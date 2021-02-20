from ukf import UKF
import csv
import numpy as np
import math
import matplotlib.pyplot as plt



class ukf():
    
    def __init__(self):

        self.corr_acc_x = []
        self.corr_acc_y = []
        self.corr_acc_z = []
        self.data = []

        self.read_data('sensor_fusion_data.txt')
        self.mx = np.array([item[0] for item in self.data])
        self.my = np.array([item[1] for item in self.data])
        self.mz = np.array([item[2] for item in self.data])

        self.mukf_x = np.array([item[3] for item in self.data])
        self.mukf_y = np.array([item[4] for item in self.data])
        self.mukf_z = np.array([item[5] for item in self.data])

        self.macc_x = np.array([item[6] for item in self.data])
        self.macc_y = np.array([item[7] for item in self.data])
        self.macc_z = np.array([item[8] for item in self.data])

        self.mpsi = np.array([item[9] for item in self.data])
        self.mphi = np.array([item[10] for item in self.data])
        self.mtheta = np.array([item[11] for item in self.data])

        self.mgyro_x = np.array([item[12] for item in self.data])
        self.mgyro_y = np.array([item[13] for item in self.data])
        self.mgyro_z = np.array([item[14] for item in self.data])

        self.mtime = np.array([item[15] for item in self.data])
        self.old_x = 0.0

    def read_data(self, file_name):

        with open(file_name,"r") as text_file:
            for line in text_file:
                items = line.split(' ')
                items[-1] = items[-1].strip()
                str_to_float = [float(item) for item in items]
                if not str_to_float[0] == 0:
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

        plt.scatter(self.mx, self.my, s=10, label='GPS Measurements')
        plt.scatter(self.mukf_x, self.mukf_y, s=2, label='EKF Position', c='k')

        # Start/Goal
        plt.scatter(self.mx[0], self.my[0], s=60, label='Start', c='g')
        plt.scatter(self.mx[-1], self.my[-1], s=60, label='Goal', c='r')

        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.title('Position')
        plt.legend(loc='best')
        plt.axis('equal')

        plt.savefig('Position.png')

    def plot_ground_truth(self, x, y):

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

        plt.scatter(x, y, s=5, label='GPS Measurements', c='r')

        # Start/Goal
        plt.scatter(x[0], y[0], s=60, label='Start', c='g')
        plt.scatter(x[-1], y[-1], s=60, label='Goal', c='r')

        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.title('position_ground_truth')
        plt.legend(loc='best')
        plt.axis('equal')

        plt.savefig('position_ground_truth.png')

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
    ukf.plot_position()

