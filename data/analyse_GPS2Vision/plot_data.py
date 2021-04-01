import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
import pandas as pd
from matplotlib import style
style.use('fivethirtyeight')
from matplotlib.patches import Circle, Wedge, Polygon

class plot_data():
    
    def __init__(self):

        self.x_gps = []
        self.y_gps = []
        self.time_gps = []

        self.x_locate_board = []
        self.y_locate_board = []
        self.time_locate_board = []

        self.x_navigate_to_board = []
        self.y_navigate_to_board = []
        self.time_navigate_to_board = []
        
        self.x_gps2vision_transition = []
        self.y_gps2vision_transition  = []
        self.time_gps2vision_transition  = []
        
        self.data = []
        self.read_data('../gps2vision.txt')
    
    def read_data(self, file_name):
        init = True
        seq = 0
        with open(file_name,"r") as text_file:
            for line in text_file:
                items = line.split(' ')
                items[-1] = items[-1].strip()
                str_to_float = [float(item) for item in items]

                if str_to_float[-1] == 0:
                    self.x_gps.append(str_to_float[0])
                    self.y_gps.append(str_to_float[1])
                    self.time_gps.append(str_to_float[2])

                if str_to_float[-1] == 1:
                    self.x_locate_board.append(str_to_float[0])
                    self.y_locate_board.append(str_to_float[1])
                    self.time_locate_board.append(str_to_float[2])

                if str_to_float[-1] == 2:
                    self.x_navigate_to_board.append(str_to_float[0])
                    self.y_navigate_to_board.append(str_to_float[1])
                    self.time_navigate_to_board.append(str_to_float[2])
                
                if str_to_float[-1] == 3:
                    self.x_gps2vision_transition.append(str_to_float[0])
                    self.y_gps2vision_transition.append(str_to_float[1])
                    self.time_gps2vision_transition.append(str_to_float[2])

    def plot_data(self, index, label_x_fig1, label_x_fig2, label_y_fig1, label_y_fig2, title, labels, fig_name):

        plt.rcParams.update({
        "text.usetex": True})
        
        fig = plt.figure(figsize=(25,25),facecolor='white') #21,15
        fig.set_facecolor((1,1,1)) 
        ax1 = plt.subplot2grid((2,1), (0,0))
        ax2 = plt.subplot2grid((2,1), (1,0), sharex=ax1)
        
        ax1.legend(loc='best',fontsize=60)
        
        df = pd.read_csv('../hold_pose_using_aruco_pose_estimation.txt', delimiter=" ")
        aruco = pd.Series(index[0], index=self.time)
        setpoint = pd.Series(index[1], index=self.time)
        ground_truth = pd.Series(index[2], index=self.time)
        
        aruco.plot(ax=ax1, label=labels[0][0],fontsize=40,color='b')
        ground_truth.plot(ax=ax1,label=labels[2][0],fontsize=80,linestyle='dotted',color='r')
        setpoint.plot(ax=ax1,label=labels[1][0], fontsize=80, linestyle=':', color='g')

        error_pose_aruco = aruco - ground_truth
        error_aruco_setpoint = aruco - setpoint
        
        error_pose_aruco.plot(ax=ax2, label='Pose estimation error',fontsize=80)
        error_aruco_setpoint.plot(ax=ax2, label='Setpoint error',fontsize=80)
        
        ax1.legend(loc='best',fontsize=60)
        ax1.set_title(title,fontsize=70)
        ax1.set_xlabel(label_x_fig1,fontsize=80)
        ax1.set_ylabel(label_y_fig1,fontsize=80)
        ax2.set_xlabel(label_x_fig2,fontsize=80)
        ax2.set_ylabel(label_y_fig2,fontsize=80)
        ax2.legend(loc='best',fontsize=60)
        
        ax1.set_facecolor((1,1,1))
        ax2.set_facecolor((1,1,1))
        
        plt.tight_layout()
        plt.subplots_adjust(bottom=0.1, top=0.93, hspace=0.2, wspace=0)
        plt.savefig(fig_name, facecolor=fig.get_facecolor())

    def plot_data_2d(self):
        
        plt.rcParams.update({
        "text.usetex": True})
        
        gps = pd.DataFrame.from_dict({'x' : self.x_gps, 'y' : self.y_gps})
        locate_board = pd.DataFrame.from_dict({'x' : self.x_locate_board, 'y' : self.y_locate_board})
        navigate_to_board = pd.DataFrame.from_dict({'x' : self.x_navigate_to_board, 'y' : self.y_navigate_to_board})
        gps2vision_transition = pd.DataFrame.from_dict({'x' : self.x_gps2vision_transition, 'y' : self.y_gps2vision_transition})

        states = [gps, locate_board, navigate_to_board, gps2vision_transition]

        index = 0
        colors = ['b','r','y','g']
        legends = ['GPS', 'Locating board (GPS)', 'Navigate to board (GPS)', 'GPS2Vision transition (vision)']
        fig, ax = plt.subplots(figsize=(15,25),facecolor='white')
        ax.set_xlim([self.x_gps[0]-7, self.x_gps[0]+7])
        ax.set_ylim([self.y_gps[0]-5, self.y_gps[0]+22])

        for state in states:

            x = state['x'].values
            y = state['y'].values

            ax.plot(x,y, linewidth=3, marker="o",color=colors[index],label=legends[index])
            if len(x) and len(y): 
                u = np.diff([x[len(x)/2], x[len(y)/2+1]])
                v = np.diff([y[len(y)/2], y[len(y)/2+1]])
                pos_x = x[len(x)/2-1] + u/2
                pos_y = y[len(y)/2-1] + v/2
                norm = np.sqrt(u**2+v**2)
                ax.quiver(pos_x, pos_y, u/norm, v/norm, angles="xy",zorder=5, pivot="mid")

            index += 1
        
        
        x = []
        y = []
        for route_x in [self.x_gps, self.x_locate_board, self.x_navigate_to_board, self.x_gps2vision_transition]:
            for x_ in route_x:
                x.append(x_)

        for route_y in [self.y_gps, self.y_locate_board, self.y_navigate_to_board, self.y_gps2vision_transition]:
            for y_ in route_y:
                y.append(y_)
        
        ax.plot(x,y,linewidth=0.8, linestyle='--', color='k')
        
        ax.plot(self.x_gps[0],self.y_gps[0], marker='o', linestyle='--', color='m', markersize=13)
        
        wedge = Wedge((3.65, 2.99), 5, 180, 360, alpha=0.05, label="Safe GP2Vision transition area")
        
        ax.annotate('Start from GPS', xy=(3,0.5), xytext=(2.6,-18), size=20,bbox=dict(boxstyle="round, pad=0.3",fc="white", ec="k", lw=1))
        ax.annotate('GPS2Vision board', xy=(3,0.5), xytext=(2.65, 2.5), size=20,bbox=dict(boxstyle="round, pad=0.3",fc="white", ec="k", lw=1))
        
        circle1 = plt.Circle((3.65, -6),5, alpha=0.05, edgecolor='b', color='black', label='Target GPS location with uncertainty')
        ax.add_patch(circle1)

        ax.add_patch(wedge)
        plt.tick_params(axis='x', labelsize=40)
        plt.tick_params(axis='y', labelsize=40)
        ax.legend(loc='best',fontsize=20)
        ax.set_title('GPS to vision based navigation',fontsize=40)
        ax.set_xlabel('x [m]',fontsize=30)
        ax.set_ylabel('y [m]',fontsize=30)
        plt.tight_layout()
        plt.savefig('test.png')

if __name__ == "__main__":

    tt = plot_data()
    tt.plot_data_2d()
