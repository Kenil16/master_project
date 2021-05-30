import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
import pandas as pd
from matplotlib import style
style.use('fivethirtyeight')
from matplotlib.patches import Circle, Wedge, Polygon
import glob

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

        self.success = 0
        self.gps_time = []
        self.locate_board_time = []
        self.navigate_to_board_time = []
        self.gps2vision_time = []
        
    def plot_data(self, path):
        
        path = path + '*.txt' #Read all txt files in tests directory
        files = glob.glob(path)

        plt.rcParams.update({
        "text.usetex": True})

        fig, ax = plt.subplots(figsize=(15,25),facecolor='white')
        colors = ['b','r','y','g']
        legends = ['GPS navigation', 'Locating board (GPS) navigation', 'Navigate to board (GPS) navigation', 'GPS2Vision transition (vision) navigation']
        index_files = 0
        num_of_files = len(files)
        index_files = num_of_files
        for fle in files:
            
            #Reset locations 
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
            
            #Find which lines belongs to each state 
            with open(fle,"r") as text_file:
                for line in text_file:
                    items = line.split(' ')
                    items[-1] = items[-1].strip()
                    str_to_float = [float(item) for item in items]
                    
                    #These defines which substate the uav is in 0(gps), 1(locate board), 2(navigate to board) and 3(gps2vision)
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

            #See if last locations is GOAL
            if len(self.x_gps2vision_transition):
                if (self.x_gps2vision_transition[-1] < 4 and self.x_gps2vision_transition > 2.5) and (self.y_gps2vision_transition[-1] < 2 and self.y_gps2vision_transition > 0):
                    self.success += 1

            #Find travel time
            time = 0.0
            for step in range(1,len(self.time_gps)):
                t = self.time_gps[step] - self.time_gps[step-1]
                if t < 1:
                    time += t
            self.gps_time.append(time)

            time = 0.0
            for step in range(1,len(self.time_locate_board)):
                t = self.time_locate_board[step] - self.time_locate_board[step-1]
                print(t)
                if t < 1:
                    time += t
            self.locate_board_time.append(time)
            
            time = 0.0
            for step in range(1,len(self.time_navigate_to_board)):
                t = self.time_navigate_to_board[step] - self.time_navigate_to_board[step-1]
                if t < 1:
                    time += t
            self.navigate_to_board_time.append(time)
            
            time = 0.0
            for step in range(1,len(self.time_gps2vision_transition)):
                t = self.time_gps2vision_transition[step] - self.time_gps2vision_transition[step-1]
                if t < 1:
                    time += t
            self.gps2vision_time.append(time)
            
            #Defines locations for states 
            gps = pd.DataFrame.from_dict({'x' : self.x_gps, 'y' : self.y_gps})
            locate_board = pd.DataFrame.from_dict({'x' : self.x_locate_board, 'y' : self.y_locate_board})
            navigate_to_board = pd.DataFrame.from_dict({'x' : self.x_navigate_to_board, 'y' : self.y_navigate_to_board})
            gps2vision_transition = pd.DataFrame.from_dict({'x' : self.x_gps2vision_transition, 'y' : self.y_gps2vision_transition})
            
            #Make state locations as a list
            states = [gps, locate_board, navigate_to_board, gps2vision_transition]

            index = 0
            if num_of_files == index_files:
                ax.set_xlim([self.x_gps[0]-7.5, self.x_gps[0]+7.5])
                ax.set_ylim([self.y_gps[0]-5, self.y_gps[0]+22])

            for state in states:

                x = state['x'].values
                y = state['y'].values
                
                if  not (index_files - 1):
                    ax.plot(x,y, linewidth=3, marker="o",color=colors[index],label=legends[index])
                else:
                    ax.plot(x,y, linewidth=3, marker="o",color=colors[index])
                
                if not index and len(x) and len(y): 
                    u = np.diff([x[-2], x[-1]])
                    v = np.diff([y[-2], y[-1]])
                    pos_x = x[-1] + u/2
                    pos_y = y[-1] + v/2
                    norm = np.sqrt(u**2+v**2)
                    ax.quiver(pos_x, pos_y, u/norm, v/norm, angles="xy",zorder=5, pivot="mid")

                index += 1
            index_files -= 1
            
            
            #Show each route from GPS to vision as dotted lines 
            x = []
            y = []
            for route_x in [self.x_gps, self.x_locate_board, self.x_navigate_to_board, self.x_gps2vision_transition]:
                for x_ in route_x:
                    x.append(x_)
            for route_y in [self.y_gps, self.y_locate_board, self.y_navigate_to_board, self.y_gps2vision_transition]:
                for y_ in route_y:
                    y.append(y_)
            ax.plot(x,y,linewidth=0.8, linestyle='--', color='k')
            
        #Show text for start and GPS2Vision board locations 
        ax.annotate('Start from GPS', xy=(3,0.5), xytext=(2.55,-17.7), size=25,bbox=dict(boxstyle="round, pad=0.3",fc="white", ec="k", lw=1))
        ax.annotate('GPS2Vision board', xy=(3,0.5), xytext=(2.35, 2.5), size=25,bbox=dict(boxstyle="round, pad=0.3",fc="white", ec="k", lw=1))
        
        #Show start location
        #ax.plot(self.x_gps[0],self.y_gps[0], marker='o', linestyle='--', color='m', markersize=13)
        
        #Show safe area to change from GPS to vision
        wedge = Wedge((3.65, 2.99), 5, 180, 360, alpha=0.05, label="Safe GP2Vision transition area")
        
        #GPS uncertainty
        circle1 = plt.Circle((3.65, -7.5),6, alpha=0.05, edgecolor='b', color='black', label='Target GPS location with uncertainty')
        ax.add_patch(circle1)
        
        #Safe GPS2Vision transitiom
        ax.add_patch(wedge)

        #Plot parameters 
        plt.tick_params(axis='x', labelsize=40)
        plt.tick_params(axis='y', labelsize=40)
        ax.legend(loc='best',fontsize=25)
        ax.set_title('GPS to vision based navigation',fontsize=40)
        ax.set_xlabel('x [m]',fontsize=40)
        ax.set_ylabel('y [m]',fontsize=40)
        ax.set_facecolor((1,1,1))
        plt.tight_layout()
        plt.savefig('gps2vision.png', facecolor=fig.get_facecolor())

        print(self.navigate_to_board_time)

        print("Runs: " + str(self.success))
        print("GPS travel time: " + "{:.4f}".format(sum(self.gps_time)/len(self.gps_time)))
        print("Locate board travel time: " + "{:.4f}".format(sum(self.locate_board_time)/len(self.locate_board_time)))
        print("Navigate to board travel time: " + "{:.4f}".format(sum(self.navigate_to_board_time)/len(self.navigate_to_board_time)))
        print("GPS2Vision travel time: " + "{:.4f}".format(sum(self.gps2vision_time)/len(self.gps2vision_time)))

if __name__ == "__main__":

    tt = plot_data()
    #tt.plot_data('test4_5-7ms_wind_20_runs/')
    #tt.plot_data('test2_noWind_20_runs/')
    tt.plot_data('test5_7-10ms_wind_20_runs/')
