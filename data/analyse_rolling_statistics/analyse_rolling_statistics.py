import pandas as pd
import numpy as np

import matplotlib.pyplot as plt
from matplotlib import style
style.use('fivethirtyeight')


class calculate_rolling_average:

    def __init__(self):
        pass

    def plot_data(self,file_nam, index, label_x_fig1, label_x_fig2, label_y_fig1, label_y_fig2, title, fig_name):

        plt.rcParams.update({
        "text.usetex": True})
        
        fig = plt.figure(figsize=(25,25),facecolor='white') #21,15
        fig.set_facecolor((1,1,1)) 
        ax1 = plt.subplot2grid((2,1), (0,0))
        ax2 = plt.subplot2grid((2,1), (1,0), sharex=ax1)
        
        ax1.legend(loc='best',fontsize=60)
        df = pd.read_csv('test2/aruco_pose_estimation_rolling_average.txt', delimiter = " ")
        ts = pd.Series(df.iloc[:,index])

        data_mean = ts.rolling(200).mean()
        data_std = ts.rolling(200).std()

        ts.plot(ax=ax1, label='Measurements',fontsize=40)
        ts.rolling(5).mean().plot(ax=ax1,label='Mean',fontsize=80)
        ts.rolling(5).std().plot(ax=ax2, label='STD',fontsize=80)
        
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
        plt.savefig(fig_name,  facecolor=fig.get_facecolor())

if __name__=="__main__":

    cra = calculate_rolling_average()

    cra.plot_data('../aruco_pose_estimation_rolling_average.txt', 0, 'Iteration','Iteration','Mean','STD','Rolling average in x in meters',
            'Calculated_rolling_average_in_x_with_mean_and_STD.png')
    cra.plot_data('../aruco_pose_estimation_rolling_average.txt', 1, 'Iteration','Iteration','Mean','STD','Rolling average in y in meters',
            'Calculated_rolling_average_in_y_with_mean_and_STD.png')
    cra.plot_data('../aruco_pose_estimation_rolling_average.txt', 2, 'Iteration','Iteration','Mean','STD','Rolling average in z in meters',
            'Calculated_rolling_average_in_z_with_mean_and_STD.png')
    cra.plot_data('../aruco_pose_estimation_rolling_average.txt', 3, 'Iteration','Iteration','Mean','STD','Rolling average in roll in degress',
            'Calculated_rolling_average_in_roll_with_mean_and_STD.png')
    cra.plot_data('../aruco_pose_estimation_rolling_average.txt', 4, 'Iteration','Iteration','Mean','STD','Rolling average in pitch in degress',
            'Calculated_rolling_average_in_pitch_with_mean_and_STD.png')
    cra.plot_data('../aruco_pose_estimation_rolling_average.txt', 5, 'Iteration','Iteration','Mean','STD','Rolling average in yaw in degress',
            'Calculated_rolling_average_in_yaw_with_mean_and_STD.png')
