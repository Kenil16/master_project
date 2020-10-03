#!/usr/bin/python

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

