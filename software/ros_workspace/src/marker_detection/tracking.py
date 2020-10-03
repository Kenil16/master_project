#!/usr/bin/env python2

#from __future__ import print_function
import numpy as np
#import cv2
import matplotlib.pyplot as plt
from filterpy.kalman import KalmanFilter 
from filterpy.common import Q_discrete_white_noise
from scipy.linalg import block_diag
#from numpy.random import randn
from filterpy import*

class kalman_filter:

    def __init__(self, cycle_time):
        
        #Kalman filter 
        self.is_state_initialized = False

        self.R_std = 1.5#0.34
        self.Q_std = 0.04 #0.04

        self.tracker = KalmanFilter(dim_x=6,dim_z=3)
        self.dt = cycle_time #Five frames each second
        self.tracker.F = np.array([[1,0,0,self.dt,0,0],[0,1,0,0,self.dt,0],[0,0,1,0,0,self.dt],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]])
        
        self.tracker.R = np.eye(3) * self.R_std**2
        self.q = Q_discrete_white_noise(dim=3, dt=self.dt, var=self.Q_std**2)
        self.tracker.Q = block_diag(self.q, self.q)
        
        """
        self.tracker.Q[0,0] = q[0,0]
        self.tracker.Q[1,1] = q[0,0]
        self.tracker.Q[2,2] = q[1,1]
        self.tracker.Q[3,3] = q[1,1]
        self.tracker.Q[0,2] = q[0,1]
        self.tracker.Q[2,0] = q[0,1]
        self.tracker.Q[1,3] = q[0,1]
        self.tracker.Q[3,1] = q[0,1]
        """

        #Was set to np.array([[1, 0, 0, 0, 0, 0],[0, 1, 0, 0, 0, 0],[0, 0, 1, 0, 0, 0])
        self.tracker.H = np.array([[1, 0, 0, 0, 0, 0],[0, 1, 0, 0, 0, 0],[0, 0, 1, 0, 0, 0]])
        self.tracker.x = np.array([[0, 0, 0, 0, 0, 0]]).T
        self.tracker.P = np.eye(6) * pow(500,2)
        
    def first_update(self,data):
    
        #Init values for Kalman filtering
        self.is_state_initialized = True

        self.tracker.R = np.eye(3) * self.R_std**2
        self.tracker.P = np.eye(6) * 500.
        self.tracker.x = np.array([data[0], data[1], data[2], 0, 0, 0]).T
        
        self.tracker.predict()
        self.tracker.update(data)
    
    def get_measurement(self, data):

        #Initialize kalman filter
        if not self.is_state_initialized:
            self.first_update(data)
        else:
            self.tracker.predict()
            self.tracker.update(data)

if __name__ == "__main__":

        #This is only for testing of the Kalman filter
        kf = kalman_filter()
    
        pos, vel = (0, 0), (2, 0.5)
        sensor = kf.init_pos(pos, vel, noise_std=1)
        
        ps1 = np.array([kf.read() for _ in range(30)])
        pos, vel = (0, 0), (2, 0.5)
        sensor = kf.init_pos(pos, vel, noise_std=1)

        ps2 = np.array([kf.read() for _ in range(30)])
        
        pt0 = []
        pt0.append(ps1[0])
        pt0.append(ps2[0])

        kf.get_measurement(pt0)

        x1, y1, kfx1, kfy1 = [], [], [], []
        x2, y2, kfx2, kfy2 = [], [], [], []
        

        for z in range(29):

                points = []
                points.append(ps1[z+1])
                points.append(ps2[z+1])
            
                kf_trace = kf.get_measurement(points)
                
                kfx1.append(kf.x[0][0])
                kfy1.append(kf.x[0][1])
                x1.append(ps1[z][0])
                y1.append(ps1[z][1])

                kfx2.append(kf.x[1][0])
                kfy2.append(kf.x[1][1])
                x2.append(ps2[z][0])
                y2.append(ps2[z][1])


        fig, ax = plt.subplots()
        ax.scatter(x1, y1)
        ax.scatter(x2, y2)
        ax.plot(kfx1,kfy1)
        ax.plot(kfx2,kfy2)
        ax.set(xlabel='Position x [m]', ylabel='Position y [m]',title='2D Position tracking Kalman filter')
        ax.grid()
        fig.savefig("test.png")

