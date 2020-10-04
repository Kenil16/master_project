#!/usr/bin/env python2

import numpy as np
from filterpy.kalman import KalmanFilter 
from filterpy.common import Q_discrete_white_noise
from scipy.linalg import block_diag
from filterpy import*

#Inspiration from https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/14-Adaptive-Filtering.ipynb
class kalman_filter:

    def __init__(self, cycle_time):
        
        #Kalman filter 
        self.is_state_initialized = False

        self.tracker = KalmanFilter(dim_x=3, dim_z=1)
        self.dt = cycle_time #Five frames each second
        self.tracker.x = np.array([0., 0., 0.]) #State mean
        self.tracker.P *= np.eye(3) * 0.5 #Variance of the state 0.01
        self.tracker.R *= 1 #Measurement noise 100
        self.tracker.Q = Q_discrete_white_noise(dim=3, dt=self.dt, var=0.01) #Process noise 0.01
        self.tracker.F = np.array([[1, self.dt, 0.5*self.dt*self.dt],[0, 1, self.dt],[0, 0, 1]]) #State transition function
        self.tracker.H = np.array([[1., 0, 0]]) #Measurement function
        
        #Adaptive filtering
        self.Q_scale_factor = 1000.
        self.eps_max = 4.
        self.count = 0

    def first_update(self,data):
    
        #Init values for Kalman filtering
        self.is_state_initialized = True
        
        self.tracker.x = np.array([data, 0., 0.])
        #self.tracker.P = np.eye(3) * 3
        
        self.tracker.predict()
        self.tracker.update(data)
    
    def get_measurement(self, data):

        #Initialize kalman filter
        if not self.is_state_initialized:
            self.first_update(data)
        else:
            self.tracker.predict()
            self.tracker.update(data)
            
            #Adaptive filtering
            y, S = self.tracker.y, self.tracker.S
            eps = np.dot(y.T, np.linalg.inv(S)).dot(y)
            if eps > self.eps_max:
                self.tracker.Q *= self.Q_scale_factor
                self.count += 1
            elif self.count > 0:
                self.tracker.Q /= self.Q_scale_factor
                self.count -= 1

if __name__ == "__main__":

        #This is only for testing of the Kalman filter
        kf = kalman_filter()
