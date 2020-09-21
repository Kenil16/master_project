from __future__ import print_function
import numpy as np
import cv2
import matplotlib.pyplot as plt
from filterpy.kalman import KalmanFilter 
from filterpy.common import Q_discrete_white_noise
from numpy.random import randn

class kalman_filter:

    def __init__(self):
        
        #Kalman filter 
        self.is_state_initialized = False

        self.R_std = 0.34#30 #0.35
        self.Q_std = 0.04 #0.04

        self.tracker = KalmanFilter(dim_x=4,dim_z=2)
        self.dt = 0.05 #Five frames each second
        self.tracker.F = np.array([[1,0,self.dt,0],[0,1,0,self.dt],[0,0,1,0],[0,0,0,1]])
        
        self.tracker.R = np.eye(2) * self.R_std**2
        q = Q_discrete_white_noise(dim=2, dt=self.dt, var=self.Q_std**2)
        self.tracker.Q[0,0] = q[0,0]
        self.tracker.Q[1,1] = q[0,0]
        self.tracker.Q[2,2] = q[1,1]
        self.tracker.Q[3,3] = q[1,1]
        self.tracker.Q[0,2] = q[0,1]
        self.tracker.Q[2,0] = q[0,1]
        self.tracker.Q[1,3] = q[0,1]
        self.tracker.Q[3,1] = q[0,1]
        
        self.tracker.H = np.array([[1, 0, 0, 0],[0, 1, 0, 0]])
        #self.tracker.R = np.array([[50000., 0],[0 , 5.]])
        self.tracker.x = np.array([[0, 0, 0, 0]]).T
        self.tracker.P = np.eye(4) * pow(500,2)
        
        #Lists with values for found objetcs to track
        self.R = []
        self.P = []
        self.x = []
        
        self.points = []

    def FirstOrderKF(self,R, Q, dt):

        kf = KalmanFilter(dim_x=2, dim_z=1)
        kf.x = np.eye(2)
        kf.P *= np.array([[500, 0], [0, 500]])
        kf.R *= R
        kf.Q = Q_discrete_white_noise(2, dt, Q)
        kf.F = np.array([[1., dt],
                     [0., 1]])
        kf.H = np.array([[1., 0]])
        return kf

    def first_update(self,data):

        for i in range(len(data)):
            
            #Init values for Kalman filtering
            self.tracker.R = np.eye(2) * self.R_std**2
            self.tracker.P = np.eye(4) * 500.
            self.tracker.x = np.array([[data[i][0], data[i][1], 0, 0]]).T

            self.tracker.predict()
            self.tracker.update(data[i])

            self.R.append(self.tracker.R)
            self.P.append(self.tracker.P)
            self.x.append(self.tracker.x)

        
    def get_measurement(self, data):

        #Number of new found objects to be initialized on the Kalman filter list
        new_points = 0
        #List with updated values for x and y
        list_points = []

        #Initialize kalman filter for new points if any 
        if len(data) > len(self.x):
            new_points = len(data) - len(self.x)
            init_list = data[len(self.x):len(data)]
            self.first_update(init_list)

        #Now update the kalman filter of old points
        for i in range(len(self.x)-new_points):
            
            #Load values for that specific object
            self.tracker.R = self.R[i]
            self.tracker.P = self.P[i]
            self.tracker.x = self.x[i]
        
            self.tracker.predict()

            #Only update if the point can be found on the list.
            #(0,0) means no new update to that objetc  
            if (data[i][0] != 0) and (data[i][1] != 0):
                self.tracker.update(data[i])

            #Save calues for that specific objetc
            self.R[i] = self.tracker.R
            self.P[i] = self.tracker.P
            self.x[i] = self.tracker.x

            #Put new values on a list
            list_points.append([self.x[i][0],self.x[i][1]])

        self.points = list_points

    #The following two funcitions are only for testing purposes 
    def init_pos(self, pos=(0, 0), vel=(0, 0), noise_std=1.):
        self.vel = vel
        self.noise_std = noise_std
        self.pos = [pos[0], pos[1]]
        
    def read(self):
        self.pos[0] += self.vel[0]
        self.pos[1] += self.vel[1]
        
        return [self.pos[0] + randn() * self.noise_std,
                self.pos[1] + randn() * self.noise_std]

class optical_flow:
    
    def __init__(self):
        
        #Optical flow
        self.lk_params = dict( winSize  = (15, 15),maxLevel = 2,criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
        self.feature_params = dict( maxCorners = 500,qualityLevel = 0.3,minDistance = 7,blockSize = 7 )
        
        self.track_len = 10
        self.detect_interval = 5
        self.tracks = []
        self.points = []
        self.frame_idx = 0
        self.prev_gray = None
        
        
    def track_objects(self, frame):
       
        # https://github.com/opencv/opencv/blob/master/samples/python/lk_track.py
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        vis = frame.copy()
        new_tracks = []
        
        if len(self.tracks) > 0:
            img0, img1 = self.prev_gray, frame_gray
            p0 = np.float32([tr[-1] for tr in self.tracks]).reshape(-1, 1, 2)
            p1, _st, _err = cv2.calcOpticalFlowPyrLK(img0, img1, p0, None, **self.lk_params)
            p0r, _st, _err = cv2.calcOpticalFlowPyrLK(img1, img0, p1, None, **self.lk_params)
            d = abs(p0-p0r).reshape(-1, 2).max(-1)
            
            good = d < 1
            
            for tr, (x, y), good_flag in zip(self.tracks, p1.reshape(-1, 2), good):
                if not good_flag:
                    continue
                tr.append((x, y))
                if len(tr) > self.track_len:
                    del tr[0]
                new_tracks.append(tr)
                cv2.circle(vis, (x, y), 2, (0, 255, 0), -1)
                
            self.tracks = new_tracks
            cv2.polylines(vis, [np.int32(tr) for tr in self.tracks], False, (0, 255, 0))
            #cv2.putText(vis,  'track count: %d' % len(self.tracks), (20,20), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2, cv2.LINE_AA)
            #draw_str(vis, (20, 20), 'track count: %d' % len(self.tracks))
            
        if self.frame_idx % self.detect_interval == 0:
            mask = np.zeros_like(frame_gray)
            mask[:] = 255
            for x, y in [np.int32(tr[-1]) for tr in self.tracks]:
                cv2.circle(mask, (x, y), 5, 0, -1)
            p = cv2.goodFeaturesToTrack(frame_gray, mask = mask, **self.feature_params)
            if p is not None:
                for x, y in np.float32(p).reshape(-1, 2):
                    self.tracks.append([(x, y)])
                    self.points.append([x,y])
        
        self.frame_idx += 1
        self.prev_gray = frame_gray
        
        return vis
    
if __name__ == "__main__":

        #This is only for testing of the Kalman filter
        kf = kalman_filter()
        op = optical_flow()
    
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

