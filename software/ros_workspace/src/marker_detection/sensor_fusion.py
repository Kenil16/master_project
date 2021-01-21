#!/usr/bin/env python2

import numpy as np
from filterpy.kalman import KalmanFilter 
from filterpy.common import Q_discrete_white_noise
from scipy.linalg import block_diag
from filterpy import*
from timeit import default_timer as timer
from pathlib import Path

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import* #quaternion_from_euler, euler_from_quaternion, quaternion_matrix,quaternion_from_matrix

#Inspiration from https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/14-Adaptive-Filtering.ipynb
class kalman_filter:

    def __init__(self):
        
        #Init ROS node
        rospy.init_node('sensor_fusion')
        
        self.imu_data = Imu()
        self.aruco_marker_pose = PoseStamped()
        self.sensor_fusion_pose = PoseStamped()

        #IMU update rate 50Hz (rostopic hz /mavros/imu/data)
        self.dt = 1/30.

        #Subscribers
        rospy.Subscriber('/mavros/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/onboard/aruco_marker_pose', PoseStamped, self.aruco_marker_pose_callback)
        
        #Publishers
        self.sensor_fusion_pose_pub = rospy.Publisher('/onboard/sensor_fusion',PoseStamped, queue_size=1)
        
        #Kalman filter 
        self.is_state_initialized = False
        
        self.tracker = KalmanFilter(dim_x=9, dim_z=3)
        
        """
        #State transition matrix
        self.tracker.F = np.array([[1, self.dt, 0.5*self.dt*self.dt, 0, 0, 0, 0, 0, 0],
                                   [0, 1, self.dt, 0, 0, 0, 0, 0 ,0],
                                   [0, 0, 1, 0, 0, 0, 0, 0, 0],
                                   [0, 0, 0, 1, self.dt, 0.5*self.dt*self.dt, 0, 0, 0],
                                   [0, 0, 0, 0, 1, self.dt, 0, 0, 0],
                                   [0, 0, 0, 0, 0, 1, 0, 0, 0],
                                   [0, 0, 0, 0, 0, 0, 1, self.dt, 0.5*self.dt*self.dt],
                                   [0, 0, 0, 0, 0, 0, 0, 1, self.dt],
                                   [0, 0, 0, 0, 0, 0, 0, 0, 1]])
        
        #Measurement function 
        self.tracker.H = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0],
                                   [0, 0, 0, 1, 0, 0, 0, 0, 0],
                                   [0, 0, 0, 0, 0, 0, 1, 0, 0]])

        
        #Measurement noise
        self.tracker.R = np.array([[15.0, 0, 0],
                                   [0, 15.0, 0],
                                   [0, 0, 0.01]])

        #Process noise
        x = Q_discrete_white_noise(dim=3, dt=self.dt, var=10.0)
        y = Q_discrete_white_noise(dim=3, dt=self.dt, var=10.0)
        z = Q_discrete_white_noise(dim=3, dt=self.dt, var=0.1)
        self.tracker.Q = block_diag(x, y, z)
        
        #State mean
        self.tracker.x = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0]]).T
        
        #Variance of the state
        self.tracker.P = np.eye(9) * 500.

        #Adaptive filtering
        self.Q_scale_factor = 10000.
        self.eps_max = 4.
        self.count = 0
        """
        
        #State transition matrix
        self.tracker.F = np.array([[1, self.dt, 0.5*self.dt*self.dt, 0, 0, 0, 0, 0, 0],
                                   [0, 1, self.dt, 0, 0, 0, 0, 0 ,0],
                                   [0, 0, 1, 0, 0, 0, 0, 0, 0],
                                   [0, 0, 0, 1, self.dt, 0.5*self.dt*self.dt, 0, 0, 0],
                                   [0, 0, 0, 0, 1, self.dt, 0, 0, 0],
                                   [0, 0, 0, 0, 0, 1, 0, 0, 0],
                                   [0, 0, 0, 0, 0, 0, 1, self.dt, 0.5*self.dt*self.dt],
                                   [0, 0, 0, 0, 0, 0, 0, 1, self.dt],
                                   [0, 0, 0, 0, 0, 0, 0, 0, 1]])
        
        #Measurement function 
        self.tracker.H = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0],
                                   [0, 0, 0, 1, 0, 0, 0, 0, 0],
                                   [0, 0, 0, 0, 0, 0, 1, 0, 0]])

        
        #Measurement noise
        self.tracker.R = np.array([[15.0, 0, 0],
                                   [0, 15.0, 0],
                                   [0, 0, 0.01]])

        #Process noise
        x = Q_discrete_white_noise(dim=3, dt=self.dt, var=10.0)
        y = Q_discrete_white_noise(dim=3, dt=self.dt, var=10.0)
        z = Q_discrete_white_noise(dim=3, dt=self.dt, var=0.1)
        self.tracker.Q = block_diag(x, y, z)
        
        #State mean
        self.tracker.x = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0]]).T
        
        #Variance of the state
        self.tracker.P = np.eye(9) * 500.

        #Offset IMU
        self.acc_x = 0.0
        self.acc_y = 0.0
        self.acc_z = 0.0
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0

        #Init data textfile
        data = Path('../../../../data/estimate_imu_noise/data.txt')
        self.imu_data_path = '../../../../data/estimate_imu_noise/data.txt'
        if not data.is_file:
            data = open(self.imu_data_path,'r+')
            data.truncate(0)
            data.close
        else:
            data = open(self.imu_data_path,'w+')
            data.close()

        rospy.Timer(rospy.Duration(self.dt), self.timer_callback)
        rospy.spin()

    def aruco_marker_pose_callback(self, data):
        self.aruco_marker_pose = data

    def imu_callback(self, data):
        self.imu_data = data

    def first_update(self,data):
    
        #Init values for Kalman filtering
        self.is_state_initialized = True
        
        self.tracker.x = np.array([data[0], 0., 0., data[1], 0., 0., data[2], 0., 0.])
        self.tracker.predict()
        self.tracker.update(data)
    
    def get_measurement(self, data):

        #Initialize kalman filter
        if not self.is_state_initialized:
            self.first_update(data)
        else:
            self.tracker.predict()
            self.tracker.update(data)
            
            """
            #Adaptive filtering
            y, S = self.tracker.y, self.tracker.S
            eps = np.dot(y.T, np.linalg.inv(S)).dot(y)
            if eps > self.eps_max:
                self.tracker.Q *= self.Q_scale_factor
                self.count += 1
            elif self.count > 0:
                self.tracker.Q /= self.Q_scale_factor
                self.count -= 1
            """

    def write_imu_data(self, acc_x=0, acc_y=0, acc_z=0, gyro_x=0, gyro_y=0, gyro_z=0, time=0):

        data = open(self.imu_data_path,'a')
        data.write(str(acc_x) + " " + str(acc_y) + " " + str(acc_z) + " " + str(gyro_x) + " " + str(gyro_y) + " " + str(gyro_z) + " " + str(time))
        data.write('\n')
        data.close()

    def timer_callback(self,event):
        
        q = (self.aruco_marker_pose.pose.orientation.x,
             self.aruco_marker_pose.pose.orientation.y,
             self.aruco_marker_pose.pose.orientation.z,
             self.aruco_marker_pose.pose.orientation.w)
        euler = euler_from_quaternion(q)
        acc_x = np.cos(euler[2])*self.imu_data.linear_acceleration.x - np.sin(euler[2])*self.imu_data.linear_acceleration.y
        acc_y = np.sin(euler[2])*self.imu_data.linear_acceleration.x + np.cos(euler[2])*self.imu_data.linear_acceleration.y
        acc_z = self.imu_data.linear_acceleration.z
        time = self.imu_data.header.stamp.secs + self.imu_data.header.stamp.nsecs/1000000000.

        if time > 10 and time < 20:
            self.write_imu_data(acc_x, acc_y, acc_z, time=time)

if __name__ == "__main__":

        #This is only for testing of the Kalman filter
        kf = kalman_filter()
