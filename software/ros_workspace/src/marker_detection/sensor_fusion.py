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
from tf.transformations import* 
from std_msgs.msg import (String, Int8, Float64, Bool)

from scipy.stats import norm
#from sympy import Symbol, symbols, Matrix, sin, cos

#Inspiration from https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/14-Adaptive-Filtering.ipynb
class kalman_filter:

    def __init__(self, dt):
        
        #Kalman filter 
        self.is_state_initialized = False
        self.dt = dt
        """
        self.tracker = KalmanFilter(dim_x=9, dim_z=3)
        
        
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
        
        self.tracker = KalmanFilter(dim_x=18, dim_z=9)
        
        #State transition matrix
        self.tracker.F = np.array([[1, self.dt, 0.5*self.dt*self.dt, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                   [0, 1, self.dt, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                   [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                   [0, 0, 0, 1, self.dt, 0.5*self.dt*self.dt, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                   [0, 0, 0, 0, 1, self.dt, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                   [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                   [0, 0, 0, 0, 0, 0, 1, self.dt, 0.5*self.dt*self.dt, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                   [0, 0, 0, 0, 0, 0, 0, 1, self.dt, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                   [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                   [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, self.dt, 0.5*self.dt*self.dt, 0, 0, 0, 0, 0, 0],
                                   [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, self.dt, 0, 0, 0, 0, 0, 0],
                                   [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                                   [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, self.dt, 0.5*self.dt*self.dt, 0, 0, 0],
                                   [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, self.dt, 0, 0, 0],
                                   [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                                   [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, self.dt, 0.5*self.dt*self.dt],
                                   [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, self.dt],
                                   [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])
        
        #Measurement function 
        self.tracker.H = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                   [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                   [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                   [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                   [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                   [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0],
                                   [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0],
                                   [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0],
                                   [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0]])
        
        #Measurement noise
        self.tracker.R = np.array([[15.0, 0, 0, 0, 0, 0, 0, 0, 0],
                                   [0, 15.0, 0, 0, 0, 0, 0, 0, 0],
                                   [0, 0, 0.01, 0, 0, 0, 0, 0, 0],
                                   [0, 0, 0, 25.0, 0, 0, 0, 0, 0],
                                   [0, 0, 0, 0, 25.0, 0, 0, 0, 0],
                                   [0, 0, 0, 0, 0, 25.0, 0, 0, 0],
                                   [0, 0, 0, 0, 0, 0, 20.0, 0, 0],
                                   [0, 0, 0, 0, 0, 0, 0, 20.0, 0],
                                   [0, 0, 0, 0, 0, 0, 0, 0, 20.0]])

        #Process noise
        x = Q_discrete_white_noise(dim=3, dt=self.dt, var=10.0)
        y = Q_discrete_white_noise(dim=3, dt=self.dt, var=10.0)
        z = Q_discrete_white_noise(dim=3, dt=self.dt, var=0.1)
        roll = Q_discrete_white_noise(dim=3, dt=self.dt, var=15.0)
        pitch = Q_discrete_white_noise(dim=3, dt=self.dt, var=15.0)
        yaw = Q_discrete_white_noise(dim=3, dt=self.dt, var=15.0)
        self.tracker.Q = block_diag(x, y, z, roll, pitch, yaw)
        
        #State mean
        self.tracker.x = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]).T
        
        #Variance of the state
        self.tracker.P = np.eye(18) * 500.

    def first_update(self,data):
    
        #Init values for Kalman filtering
        self.is_state_initialized = True
        print(data)
        #self.tracker.x = np.array([data[0], 0., 0., data[1], 0., 0., data[2], 0., 0.])
        self.tracker.x = [data[0], 0., data[1], data[2], 0., data[3], data[4], 0., data[5], data[6], data[7], 0., data[8], data[9], 0., data[10], data[11], 0.]
        print(self.tracker.x[0])
        self.tracker.predict()
        self.tracker.update(data)
    
    def get_measurement(self, data):

        #Initialize kalman filter
        #if not self.is_state_initialized:
        #    self.first_update(data)
        #else:
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
"""
class ekf():
    
    def __init__(self):
        
        self.numstates = 12 #States
        self.dt = 1.0/50.0 # Sample Rate of the Measurements is 50Hz
        self.dtVision = 1.0/10.0 # Sample Rate of GPS is 10Hz
        xs, ys, zs, dts, xs, ys, lats, lons = symbols('x y z \dotx \doty \dot \alpha \beta \gamma ')
        
        self.gs = Matrix([[xs+(vs/dpsis)*(sin(psis+dpsis*dts)-sin(psis))],
            [ys+(vs/dpsis)*(-cos(psis+dpsis*dts)+cos(psis))],
            [psis+dpsis*dts],
            [vs],
            [dpsis]])
        
        state = Matrix([xs,ys,psis,vs,dpsis])

"""
class sensor_fusion():

    def __init__(self):
        
        #Init ROS node
        rospy.init_node('sensor_fusion')

        self.dt = 1./50.
        self.kf = kalman_filter(self.dt)

        self.imu_data = Imu()
        self.imu_data_corrected = []
        self.aruco_marker_pose = PoseStamped()
        self.sensor_fusion_pose = PoseStamped()
        self.aruco_board_found = Bool()
        self.aruco_board_found = False
        self.start_kf_tracking = False

        #IMU update rate 50Hz (rostopic hz /mavros/imu/data)
        self.old_imu_time = 0.0
        self.new_imu_time = 0.0

        #Subscribers
        rospy.Subscriber('/mavros/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/onboard/aruco_marker_pose', PoseStamped, self.aruco_marker_pose_callback)
        rospy.Subscriber('/onboard/aruco_board_found', Bool, self.aruco_board_found_callback)

        #Publishers
        self.sensor_fusion_pose_pub = rospy.Publisher('/onboard/sensor_fusion',PoseStamped, queue_size=1)
        
        #Offset IMU found from test in Gazebo where the drone was in off mode in 20 sec
        self.acc_x_offset = -0.190006656546
        self.acc_y_offset = -0.174740895383
        self.acc_z_offset = 9.79531049538
        self.gyro_x_offset = -0.000143702855887
        self.gyro_y_offset = -0.000105893216729
        self.gyro_z_offset = 0.0018871804653

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
            
        #Init data textfile
        data = Path('../../../../data/create_ekf/data.txt')
        self.create_ekf_path = '../../../../data/create_ekf/data.txt'
        if not data.is_file:
            data = open(self.imu_data_path,'r+')
            data.truncate(0)
            data.close
        else:
            data = open(self.imu_data_path,'w+')
            data.close()

        rospy.Timer(rospy.Duration(self.dt), self.timer_callback)
        rospy.spin()
        
    def aruco_board_found_callback(self, msg):
        self.aruco_board_found = msg
        self.start_kf_tracking = True

    def aruco_marker_pose_callback(self, data):
        self.aruco_marker_pose = data

    def imu_callback(self, data):
        self.imu_data = data

    def write_imu_data(self, acc_x=0, acc_y=0, acc_z=0, gyro_x=0, gyro_y=0, gyro_z=0, time=0):

        data = open(self.imu_data_path,'a')
        data.write(str(acc_x) + " " + str(acc_y) + " " + str(acc_z) + " " + str(gyro_x) + " " + str(gyro_y) + " " + str(gyro_z) + " " + str(time))
        data.write('\n')
        data.close()
        
    def write_vision_imu_data(self, x=0, acc_x=0, y=0, acc_y=0, z=0, acc_z=0, a=0, gyro_x=0, b=0, gyro_y=0, g=0, gyro_z=0, time=0):
        
        data = open(self.create_ekf_path,'a')
        data.write(str(x) + " " + str(acc_x) + " " + \
                   str(y) + " " + str(acc_y) + " " + \
                   str(z) + " " + str(acc_z) + " " + \
                   str(a) + " " + str(gyro_x) + " " + \
                   str(b) + " " + str(gyro_y) + " " + \
                   str(g) + " " + str(gyro_z) + " " + \
                   str(time))
        data.write('\n')
        data.close()
    
    def correct_imu_offset(self):
        
        q = (self.aruco_marker_pose.pose.orientation.x,
             self.aruco_marker_pose.pose.orientation.y,
             self.aruco_marker_pose.pose.orientation.z,
             self.aruco_marker_pose.pose.orientation.w)
        
        euler = euler_from_quaternion(q)
        acc_x = np.cos(euler[2])*self.imu_data.linear_acceleration.x - np.sin(euler[2])*self.imu_data.linear_acceleration.y
        acc_y = np.sin(euler[2])*self.imu_data.linear_acceleration.x + np.cos(euler[2])*self.imu_data.linear_acceleration.y
        acc_z = self.imu_data.linear_acceleration.z

        gyro_x = self.imu_data.angular_velocity.x
        gyro_y = self.imu_data.angular_velocity.y
        gyro_z = self.imu_data.angular_velocity.z
        
        time = self.imu_data.header.stamp.secs + self.imu_data.header.stamp.nsecs/1000000000.

        self.imu_data_corrected = [acc_x - self.acc_x_offset, 
                                   acc_y - self.acc_y_offset, 
                                   acc_z - self.acc_z_offset, 
                                   gyro_x - self.gyro_x_offset,
                                   gyro_y - self.gyro_y_offset, 
                                   gyro_z - self.gyro_z_offset, 
                                   time]
        
        #For testing and visualisation
        """
        if time > 10 and time < 30:
            self.write_imu_data(self.imu_data_corrected[0], self.imu_data_corrected[1],
                                self.imu_data_corrected[2], self.imu_data_corrected[3],
                                self.imu_data_corrected[4], self.imu_data_corrected[5],
                                self.imu_data_corrected[6])
        """

    def get_sensor_data(self):
        
        x, y, z, roll, pitch, yaw = 0., 0., 0., 0., 0., 0.
        acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = 0., 0., 0., 0., 0., 0.

        if self.aruco_board_found:
            q = (self.aruco_marker_pose.pose.orientation.x,
                 self.aruco_marker_pose.pose.orientation.y,
                 self.aruco_marker_pose.pose.orientation.z,
                 self.aruco_marker_pose.pose.orientation.w)
            
            euler = euler_from_quaternion(q)
            
            x = self.aruco_marker_pose.pose.position.x
            y = self.aruco_marker_pose.pose.position.y
            z = self.aruco_marker_pose.pose.position.z
            
            roll = euler[0]
            pitch = euler[1]
            yaw = euler[2]

        acc_x = (self.imu_data.linear_acceleration.x-self.acc_x_offset) 
        #np.cos(yaw)*(self.imu_data.linear_acceleration.x-self.acc_x_offset) - np.sin(yaw)*(self.imu_data.linear_acceleration.y-self.acc_y_offset)
        
        acc_y = (self.imu_data.linear_acceleration.y-self.acc_y_offset) 
        #np.sin(yaw)*(self.imu_data.linear_acceleration.x-self.acc_x_offset) + np.cos(yaw)*(self.imu_data.linear_acceleration.y-self.acc_y_offset)
        
        acc_z = self.imu_data.linear_acceleration.z-self.acc_z_offset

        gyro_x = self.imu_data.angular_velocity.x-self.gyro_x_offset
        gyro_y = self.imu_data.angular_velocity.y-self.gyro_y_offset
        gyro_z = self.imu_data.angular_velocity.z-self.gyro_z_offset
        
        if self.aruco_board_found:
            time = self.imu_data.header.stamp.secs + self.imu_data.header.stamp.nsecs/1000000000.
            self.write_vision_imu_data(x, acc_x, y, acc_y, z, acc_z, roll, gyro_x, pitch, gyro_y, yaw, gyro_z, time)

        data = []
        if self.aruco_board_found:
            data = [x, acc_x, y, acc_y, z, acc_z, roll, gyro_x, pitch, gyro_y, yaw, gyro_z]
        else:
            data = [acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z]

        return data

    def timer_callback(self,event):
        
        
        self.get_sensor_data()
        """
        if self.start_kf_tracking:
            if not self.aruco_board_found:
                self.kf.tracker.H = np.array([[0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                              [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                              [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                                              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                                              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0]])
            else:
                self.kf.tracker.H = np.array([[1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                              [0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                              [0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                              [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0],
                                              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0],
                                              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0]])
            
            data = self.get_sensor_data(self.kf.tracker.x[15][0])
            self.kf.get_measurement(data)
        """
if __name__ == "__main__":

        #This is only for testing of the Kalman filter
        sf = sensor_fusion()
