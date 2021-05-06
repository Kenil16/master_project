#!/usr/bin/env python2

import numpy as np
from scipy.linalg import block_diag
#from timeit import default_timer as timer
#from pathlib import Path

import rospy
from sensor_msgs.msg import Imu
from mavros_msgs.msg import Altitude
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import* 
from std_msgs.msg import (String, Int8, Float64, Bool)
from nav_msgs.msg import Odometry 
from scipy.stats import norm
from hector_uav_msgs.msg import Altimeter 
from ukf import UKF
from log_data import*

class sensor_fusion():

    def __init__(self):

        #Initialise the uncented kalman filter (UKF)
        np.set_printoptions(precision=3)
        
        # Process Noise
        self.q = np.eye(12)
        self.q[0][0] = 0.5 #x 0.5
        self.q[1][1] = 0.5 #y 0.5
        self.q[2][2] = 0.5 #z 0.5
        self.q[3][3] = 0.5 #vel x 0.5
        self.q[4][4] = 0.5 #vel y 0.5
        self.q[5][5] = 0.5 #roll 0.5
        self.q[6][6] = 0.5 #pitch 0.5
        self.q[7][7] = 0.5 #yaw 0.5
        self.q[8][8] = 0.5 #rate roll 0.5
        self.q[9][9] = 0.5 #rate pitch 0.5
        self.q[10][10] = 0.5 #rate yaw 0.5
        self.q[11][11] = 3.0 #baro bias 2.5


        # Create measurement noise covariance matrices
        self.r_imu_acc = np.zeros([2, 2])
        self.r_imu_gyro_v = np.zeros([3, 3])
        self.r_imu_acc[0][0] = 0.3 #acc x 0.5
        self.r_imu_acc[1][1] = 0.3 #acc y 0.5
        self.r_imu_gyro_v[0][0] = 0.03 #gyro roll 0.05
        self.r_imu_gyro_v[1][1] = 0.03 #gyro pitch 0.05
        self.r_imu_gyro_v[2][2] = 0.03 #gyro yaw 0.05

        self.r_vision_pos = np.zeros([3, 3])
        self.r_vision_ori = np.zeros([3, 3])
        self.r_vision_pos[0][0] = 0.011 #x 0.0005 
        self.r_vision_pos[1][1] = 0.011 #y 0.0005
        self.r_vision_pos[2][2] = 0.011 #z 0.0005
        self.r_vision_ori[0][0] = 0.0054 #roll rate 0.0005
        self.r_vision_ori[1][1] = 0.0054 #pitch rate 0.0005
        self.r_vision_ori[2][2] = 0.0054 #yaw rate 0.0005

        self.r_baro = np.zeros([1,1])
        self.r_baro[0][0] = 0.5 #0.5

        self.r_baro_offset = np.zeros([1,1])
        self.r_baro_offset[0][0] = 0.5 #0.005

        #For visualisation of data
        self.write_data_log = True
        self.log_data = log_data()

        self.x0 = [] #x
        self.x1 = [] #y
        self.x2 = [] #z
        self.x3 = [] #vel_x
        self.x4 = [] #vel_y
        self.x5 = [] #vel_z
        self.x6 = [] #roll
        self.x7 = [] #pitch
        self.x8 = [] #yaw
        self.x9 = [] #psi
        self.x10 = [] #phi
        self.x11 = [] #theta

        self.vision_x = 0.0
        self.vision_y = 0.0
        self.vision_z = 0.0
        
        self.vision_roll = 0.0
        self.vision_pitch = 0.0
        self.vision_yaw = 0.0

        self.imu_accX = 0.0
        self.imu_accY = 0.0
        
        self.imu_rollV = 0.0
        self.imu_pitchV = 0.0
        self.imu_yawV = 0.0
        
        self.baro = 0.0
        self.baro_offset = 0.0
        self.baro_corrected = 0.0

        self.last_sample_time_imu = 0.0
        self.last_sample_time_vision = 0.0
        
        self.seq_imu = 0
        self.seq_aruco_pose = 0
        self.seq_baro = 0
        self.init_ukf = True

        self.inversions_z = []
        self.last_step_z = 0
        self.start_z = 0
        self.attenuation_vel_z = 1.0

        self.ground_truth = Odometry()
        self.imu_data = Imu()
        self.baro_data = Altimeter()
        
        self.init_baro_altitude = True
        self.altitude_ref = []

        self.imu_data_corrected = []
        self.aruco_marker_pose = PoseStamped()
        self.sensor_fusion_pose = PoseStamped()
        self.aruco_board_found = Bool()
        self.aruco_board_found = False
        self.start_tracking = False

        self.measurements = []
        
        #Offset IMU found from test in Gazebo where the drone was in off mode in 20 sec
        self.acc_x_offset = -0.190006656546
        self.acc_y_offset = -0.174740895383
        self.acc_z_offset = 9.79531049538
        self.gyro_x_offset = -0.000143702855887
        self.gyro_y_offset = -0.000105893216729
        self.gyro_z_offset = 0.0018871804653

    def initialize_ukf(self, aruco_board_found, aruco_marker_pose):
        
        if self.init_ukf and aruco_board_found and aruco_marker_pose.header.seq > 0:
            
            #Choose initial values for the UKF
            i = self.get_measurements()
            self.x_init = [i[0], i[1], i[2], 0, 0, i[6], i[7], i[8], 0, 0, 0, 0]
            
            #Initiate the UKF
            self.state_estimator = UKF(12, self.q, self.x_init, 0.0001*np.eye(12), 0.04, 15.0, 2.0, self.iterate_x)

            self.init_ukf = False
            self.start_tracking = True

    def iterate_x(self, x, dt, inputs):

        ret = np.zeros(len(x))

        ret[0] = x[0] + 0.6*x[3]*dt
        ret[1] = x[1] + 0.6*x[4]*dt
        ret[2] = x[2]
        ret[3] = x[3]
        ret[4] = x[4]
        ret[5] = x[5] + x[9]*dt
        ret[6] = x[6] + x[8]*dt
        ret[7] = x[7] + x[10]*dt
        ret[8] = x[8]
        ret[9] = x[9]
        ret[10] = x[10]
        ret[11] = x[11]

        return ret

    def sensor_fusion_update(self, aruco_marker_pose, imu_data, baro_data, ground_truth):

        #Update internal parameters
        self.aruco_marker_pose = aruco_marker_pose
        self.imu_data = imu_data
        self.baro_data = baro_data
        self.ground_truth = ground_truth
        
        #Get vision, IMU and baro data
        row = self.get_measurements()

        #Update sample time (this is not the same everytime)
        time = self.imu_data.header.stamp.secs + self.imu_data.header.stamp.nsecs/1000000000.
        dt_imu = time-self.last_sample_time_imu
        self.last_sample_time_imu = time

        time = self.aruco_marker_pose.header.stamp.secs + self.aruco_marker_pose.header.stamp.nsecs/1000000000.
        dt_vision = time-self.last_sample_time_vision
        self.last_sample_time_vision = time
        
        #Get first estimate of the state 
        x = self.state_estimator.get_state()

        #Accelerometer bias
        bias = [self.acc_x_offset, self.acc_y_offset]# -0.190006656546, -0.174740895383]

        #Correct for acceleration shift for the orientation of the drone and bias for x and y
        R2 = self.eulerAnglesToRotationMatrix([x[6], x[5], x[7]])
        acc = np.matmul(R2, np.array([row[3], row[4], 0]))
        acc_bias = np.matmul(R2, np.array([bias[0], bias[1], 0]))

        corrected_acc_x = -acc[0] #+ acc_bias[0]
        corrected_acc_y = -acc[1] #+ acc_bias[1]

        #Rotation matrix to align gyro velocity to the orientation of the world (marker)
        R3 = self.eulerAnglesToRotationMatrix([0, 0, x[7]-np.pi])
        corrected_gyro = np.matmul(R3, np.array([row[10], -row[9], row[11]]))

        self.vision_x = row[0]
        self.vision_y = row[1]
        self.vision_z = row[2]
        
        self.vision_roll = row[6]
        self.vision_pitch = row[7]
        self.vision_yaw = row[8]

        self.imu_accX = corrected_acc_x
        self.imu_accY = corrected_acc_y
        
        self.imu_rollV = corrected_gyro[0]
        self.imu_pitchV = corrected_gyro[1]
        self.imu_yawV = corrected_gyro[2]
        
        vision_data_pos = np.array([self.vision_x, self.vision_y, self.vision_z])
        vision_data_ori = np.array([self.vision_roll, self.vision_pitch, self.vision_yaw])
        imu_data_acc = np.array([self.imu_accX, self.imu_accY])
        imu_data_gyro_v = np.array([self.imu_rollV, self.imu_pitchV, self.imu_yawV])
        
        baro_data = np.array([row[12]])
        baro_offset = self.vision_z - baro_data
        
        self.baro = row[12]
        self.baro_corrected = self.baro + x[11]

        #Prediction step for UKF
        self.state_estimator.predict(dt_imu)

        #Update only if new data has been received based on sequence number (ROS messege)
        if not self.seq_aruco_pose == self.aruco_marker_pose.header.seq:
            self.state_estimator.update([0, 1, 2], vision_data_pos, self.r_vision_pos)
            self.state_estimator.update([5, 6, 7], vision_data_ori, self.r_vision_ori)
            self.state_estimator.update([11], baro_offset, self.r_baro_offset)
            self.seq_aruco_pose = self.aruco_marker_pose.header.seq
        
        #Update only if new data has been received based on sequence number (ROS messege)
        if not self.seq_imu == self.imu_data.header.seq:
            self.state_estimator.update([3, 4], imu_data_acc, self.r_imu_acc)
            self.state_estimator.update([8, 9, 10], imu_data_gyro_v, self.r_imu_gyro_v)
            self.seq_imu = self.imu_data.header.seq

        if not self.seq_baro == self.baro_data.header.seq:
            self.state_estimator.update([2], baro_data + x[11], self.r_baro)
            self.seq_baro = self.baro_data.header.seq
 
        #Get estimated state
        x = self.state_estimator.get_state()
        
        self.sensor_fusion_pose.pose.position.x = x[0]
        self.sensor_fusion_pose.pose.position.y = x[1]
        self.sensor_fusion_pose.pose.position.z = x[2]
        self.sensor_fusion_pose.pose.orientation = Quaternion(*quaternion_from_euler(x[5], x[6], x[7],'rxyz'))

    def get_measurements(self):
        
        x, y, z, roll, pitch, yaw = 0., 0., 0., 0., 0., 0.
        acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = 0., 0., 0., 0., 0., 0.

        #Get orientation of marker
        q = (self.aruco_marker_pose.pose.orientation.x,
             self.aruco_marker_pose.pose.orientation.y,
             self.aruco_marker_pose.pose.orientation.z,
             self.aruco_marker_pose.pose.orientation.w)
        euler = euler_from_quaternion(q)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]

        #Get translation of marker
        x = self.aruco_marker_pose.pose.position.x
        y = self.aruco_marker_pose.pose.position.y
        z = self.aruco_marker_pose.pose.position.z
        vision_seq = self.aruco_marker_pose.header.seq

        #Get acceleration from accelerometer
        acc_x = self.imu_data.linear_acceleration.x
        acc_y = self.imu_data.linear_acceleration.y 
        acc_z = self.imu_data.linear_acceleration.z
        imu_seq = self.imu_data.header.seq

        #Get current ground truth orientation
        t_g = quaternion_matrix(np.array([self.ground_truth.pose.pose.orientation.x,
                                          self.ground_truth.pose.pose.orientation.y,
                                          self.ground_truth.pose.pose.orientation.z,
                                          self.ground_truth.pose.pose.orientation.w]))

        #Get current ground truth translation
        t_g[0][3] = self.ground_truth.pose.pose.position.x
        t_g[1][3] = self.ground_truth.pose.pose.position.y
        t_g[2][3] = self.ground_truth.pose.pose.position.z

        #Rotation to align ground truth to aruco marker
        r_m = euler_matrix(0, 0, np.pi/2, 'rxyz')

        #Perform matrix multiplication for pose aligment
        T =  np.matmul(r_m, t_g)

        #Get angles and translation in degress
        g_euler = euler_from_matrix(T,'rxyz')

        #Because plugin is initiated in (0,0,0).
        g_x = T[0][3] + 7.4/2
        g_y = T[1][3] + 7.4/2
        g_z = T[2][3]
        g_roll = np.rad2deg(g_euler[0])
        g_pitch = np.rad2deg(-g_euler[1])
        g_yaw = np.rad2deg(g_euler[2])

        #Get gyro measurements
        gyro_x = self.imu_data.angular_velocity.x-self.gyro_x_offset
        gyro_y = self.imu_data.angular_velocity.y-self.gyro_y_offset
        gyro_z = self.imu_data.angular_velocity.z-self.gyro_z_offset

        #Get altitude from barometer (altimeter)
        altitude = self.baro_data.altitude
        baro_seq = self.baro_data.header.seq

        #Get time of message
        time_imu = self.imu_data.header.stamp.secs + self.imu_data.header.stamp.nsecs/1000000000.
        time_vision = self.aruco_marker_pose.header.stamp.secs + self.aruco_marker_pose.header.stamp.nsecs/1000000000.
        
        if not self.init_ukf and self.write_data_log:
            
            self.log_data.write_vision_imu_data(x, y, z, acc_x, acc_y, acc_z, roll, pitch, yaw, gyro_x, gyro_y, gyro_z, altitude, time_imu, vision_seq, imu_seq, baro_seq, g_roll, g_pitch, g_yaw, 
                    g_x, g_y, g_z, time_vision)
            
            
            i = self.state_estimator.get_state()
            self.log_data.write_sensor_fusion_data(i[0], i[1], i[2], i[3], i[4], #ukf_x, ukf_y, ukf_z , ukf_acc_x, ukf_acc_y
                                                   np.rad2deg(i[5]), np.rad2deg(i[6]), np.rad2deg(i[7]), #ukf_roll, ukf_pitch, ukf_yaw
                                                   np.rad2deg(i[8]), np.rad2deg(i[9]), np.rad2deg(i[10]), #ukf_roll_rate, ukf_pitch_rate, ukf_yaw_rate
                                                   self.baro, self.baro_corrected, i[11], #baro data
                                                   self.vision_x, self.vision_y, self.vision_z,
                                                   time_imu, g_roll, g_pitch, g_yaw, g_x, g_y, g_z) #Time and ground truth of pose
            
        self.measurements = [x, y, z, acc_x, acc_y, acc_z, roll, pitch, yaw, gyro_x, gyro_y, gyro_z, altitude]
        return self.measurements
 
    def eulerAnglesToRotationMatrix(self, theta):

            R_x = np.array([[1,         0,                  0                   ],
                            [0,         np.cos(theta[0]), -np.sin(theta[0]) ],
                            [0,         np.sin(theta[0]), np.cos(theta[0])  ]])

            R_y = np.array([[np.cos(theta[1]),    0,      np.sin(theta[1])  ],
                            [0,                     1,      0                   ],
                            [-np.sin(theta[1]),   0,      np.cos(theta[1])  ]])

            R_z = np.array([[np.cos(theta[2]),    -np.sin(theta[2]),    0],
                            [np.sin(theta[2]),    np.cos(theta[2]),     0],
                            [0,                     0,                      1]])


            R = np.matmul(R_x, np.matmul( R_y, R_z ))
            return R

if __name__ == "__main__":
    sf = sensor_fusion()
