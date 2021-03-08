#!/usr/bin/env python2
from pathlib import Path
from tf.transformations import euler_matrix, euler_from_matrix, quaternion_from_euler, euler_from_quaternion, quaternion_matrix, quaternion_from_matrix
from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovariance, PoseWithCovarianceStamped, Point
from nav_msgs.msg import Odometry
import numpy as np
class log_data():

    def __init__(self):

        #Aruco pose estimation 
        data = Path('../../../../data/aruco_pose_estimation.txt')
        self.aruco_pose_estimation = '../../../../data/aruco_pose_estimation.txt'
        if not data.is_file:
            data = open(self.aruco_pose_estimation,'r+')
            data.truncate(0)
            data.close
        else:
            data = open(self.aruco_pose_estimation,'w+')
            data.close()
        
        #IMU, vision and ground truth data to be used in sensor fusion analysis
        data = Path('../../../../data/imu_vision_data.txt')
        self.imu_vision_data_path = '../../../../data/imu_vision_data.txt'
        if not data.is_file:
            data = open(self.imu_vision_data_path,'r+')
            data.truncate(0)
            data.close
        else:
            data = open(self.imu_vision_data_path,'w+')
            data.close()
            
        #Sensor fusion data to be compared to ground truth 
        data = Path('../../../../data/sensor_fusion_data.txt')
        self.sensor_fusion_data_path = '../../../../data/sensor_fusion_data.txt'
        if not data.is_file:
            data = open(self.sensor_fusion_data_path,'r+')
            data.truncate(0)
            data.close
        else:
            data = open(self.sensor_fusion_data_path,'w+')
            data.close()

        #IMU noise estimation
        data = Path('../../../../data/estimate_imu_noise.txt')
        self.imu_noise_data_path = '../../../../data/estimate_imu_noise.txt'
        if not data.is_file:
            data = open(self.imu_noise_data_path,'r+')
            data.truncate(0)
            data.close
        else:
            data = open(self.imu_noise_data_path,'w+')
            data.close()


    #To be used in sensor fusion 
    def write_vision_imu_data(self, x, y, z, acc_x, acc_y, acc_z, roll, pitch, 
                              yaw, gyro_x, gyro_y, gyro_z, baro, time, vision_seq, 
                              imu_seq, baro_seq, g_roll, g_pitch, g_yaw, g_x, g_y, g_z):

        data = open(self.imu_vision_data_path,'a')
        data.write(str(x) + " " + str(y) + " " + \
                   str(z) + " " + str(acc_x) + " " + \
                   str(acc_y) + " " + str(acc_z) + " " + \
                   str(roll) + " " + str(pitch) + " " + \
                   str(yaw) + " " + str(gyro_x) + " " + \
                   str(gyro_y) + " " + str(gyro_z) + " " + \
                   str(baro) + " " + str(time) + " " + str(vision_seq) + " " + \
                   str(imu_seq) + " " + str(baro_seq) + " " + str(g_roll) + " " + \
                   str(g_pitch) + " " + str(g_yaw) + " " + str(g_x) + " " + str(g_y) + " " + str(g_z))
        data.write('\n')
        data.close()

    def write_sensor_fusion_data(self, x, y, z, acc_x, acc_y, acc_z, roll, pitch, yaw, 
                                 gyro_x, gyro_y, gyro_z, baro, time, vision_seq, imu_seq, baro_seq,
                                 g_roll, g_pitch, g_yaw, g_x, g_y, g_z):

        data = open(self.sensor_fusion_data_path,'a')
        data.write(str(x) + " " + str(y) + " " + \
                   str(z) + " " + str(acc_x) + " " +\
                   str(acc_y) + " " + str(acc_z) + " " + \
                   str(roll) + " " + str(pitch) + " " + \
                   str(yaw) + " " + str(gyro_x) + " " + \
                   str(gyro_y) + " " + str(gyro_z) + " " + \
                   str(baro) + " " + str(time) + " " + str(vision_seq) + " " +\
                   str(imu_seq) + " " + str(baro_seq) + " " + str(g_roll) + " " +\
                   str(g_pitch) + " " + str(g_yaw) + " " + str(g_x) + " " + str(g_y) + " " + str(g_z))
        data.write('\n')
        data.close()

    #To be used in aruco marker detection 
    def write_marker_detection_data(self, aruco_pose, ground_truth):

        #Rotation to align ground truth to aruco marker
        r_m = euler_matrix(0, 0, np.pi/2, 'rxyz')

        #Get current ground truth orientation
        t_g = quaternion_matrix(np.array([ground_truth.pose.pose.orientation.x,
                                          ground_truth.pose.pose.orientation.y,
                                          ground_truth.pose.pose.orientation.z,
                                          ground_truth.pose.pose.orientation.w]))

        #Get current ground truth translation
        t_g[0][3] = ground_truth.pose.pose.position.x
        t_g[1][3] = ground_truth.pose.pose.position.y
        t_g[2][3] = ground_truth.pose.pose.position.z

        #Perform matrix multiplication for pose aligment
        T =  np.matmul(r_m, t_g)

        #Get angles and translation in degress
        g_euler = euler_from_matrix(T,'rxyz')

        #Because plugin is initiated in (0,0,0).
        g_x = T[0][3] + 7.5/2
        g_y = T[1][3] + 7.5/2
        g_z = T[2][3]
        g_roll = np.rad2deg(g_euler[0])
        g_pitch = np.rad2deg(g_euler[1])
        g_yaw = np.rad2deg(g_euler[2])

        #Get time in seconds
        time = ground_truth.header.stamp.secs + ground_truth.header.stamp.nsecs/1000000000.

        #Get estimated aruco pose
        x = aruco_pose.pose.position.x
        y = aruco_pose.pose.position.y
        z = aruco_pose.pose.position.z
        euler = euler_from_quaternion([aruco_pose.pose.orientation.x,
                                       aruco_pose.pose.orientation.y,
                                       aruco_pose.pose.orientation.z,
                                       aruco_pose.pose.orientation.w])
        roll = np.rad2deg(euler[0])
        pitch = np.rad2deg(-euler[1])
        yaw = np.rad2deg(euler[2])

        #Write data to file
        data = open(self.aruco_pose_estimation,'a')
        data.write(str(x) + " " + str(y) + " " + str(z) + " " + \
                   str(roll) + " " + str(pitch) + " " + str(yaw) + " " + \
                   str(g_x) + " " + str(g_y) + " " + str(g_z) + " " + \
                   str(g_roll) + " " + str(g_pitch) + " " + str(g_yaw) + " " + str(time))
        data.write('\n')
        data.close()

