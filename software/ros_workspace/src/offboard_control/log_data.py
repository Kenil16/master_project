#!/usr/bin/env python2

from pathlib import Path
from tf.transformations import euler_matrix, euler_from_matrix, quaternion_from_euler, euler_from_quaternion, quaternion_matrix, quaternion_from_matrix
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
import numpy as np

class log_data():

    def __init__(self):

        #Variables for aruco pose estimation error 
        self.aruco_x = []
        self.aruco_y = []
        self.aruco_z = []
        self.aruco_roll = []
        self.aruco_pitch = []
        self.aruco_yaw = []
        
        self.delta_x = 0.0
        self.delta_y = 0.0
        self.delta_z = 0.0

        self.ground_truth_time = 0.0
        
        #Reset data files for new tests 
        self.reset_data_files()

    def reset_data_files(self):

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
        
        #IMU, vision and ground truth data to be used in creating UKF
        data = Path('../../../../data/imu_vision_data.txt')
        self.imu_vision_data_path = '../../../../data/imu_vision_data.txt'
        if not data.is_file:
            data = open(self.imu_vision_data_path,'r+')
            data.truncate(0)
            data.close
        else:
            data = open(self.imu_vision_data_path,'w+')
            data.close()
            
        #Sensor fusion data to be used in sensor fusion analysis
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

        #GPS2Vision data for analysing error in aruco pose estimation 
        data = Path('../../../../data/GPS2Vision_aruco_pose_estimation.txt')
        self.GPS2Vision_aruco_pose_estimation_path = '../../../../data/GPS2Vision_aruco_pose_estimation.txt'
        if not data.is_file:
            data = open(self.GPS2Vision_aruco_pose_estimation_path,'r+')
            data.truncate(0)
            data.close
        else:
            data = open(self.GPS2Vision_aruco_pose_estimation_path,'w+')
            data.close()

        #Hold pose data for analysing error in keeping the current pose from aruco marker estimation
        data = Path('../../../../data/hold_pose_using_aruco_pose_estimation.txt')
        self.hold_pose_using_aruco_pose_estimation_path = '../../../../data/hold_pose_using_aruco_pose_estimation.txt'
        if not data.is_file:
            data = open(self.hold_pose_using_aruco_pose_estimation_path,'r+')
            data.truncate(0)
            data.close
        else:
            data = open(self.hold_pose_using_aruco_pose_estimation_path,'w+')
            data.close()
    
        #Vision landing according to  pose from aruco marker estimation
        data = Path('../../../../data/vision_landing_precision_and_accuracy.txt')
        self.vision_landing_precision_and_accuracy_path = '../../../../data/vision_landing_precision_and_accuracy.txt'
        if not data.is_file:
            data = open(self.vision_landing_precision_and_accuracy_path,'r+')
            data.truncate(0)
            data.close
        else:
            data = open(self.vision_landing_precision_and_accuracy_path,'w+')
            data.close()
        
        """These data files are just used to track the transitions beetween GPS, locate board, navigate to board 
           and gps2vision for visualizations of the transitions between the substates of the GPS2Vision procedure"""
        #Data for GPS navigation 
        data = Path('../../../../data/gps2vision.txt')
        self.gps2vision_path = '../../../../data/gps2vision.txt'
        if not data.is_file:
            data = open(self.gps2vision_path,'r+')
            data.truncate(0)
            data.close
        else:
            data = open(self.gps2vision_path,'w+')
            data.close()
        """
        #Data for locating the board after last waypoint from GPS navigation
        data = Path('../../../../data/locate_board_navigation.txt')
        self.locate_board_navigation_path = '../../../../data/locate_board_navigation.txt'
        if not data.is_file:
            data = open(self.locate_board_navigation_path,'r+')
            data.truncate(0)
            data.close
        else:
            data = open(self.locate_board_navigation_path,'w+')
            data.close()
    
        #Data for navigating to board after successfully locating the board
        data = Path('../../../../data/navigate_to_board_navigation.txt')
        self.navigate_to_board_navigation_path = '../../../../data/navigate_to_board_navigation.txt'
        if not data.is_file:
            data = open(self.navigate_to_board_navigation_path,'r+')
            data.truncate(0)
            data.close
        else:
            data = open(self.navigate_to_board_navigation_path,'w+')
            data.close()
    
        #Data for transition between gps2vision navigation
        data = Path('../../../../data/transition_gps2vision_navigation.txt')
        self.transition_gps2vision_navigation_path = '../../../../data/transition_gps2vision_navigation.txt'
        if not data.is_file:
            data = open(self.transition_gps2vision_navigation_path,'r+')
            data.truncate(0)
            data.close
        else:
            data = open(self.transition_gps2vision_navigation_path,'w+')
            data.close()
        """ 
    def write_gps2vision_data(self, ground_truth, state, intervals = 0.5):

        #State represents the state (substate) 0->GPS, 1->locate_board, 2->navigate_to_board, 3-> gps2vision_transition
        
        #Get time
        time = ground_truth.header.stamp.secs + ground_truth.header.stamp.nsecs/1000000000.
        
        #Only plot with a certain interval between observations 
        if (time-self.ground_truth_time) > intervals:
            g_x, g_y, g_z, g_roll, g_pitch, g_yaw = self.ground_truth_to_aruco_pose(ground_truth)
            data = open(self.gps2vision_path,'a')
            data.write(str(g_x) + " " + str(g_y) + " " + str(time) + " " + str(state))
            data.write('\n')
            data.close()
            self.ground_truth_time = time
    """
    def write_locate_board_navigation_data(self, ground_truth, intervals = 0.5):
        
        #Get time
        time = ground_truth.header.stamp.secs + ground_truth.header.stamp.nsecs/1000000000.
        
        #Only plot with a certain interval between observations 
        if (time-self.ground_truth_time) > intervals:
            g_x, g_y, g_z, g_roll, g_pitch, g_yaw = self.ground_truth_to_aruco_pose(ground_truth)
            data = open(self.locate_board_navigation_path,'a')
            data.write(str(g_x) + " " + str(g_y))
            data.write('\n')
            data.close()
            self.ground_truth_time = time

    def write_navigato_to_board_navigation_data(self, ground_truth, intervals = 0.5):
        
        #Get time
        time = ground_truth.header.stamp.secs + ground_truth.header.stamp.nsecs/1000000000.
        
        #Only plot with a certain interval between observations 
        if (time-self.ground_truth_time) > intervals:
            g_x, g_y, g_z, g_roll, g_pitch, g_yaw = self.ground_truth_to_aruco_pose(ground_truth)
            data = open(self.navigate_to_board_navigation_path,'a')
            data.write(str(g_x) + " " + str(g_y))
            data.write('\n')
            data.close()
            self.ground_truth_time = time

    def write_transition_gps2vision_navigation_data(self, ground_truth, intervals = 0.5):
        
        #Get time
        time = ground_truth.header.stamp.secs + ground_truth.header.stamp.nsecs/1000000000.
        
        #Only plot with a certain interval between observations 
        if (time-self.ground_truth_time) > intervals:
            g_x, g_y, g_z, g_roll, g_pitch, g_yaw = self.ground_truth_to_aruco_pose(ground_truth)
            data = open(self.transition_gps2vision_navigation_path,'a')
            data.write(str(g_x) + " " + str(g_y))
            data.write('\n')
            data.close()
            self.ground_truth_time = time
    """
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

    def write_hold_pose_using_aruco_pose_estimation_data(self, marker_pose, 
                                                         setpoint_x, setpoint_y, setpoint_z,
                                                         setpoint_roll, setpoint_pitch, setpoint_yaw, ground_truth):
        
        #Get ground truth transformed from world to aruco pose 
        g_x, g_y, g_z, g_roll, g_pitch, g_yaw = self.ground_truth_to_aruco_pose(ground_truth)

        #Get time
        time = marker_pose.header.stamp.secs + marker_pose.header.stamp.nsecs/1000000000.

        #Get setpoint data
        x = marker_pose.pose.position.x
        y = marker_pose.pose.position.y
        z = marker_pose.pose.position.z
        euler = euler_from_quaternion([marker_pose.pose.orientation.x,
                                       marker_pose.pose.orientation.y,
                                       marker_pose.pose.orientation.z,
                                       marker_pose.pose.orientation.w])

        data = open(self.hold_pose_using_aruco_pose_estimation_path,'a')
        data.write(str(x) + " " + str(y) + " " + str(z) + " " + \
                   str(np.rad2deg(euler[0])) + " " + str(np.rad2deg(euler[1])) + " " + str(np.rad2deg(euler[2])) + " " + \
                   str(setpoint_x) + " " + str(setpoint_y) + " " + str(setpoint_z) + " " + \
                   str(setpoint_roll) + " " + str(setpoint_pitch) + " " + str(setpoint_yaw) + " " + \
                   str(g_x) + " " + str(g_y) + " " + str(g_z) + " " + \
                   str(g_roll) + " " + str(g_pitch) + " " + str(g_yaw) + " " + str(time))
        data.write('\n')
        data.close()

    def write_vision_landing_precision_and_accuracy_data(self, landing_station, marker_pose, setpoint_x, setpoint_y, ground_truth, pre_landing_stabilization_time, landing_time):
        
        #Get ground truth transformed from world to aruco pose 
        g_x, g_y, g_z, g_roll, g_pitch, g_yaw = self.ground_truth_to_aruco_pose(ground_truth)

        #Get marker pose estimate
        x = marker_pose.pose.position.x
        y = marker_pose.pose.position.y

        data = open(self.vision_landing_precision_and_accuracy_path,'a')
        data.write(str(landing_station) + " " + str(x) + " " + str(y) + " " + \
                str(setpoint_x) + " " + str(setpoint_y) + " " + \
                   str(g_x) + " " + str(g_y) + " " + str(pre_landing_stabilization_time) + " " + str(landing_time))
        data.write('\n')
        data.close()
    
    def write_sensor_fusion_data(self, x, y, z, acc_x, acc_y, roll, pitch, yaw, 
                                 gyro_x, gyro_y, gyro_z, baro, baro_corrected, baro_bias, 
                                 vision_x, vision_y, vision_z, time, g_roll, g_pitch, g_yaw, g_x, g_y, g_z):

        data = open(self.sensor_fusion_data_path,'a')
        data.write(str(x) + " " + str(y) + " " + \
                   str(z) + " " + str(acc_x) + " " +\
                   str(acc_y) + " " + \
                   str(roll) + " " + str(pitch) + " " + \
                   str(yaw) + " " + str(gyro_x) + " " + \
                   str(gyro_y) + " " + str(gyro_z) + " " + \
                   str(baro) + " " + str(baro_corrected) + " " + str(baro_bias) + " " + \
                   str(vision_x) + " " + str(vision_y) + " " + str(vision_z) + " " + str(time) + " " + \
                   str(g_roll) + " " + str(g_pitch) + " " + str(g_yaw) + " " + str(g_x) + " " + str(g_y) + " " + str(g_z))
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
        g_x = T[0][3] + 7.4/2
        g_y = T[1][3] + 7.4/2
        g_z = T[2][3]

        g_roll = np.rad2deg(g_euler[0])
        g_pitch = np.rad2deg(-g_euler[1])
        g_yaw = np.rad2deg(g_euler[2])

        #g_roll = np.rad2deg( (360-(g_euler[0]*180)/np.pi + 90 + 180 % 360 - 180) )
        #g_pitch = np.rad2deg( (360-(g_euler[1]*180)/np.pi + 90 + 180 % 360 - 180) )
        #g_yaw = np.rad2deg( (360-(g_euler[2]*180)/np.pi + 90 + 180 % 360 - 180) )
        
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
        pitch = np.rad2deg(euler[1])
        yaw = np.rad2deg(euler[2])

        #Write data to file
        data = open(self.aruco_pose_estimation,'a')
        data.write(str(x) + " " + str(y) + " " + str(z) + " " + \
                   str(roll) + " " + str(pitch) + " " + str(yaw) + " " + \
                   str(g_x) + " " + str(g_y) + " " + str(g_z) + " " + \
                   str(g_roll) + " " + str(g_pitch) + " " + str(g_yaw) + " " + str(time))
        data.write('\n')
        data.close()
        
    def write_GPS2Vision_marker_detection_data(self, setpoint_x, setpoint_y):
        
        mean_error_x = sum(self.aruco_x)/len(self.aruco_x)
        mean_error_y = sum(self.aruco_y)/len(self.aruco_y)
        mean_error_z = sum(self.aruco_z)/len(self.aruco_z)

        mean_error_roll = sum(self.aruco_roll)/len(self.aruco_roll)
        mean_error_pitch = sum(self.aruco_pitch)/len(self.aruco_pitch)
        mean_error_yaw = sum(self.aruco_yaw)/len(self.aruco_yaw)

        #Write data to file for analyzing
        data = open(self.GPS2Vision_aruco_pose_estimation_path,'a')
        data.write(str(setpoint_x) + " " + str(setpoint_y) + " " + \
                   str(mean_error_x) + " " + str(mean_error_y) + " " + str(mean_error_z) + " " + \
                   str(mean_error_roll) + " " + str(mean_error_pitch) + " " + str(mean_error_yaw))
        data.write('\n')
        data.close()

        self.aruco_x = []
        self.aruco_y = []
        self.aruco_z = []
        self.aruco_roll = []
        self.aruco_pitch = []
        self.aruco_yaw = []

    def calculate_error_pose(self, aruco_pose, ground_truth):

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
        g_x = T[0][3] + 7.4/2
        g_y = T[1][3] + 7.4/2
        g_z = T[2][3]

        g_roll = np.rad2deg(g_euler[0])
        g_pitch = np.rad2deg(-g_euler[1])
        g_yaw = np.rad2deg(g_euler[2])


        angle = euler_from_quaternion([aruco_pose.pose.orientation.x,
                                       aruco_pose.pose.orientation.y,
                                       aruco_pose.pose.orientation.z,
                                       aruco_pose.pose.orientation.w])

        error_x = abs(aruco_pose.pose.position.x-g_x)
        error_y = abs(aruco_pose.pose.position.y-g_y)
        error_z = abs(aruco_pose.pose.position.z-g_z)
        error_roll = abs(np.rad2deg(angle[0])-g_roll)
        error_pitch = abs(np.rad2deg(angle[1])-g_pitch)
        error_yaw = abs(np.rad2deg(angle[2])-g_yaw)

        self.aruco_x.append(error_x)
        self.aruco_y.append(error_y)
        self.aruco_z.append(error_z)
        self.aruco_roll.append(error_roll)
        self.aruco_pitch.append(error_pitch)
        self.aruco_yaw.append(error_yaw)
    
    def ground_truth_to_aruco_pose(self, ground_truth):

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
        g_x = T[0][3] + 7.4/2
        g_y = T[1][3] + 7.4/2
        g_z = T[2][3]

        g_roll = np.rad2deg(g_euler[0])
        g_pitch = np.rad2deg(-g_euler[1])
        g_yaw = np.rad2deg(g_euler[2])

        return g_x, g_y, g_z, g_roll, g_pitch, g_yaw

if __name__ == "__main__":
    ld = log_data()

