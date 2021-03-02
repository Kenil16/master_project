#!/usr/bin/env python2

import numpy as np
from scipy.linalg import block_diag
from timeit import default_timer as timer
from pathlib import Path

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

class sensor_fusion():

    def __init__(self):

        #Initialise the uncented kalman filter (UKF)
        np.set_printoptions(precision=3)

        self.covariance = [0.05, 0.05, 0.05, #Process noise x, y and z (pos)
                           0.5, 0.5, 0.05, #Process noise x, y and z (acc)
                           5.02, 5.02, 5.5, #Process noise x, y and z (acc)
                           1.5, 1.5, 0.05, #Process noise roll, pitch and yaw (angle)
                           0.9, 0.9, 0.5, #Process noise x, y and z (angle rate)
                           2.3, 2.3, 0.05, #Measurement noise x, y and z (pos)
                           0.03, 0.03, 0.08, #Measurement noise x, y and z (acc)
                           0.05, 0.05, 0.05, #Measurement noise x, y and z (angle)
                           1.5, 1.5, 0.5] #Measurement noise x, y and z (abgle rate)

        #Process Noise
        self.q = np.eye(16)
        self.q[0][0] = self.covariance[0]
        self.q[1][1] = self.covariance[1]
        self.q[2][2] = self.covariance[2]
        self.q[3][3] = self.covariance[3]
        self.q[4][4] = self.covariance[4]
        self.q[5][5] = self.covariance[5]
        self.q[6][6] = self.covariance[6]
        self.q[7][7] = self.covariance[7]
        self.q[8][8] = self.covariance[8]
        self.q[9][9] = self.covariance[9]
        self.q[10][10] = self.covariance[10]
        self.q[11][11] = self.covariance[11]
        self.q[12][12] = self.covariance[12]
        self.q[13][13] = self.covariance[13]
        self.q[14][14] = self.covariance[14]
        self.q[15][15] = 0.5
        

        #Create measurement noise covariance matrices
        self.r_imu_acc = np.zeros([3, 3])
        self.r_imu_gyro_v = np.zeros([3, 3])
        self.r_imu_acc[0][0] = self.covariance[15]
        self.r_imu_acc[1][1] = self.covariance[16]
        self.r_imu_acc[2][2] = self.covariance[17]
        self.r_imu_gyro_v[0][0] = self.covariance[18]
        self.r_imu_gyro_v[1][1] = self.covariance[19]
        self.r_imu_gyro_v[2][2] = self.covariance[20]

        self.r_vision_pos = np.zeros([3, 3])
        self.r_vision_ori = np.zeros([3, 3])
        self.r_vision_pos[0][0] = self.covariance[21]
        self.r_vision_pos[1][1] = self.covariance[22]
        self.r_vision_pos[2][2] = self.covariance[23]
        self.r_vision_ori[0][0] = self.covariance[24]
        self.r_vision_ori[1][1] = self.covariance[25]
        self.r_vision_ori[2][2] = self.covariance[26]

        self.r_baro = np.zeros([1,1])
        self.r_baro[0][0] = 0.05

        self.r_baro_offset = np.zeros([1,1])
        self.r_baro_offset[0][0] = 0.2
        
        #For visualisation of data
        self.plot_sensor_fusion_data = True
        self.plot_vision_imu_data = True

        self.sensor_fusion_data_path = '../../../../data/create_ekf/pyUKF-master/sensor_fusion_data.txt'
        self.imu_vision_data_path = '../../../../data/create_ekf/pyUKF-master/imu_vision_data.txt'
        
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

        #Choose initial values for the UKF
        #self.x_init = [i[0,0], i[1,0], i[2,0], 0, 0, 0, i[6,0], i[7,0], i[8,0], 0 ,0 ,0]

        #Initiate the UKF
        #self.state_estimator = UKF(12, q, x_init, 0.0001*np.eye(12), 0.04, 15.0, 2.0, self.iterate_x)

        self.last_sample_time = 0.0
        self.seq_imu = 0
        self.seq_aruco_pose = 0
        self.seq_baro = 0
        self.init_ukf = True

        self.inversions_z = []
        self.last_step_z = 0
        self.start_z = 0
        self.attenuation_vel_z = 1.0

        #Init ROS node
        rospy.init_node('sensor_fusion')

        self.dt = 1./50.
        #self.kf = kalman_filter(self.dt)

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

        #IMU update rate 50Hz (rostopic hz /mavros/imu/data)
        #self.old_imu_time = 0.0
        #self.new_imu_time = 0.0

        #Subscribers/mavros/altitude
        rospy.Subscriber('/mavros/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/altimeter', Altimeter, self.baro_callback)
        rospy.Subscriber('/onboard/aruco_marker_pose', PoseStamped, self.aruco_marker_pose_callback)
        rospy.Subscriber('/onboard/aruco_board_found', Bool, self.aruco_board_found_callback)
        rospy.Subscriber('/odom', Odometry, self.ground_truth_callback)
        
        #Publishers
        self.sensor_fusion_pose_pub = rospy.Publisher('/onboard/sensor_fusion',PoseStamped, queue_size=1)
        self.imux_pub = rospy.Publisher('/onboard/imux',Float64, queue_size=1)
        self.imuy_pub = rospy.Publisher('/onboard/imuy',Float64, queue_size=1)
        self.imuz_pub = rospy.Publisher('/onboard/imuz',Float64, queue_size=1)
        
        #Offset IMU found from test in Gazebo where the drone was in off mode in 20 sec
        self.acc_x_offset = -0.190006656546
        self.acc_y_offset = -0.174740895383
        self.acc_z_offset = 9.79531049538
        self.gyro_x_offset = -0.000143702855887
        self.gyro_y_offset = -0.000105893216729
        self.gyro_z_offset = 0.0018871804653

        #self.pre_acc_x = 0.0
        #self.pre_acc_y = 0.0
        #self.pre_acc_z = 0.0

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
        
        if self.init_ukf and self.aruco_marker_pose.header.seq > 5:
            
            #Choose initial values for the UKF
            i = self.get_measurements()
            self.x_init = [i[0], i[1], i[2], 0, 0, 0, 0, 0, 0, i[6], i[7], i[8], 0, 0, 0, 0]
            
            #Initiate the UKF
            self.state_estimator = UKF(16, self.q, self.x_init, 0.0001*np.eye(16), 0.04, 15.0, 2.0, self.iterate_x)

            self.init_ukf = False
            self.start_tracking = True

    def aruco_marker_pose_callback(self, data):
        self.aruco_marker_pose = data

    def imu_callback(self, data):
        self.imu_data = data

    def ground_truth_callback(self, data):
        self.ground_truth = data
    
    def baro_callback(self, data):
        self.baro_data = data
        """
        if self.init_baro_altitude:
            self.altitude_ref.append(self.baro_data.altitude)
            if len(self.altitude_ref) == 20:
                self.altitude_ref = sum(self.altitude_ref)/len(self.altitude_ref)
                self.init_baro_altitude = False
        """
    def write_imu_data(self, acc_x=0, acc_y=0, acc_z=0, gyro_x=0, gyro_y=0, gyro_z=0, time=0):

        data = open(self.imu_data_path,'a')
        data.write(str(acc_x) + " " + str(acc_y) + " " + str(acc_z) + " " + str(gyro_x) + " " + str(gyro_y) + " " + str(gyro_z) + " " + str(time))
        data.write('\n')
        data.close()
        
    def write_vision_imu_data(self, x, y, z, acc_x, acc_y, acc_z, roll, pitch, yaw, gyro_x, gyro_y, gyro_z, baro, time, vision_seq, imu_seq, baro_seq, path, g_roll, g_pitch, g_yaw, g_alt):
        
        data = open(path,'a')
        data.write(str(x) + " " + str(y) + " " + \
                   str(z) + " " + str(acc_x) + " " + \
                   str(acc_y) + " " + str(acc_z) + " " + \
                   str(roll) + " " + str(pitch) + " " + \
                   str(yaw) + " " + str(gyro_x) + " " + \
                   str(gyro_y) + " " + str(gyro_z) + " " + \
                   str(baro) + " " + str(time) + " " + str(vision_seq) + " " +\
                   str(imu_seq) + " " + str(baro_seq) + " " + str(g_roll) + " " + str(g_pitch) + " " + str(g_yaw) + " " + str(g_alt))
        data.write('\n')
        data.close()

    def write_sensor_fusion_data(self, x, y, z, ukf_x, ukf_y, ukf_z, acc_x, acc_y, acc_z, roll, pitch, yaw, gyro_x, gyro_y, gyro_z, baro, time, vision_seq, imu_seq, baro_seq, path):
        
        data = open(path,'a')
        data.write(str(x) + " " + str(y) + " " + \
                   str(z) + " " + str(ukf_x) + " " + \
                   str(ukf_y) + " " + str(ukf_z) + " " + str(acc_x) + " " +\
                   str(acc_y) + " " + str(acc_z) + " " + \
                   str(roll) + " " + str(pitch) + " " + \
                   str(yaw) + " " + str(gyro_x) + " " + \
                   str(gyro_y) + " " + str(gyro_z) + " " + \
                   str(baro) + " " + str(time) + " " + str(vision_seq) + " " +\
                   str(imu_seq) + " " + str(baro_seq))
        data.write('\n')
        data.close()

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

    def iterate_x(self, x, dt, inputs):
        '''this function is based on the x_dot and can be nonlinear as needed'''

        ret = np.zeros(len(x))

        ret[0] = x[0] + 0.6*x[3]*dt
        ret[1] = x[1] + 0.6*x[4]*dt
        ret[2] = x[2] # x[5]*dt
        ret[3] = x[6] #x[3]*self.attenuation_vel_x + x[6]*dt
        ret[4] = x[7] #x[4]*self.attenuation_vel_y + x[7]*dt
        ret[5] = x[5]*self.attenuation_vel_z + x[8]*dt
        ret[6] = x[6]
        ret[7] = x[7]
        ret[8] = x[8]
        ret[9] = x[9] + x[13]*dt # #+ np.arctan2(x[6], x[8]+off)
        ret[10] = x[10] + x[12]*dt #+ np.arctan2(-x[7], np.sqrt(x[6]**2 + x[8]**2)+off)
        ret[11] = x[11] + x[14]*dt
        ret[12] = x[12]
        ret[13] = x[13]
        ret[14] = x[14]
        ret[15] = x[15]

        return ret

    def sensor_fusion_update(self):
        
        #Get vision, IMU and baro data
        row = self.get_measurements()

        #Update sample time (this is not the same everytime)
        time = self.imu_data.header.stamp.secs + self.imu_data.header.stamp.nsecs/1000000000.
        dt = time-self.last_sample_time
        self.last_sample_time = time

        #Get first estimate of the state 
        x = self.state_estimator.get_state()

        #Accelerometer bias
        bias = [self.acc_x_offset, self.acc_y_offset]# -0.190006656546, -0.174740895383]
        gravity = 9.79531049538

        #Correct for acceleration shift for the orientation of the drone and bias for x and y
        R2 = self.eulerAnglesToRotationMatrix([x[10], x[9], x[11]])
        acc = np.matmul(R2, np.array([row[3], row[4], 0]))
        acc_bias = np.matmul(R2, np.array([bias[0], bias[1], 0]))

        corrected_acc_x = -acc[0] #+ acc_bias[0]
        corrected_acc_y = -acc[1] #+ acc_bias[1]
        corrected_acc_z = row[5] - gravity*np.cos(x[9])*np.cos(x[10])

        #Mechanical Filtering Window
        if corrected_acc_x < 0.25 and corrected_acc_x > -0.25:
            corrected_acc_x = corrected_acc_x*0.0

        if corrected_acc_y < 0.25 and corrected_acc_y > -0.25:
            corrected_acc_y = corrected_acc_y*0.0

        if corrected_acc_z < 0.25 and corrected_acc_z > -0.25:
            corrected_acc_z = corrected_acc_z*0.0

        #Z attenuation
        zero_acc = False
        sign_shift = False
        if x[8] < 0.005 and x[8] > -0.005:
            zero_acc = True

        if not self.start_z and not zero_acc:
            self.start_z = True
            self.last_step_z = np.sign(x[8])
            self.inversions_z.append(self.last_step_z)

        if self.start_z and zero_acc:
            self.start_z = False

        elif self.start_z and np.sign(x[8]) and self.last_step_z and not (self.last_step_z == np.sign(x[8])):
            self.start_z = False

        if len(self.inversions_z) > 1:
            if not self.inversions_z[-1] == self.inversions_z[-2]:
                self.attenuation_vel_z = 0.95
                self.inversions_z = []
        elif len(self.inversions_z) == 1:
            self.attenuation_vel_z = 1.00


        #Rotation matrix to align gyro velocity to the orientation of the world (marker)
        R3 = self.eulerAnglesToRotationMatrix([0, 0, x[11]-np.pi])
        corrected_gyro = np.matmul(R3, np.array([row[10], row[9], row[11]]))

        vision_x = row[0]
        vision_y = row[1]
        vision_z = row[2]
        vision_roll = row[6]
        vision_pitch = row[7]
        vision_yaw = row[8]

        imu_accX = corrected_acc_x
        imu_accY = corrected_acc_y
        imu_accZ = corrected_acc_z
        imu_rollV = corrected_gyro[0]
        imu_pitchV = corrected_gyro[1]
        imu_yawV = corrected_gyro[2]
        
        vision_data_pos = np.array([vision_x, vision_y, vision_z])
        vision_data_ori = np.array([vision_roll, vision_pitch, vision_yaw])
        imu_data_acc = np.array([imu_accX, imu_accY, imu_accZ])
        imu_data_gyro_v = np.array([imu_rollV, imu_pitchV, imu_yawV])
        
        baro_data = np.array([row[12]])
        baro_offset = vision_z - baro_data
        #print(baro_data)
        
        #Prediction step for UKF
        self.state_estimator.predict(dt)
        
        #Update only if new data has been received based on sequence number (ROS messege)
        if not self.seq_aruco_pose == self.aruco_marker_pose.header.seq:
            self.state_estimator.update([0, 1, 2], vision_data_pos, self.r_vision_pos)
            self.state_estimator.update([9, 10, 11], vision_data_ori, self.r_vision_ori)
            self.state_estimator.update([15], baro_offset, self.r_baro_offset)
            self.seq_aruco_pose = self.aruco_marker_pose.header.seq
        
        #Update only if new data has been received based on sequence number (ROS messege)
        if not self.seq_imu == self.imu_data.header.seq:
            self.state_estimator.update([6, 7, 8], imu_data_acc, self.r_imu_acc)
            self.state_estimator.update([12, 13, 14], imu_data_gyro_v, self.r_imu_gyro_v)
            self.seq_imu = self.imu_data.header.seq

        if not self.seq_baro == self.baro_data.header.seq:
            self.state_estimator.update([2], baro_data + x[15], self.r_baro)
            self.seq_baro = self.baro_data.header.seq
 
        #Get estimated state
        x = self.state_estimator.get_state()
        
        self.sensor_fusion_pose.pose.position.x = x[0]
        self.sensor_fusion_pose.pose.position.y = x[1]
        self.sensor_fusion_pose.pose.position.z = x[2]
        self.sensor_fusion_pose.pose.orientation = Quaternion(*quaternion_from_euler(x[9], x[10], x[11],'rxyz'))
        
        self.sensor_fusion_pose_pub.publish(self.sensor_fusion_pose)

        if self.plot_sensor_fusion_data:
            self.write_sensor_fusion_data(row[0], row[1], row[3], x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7], x[8], x[9], x[10], x[11], x[12], time, self.seq_aruco_pose, 
                    self.seq_imu, self.seq_baro, self.sensor_fusion_data_path)

    def get_measurements(self):
        
        x, y, z, roll, pitch, yaw = 0., 0., 0., 0., 0., 0.
        acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = 0., 0., 0., 0., 0., 0.

        q = (self.aruco_marker_pose.pose.orientation.x,
             self.aruco_marker_pose.pose.orientation.y,
             self.aruco_marker_pose.pose.orientation.z,
             self.aruco_marker_pose.pose.orientation.w)
        
        euler = euler_from_quaternion(q)
        
        x = self.aruco_marker_pose.pose.position.x
        y = self.aruco_marker_pose.pose.position.y
        z = self.aruco_marker_pose.pose.position.z
        vision_seq = self.aruco_marker_pose.header.seq

        
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]

        acc_x = self.imu_data.linear_acceleration.x
        acc_y = self.imu_data.linear_acceleration.y 
        acc_z = self.imu_data.linear_acceleration.z
        imu_seq = self.imu_data.header.seq

        
        ori = euler_from_quaternion( [self.ground_truth.pose.pose.orientation.x, self.ground_truth.pose.pose.orientation.y, 
            self.ground_truth.pose.pose.orientation.z, self.ground_truth.pose.pose.orientation.w] )
        alt = self.ground_truth.pose.pose.position.z

        gyro_x = self.imu_data.angular_velocity.x-self.gyro_x_offset
        gyro_y = self.imu_data.angular_velocity.y-self.gyro_y_offset
        gyro_z = self.imu_data.angular_velocity.z-self.gyro_z_offset

        altitude = self.baro_data.altitude #- self.altitude_ref
        baro_seq = self.baro_data.header.seq

        self.imuz_pub.publish(altitude)
        
        time = self.imu_data.header.stamp.secs + self.imu_data.header.stamp.nsecs/1000000000.
        
        if self.plot_vision_imu_data:
            self.write_vision_imu_data(x, y, z, acc_x, acc_y, acc_z, roll, pitch, yaw, gyro_x, gyro_y, gyro_z, altitude, time, vision_seq, imu_seq, baro_seq, self.imu_vision_data_path,
                    ori[0], ori[1], ori[2], alt)
        
        self.measurements = [x, y, z, acc_x, acc_y, acc_z, roll, pitch, yaw, gyro_x, gyro_y, gyro_z, altitude]
        return self.measurements
 
    def timer_callback(self,event):
        
        #Start sensor fusion
        
        if self.start_tracking:
            self.sensor_fusion_update()
        
        #self.get_measurements()
        

if __name__ == "__main__":

        #This is only for testing of the Kalman filter
        sf = sensor_fusion()
