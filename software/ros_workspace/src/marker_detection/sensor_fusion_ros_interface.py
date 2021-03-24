#!/usr/bin/env python2

#import numpy as np
#from scipy.linalg import block_diag
#from timeit import default_timer as timer
#from pathlib import Path

import rospy
from sensor_msgs.msg import Imu
#from mavros_msgs.msg import Altitude
from geometry_msgs.msg import PoseStamped
#from tf.transformations import* 
from std_msgs.msg import Float64, Bool
from nav_msgs.msg import Odometry 
from scipy.stats import norm
from hector_uav_msgs.msg import Altimeter 
#from ukf import UKF
#from log_data import*
from sensor_fusion_new import*

class sensor_fusion_ros_interface():

    def __init__(self):

        #Init ROS node
        rospy.init_node('sensor_fusion')
        self.dt = 1./50.

        #Variables
        self.aruco_marker_pose = PoseStamped()
        self.aruco_board_found = False
        self.baro_data = Altimeter()
        self.imu_data = Imu()
        self.ground_truth = Odometry()

        #Make sensor fusion object 
        self.sensor_fusion = sensor_fusion_new()

        #Subscribers
        rospy.Subscriber('/mavros/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/altimeter', Altimeter, self.baro_callback)
        rospy.Subscriber('/onboard/aruco_marker_pose', PoseStamped, self.aruco_marker_pose_callback)
        rospy.Subscriber('/onboard/aruco_board_found', Bool, self.aruco_board_found_callback)
        rospy.Subscriber('/odom', Odometry, self.ground_truth_callback)
        
        #Publishers
        self.sensor_fusion_pose_pub = rospy.Publisher('/onboard/sensor_fusion',PoseStamped, queue_size=1)
        #self.imux_pub = rospy.Publisher('/onboard/imux',Float64, queue_size=1)
        #self.imuy_pub = rospy.Publisher('/onboard/imuy',Float64, queue_size=1)
        #self.imuz_pub = rospy.Publisher('/onboard/imuz',Float64, queue_size=1)
        self.ground_truth_pub = rospy.Publisher('/onboard/ground_truth',PoseStamped, queue_size=1)
        
        rospy.Timer(rospy.Duration(self.dt), self.timer_callback)
        rospy.spin()
        
    def aruco_board_found_callback(self, msg):
        self.aruco_board_found = msg

    def aruco_marker_pose_callback(self, data):
        self.aruco_marker_pose = data

    def imu_callback(self, data):
        self.imu_data = data

    def ground_truth_callback(self, data):
        self.ground_truth = data
    
    def baro_callback(self, data):
        self.baro_data = data

    def timer_callback(self,event):

        #Initialize UKF when the aruco marker board has been found
        self.sensor_fusion.initialize_ukf(self.aruco_board_found, self.aruco_marker_pose)
        
        #Start sensor fusion where measurements are based on estimated aruco marker pose, 
        #linear acceleration from accelerometer, angular velocity from gyro and estimated height from altimeter (baro).
        #Ground truth is only used to estimate accuracy and precision of the estimated sensor fusion pose
        if self.sensor_fusion.start_tracking:
            self.sensor_fusion.sensor_fusion_update(self.aruco_marker_pose, self.imu_data, self.baro_data, self.ground_truth)
            self.sensor_fusion_pose_pub.publish(self.sensor_fusion.sensor_fusion_pose)

if __name__ == "__main__":
        sfri = sensor_fusion_ros_interface()
