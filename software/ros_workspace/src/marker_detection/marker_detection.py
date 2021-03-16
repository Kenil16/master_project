#!/usr/bin/python

from pathlib import Path
import cv2
import matplotlib as plt
import numpy as np
import rospy
from os import system
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import* #quaternion_from_euler, euler_from_quaternion, quaternion_matrix,quaternion_from_matrix
import tf
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovariance, PoseWithCovarianceStamped, Point
from std_msgs.msg import Float64, Bool, Int8
from mavlink_msgs.msg import mavlink_lora_aruco
from nav_msgs.msg import Odometry

from pandas import Series

from sensor_fusion import*
from log_data import *

import time

class marker_detection:

    def __init__(self):

        #Init ROS node
        rospy.init_node('marker_detection')

        #Init data test plotting
        #self.plot_data = True
        self.cycle_time = (1./10.)
        #self.plot_timer = int(((20)/self.cycle_time)) #Set to run 10 seconds before plot

        self.plot_time = []
        self.log_data = log_data()
        self.write_data_log = True

        self.ground_truth = Odometry()

        self.draw_markers = True
        self.draw_marker_axis = True
        self.aruco_board_found = False

        self.bottom_img = None 
        self.front_img = None

        #To be used in statistics for marker pose estimation presicion
        self.write_aruco_pose_estimate = []
        self.aruco_pose_estimate = []
        self.write_iterations_rolling_average = 100
        self.iterations_rolling_average = 5
        self.write_rolling_average = True
        self.max_std_rolling_average = 0.03
        self.time = 0.0
        
        #Transformation matrix from gps to vision marker to the ground wrt the drone
        self.T_gps2visionMarker_to_ground = identity_matrix()
        self.T_gps2visionMarker_to_ground = euler_matrix(np.pi/2, 0, 0, 'rxyz')
        self.T_gps2visionMarker_to_ground[0][3] = -3.2 #+ 0.275#
        self.T_gps2visionMarker_to_ground[1][3] = -3.00 #
        self.T_gps2visionMarker_to_ground[2][3] = -2.58 #+ 0.125#
        
        #Transformation matrix from landing marker 1 to the ground wrt the drone
        self.T_landingMarker1_to_ground = identity_matrix()
        self.T_landingMarker1_to_ground = euler_matrix(np.pi/2, 0, 0,'rxyz')
        self.T_landingMarker1_to_ground[0][3] = -0.04 
        self.T_landingMarker1_to_ground[1][3] = -8.53
        self.T_landingMarker1_to_ground[2][3] = -0.08
        
        #Transformation matrix from landing marker 2 to the ground wrt the drone
        self.T_landingMarker2_to_ground = identity_matrix()
        self.T_landingMarker2_to_ground = euler_matrix(np.pi/2, 0, 0,'rxyz')
        self.T_landingMarker2_to_ground[0][3] = -3.38
        self.T_landingMarker2_to_ground[1][3] = -8.59
        self.T_landingMarker2_to_ground[2][3] = -0.07
        
        #Transformation matrix from landing marker 3 to the ground wrt the drone
        self.T_landingMarker3_to_ground = identity_matrix()
        self.T_landingMarker3_to_ground = euler_matrix(np.pi/2, 0, 0,'rxyz')
        self.T_landingMarker3_to_ground[0][3] = -6.73
        self.T_landingMarker3_to_ground[1][3] = -8.64
        self.T_landingMarker3_to_ground[2][3] = -0.07
        
        #Local drone pose
        self.local_position = PoseStamped()
        self.aruco_pose = PoseStamped()
        self.aruco_pose_without_kf = PoseStamped()

        self.aruco_board = 1
        
        #Subscribers
        rospy.Subscriber("/mono_cam_bottom/image_raw", Image, self.bottom_img_callback)
        rospy.Subscriber("/mono_cam_front/image_raw", Image, self.front_img_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_position_callback)
        rospy.Subscriber('/onboard/aruco_board', Int8, self.aruco_board_callback)
        rospy.Subscriber('/odom', Odometry, self.ground_truth_callback)

        #Publishers
        self.aruco_marker_image_pub = rospy.Publisher('/onboard/aruco_marker_image', Image, queue_size=1)
        self.aruco_marker_pose_pub = rospy.Publisher('/onboard/aruco_marker_pose', PoseStamped, queue_size=1)
        self.aruco_marker_found_pub = rospy.Publisher('/onboard/aruco_board_found', Bool, queue_size=1)
        self.aruco_marker_pose_stable_pub = rospy.Publisher('/onboard/aruco_marker_pose_stable', Bool, queue_size=1)
        self.aruco_marker_board_center_pub = rospy.Publisher('/onboard/aruco_marker_board_center', PoseStamped, queue_size=1)

        #Initiate aruco detection (Intinsic and extrinsic camera coefficients can be found in sdu_mono_cam model)
        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_1000)
        
        self.aruco_board_gps2vision = cv2.aruco.GridBoard_create(markersX=4, markersY=2, markerLength=0.2, markerSeparation=0.1, dictionary=self.dictionary,firstMarker=100)
        self.aruco_board_vision = cv2.aruco.GridBoard_create(markersX=25, markersY=25, markerLength=0.2, markerSeparation=0.1, dictionary=self.dictionary,firstMarker=200)
        self.aruco_board_landing = cv2.aruco.GridBoard_create(markersX=2, markersY=6, markerLength=0.2, markerSeparation=0.1, dictionary=self.dictionary,firstMarker=1)
        
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        
        #Bottom camera
        self.camera_matrix_bottom = np.array([[623.680552, 0, (720/2)], [0, 623.680552, (480/2)], [0, 0, 1]], dtype=np.float)
        self.distortion_coefficients_bottom = np.array([[0, 0, 0, 0]], dtype=np.float)
        
        #Front camera
        self.camera_matrix_front = np.array([[623.680552, 0, (720/2)], [0, 623.680552, (480/2)], [0, 0, 1]], dtype=np.float)
        self.distortion_coefficients_front = np.array([[0, 0, 0, 0]], dtype=np.float)
        
        self.marker_pose = []
        self.bridge = CvBridge()
        
        rospy.Timer(rospy.Duration(self.cycle_time), self.timer_callback)
        rospy.spin()

    def  local_position_callback(self, data):
        self.local_position = data
 
    def aruco_board_callback(self, data):
        self.aruco_board = data.data

    def bottom_img_callback(self, data):
        self.bottom_img = data

    def front_img_callback(self, data):
        self.front_img = data
        
    def ground_truth_callback(self, data):
        self.ground_truth = data

    def find_aruco_markers(self, img, aruco_board, camera_matrix, distortion_coefficients):
       
        #Load in the marker image
        self.aruco_board_found = False

        if img == None:
            return
        try:
            cv_img = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            print(e)

        image_markers = cv_img
        
        #Detect aruco markers in the image
        marker_corners, marker_ids, rejected_candidates = cv2.aruco.detectMarkers(cv_img, self.dictionary, parameters=self.parameters, cameraMatrix=camera_matrix, distCoeff=distortion_coefficients)
        
        pose = PoseStamped()
        if len(marker_corners) > 0:

            retval, _rvec, _tvec = cv2.aruco.estimatePoseBoard(marker_corners, marker_ids, aruco_board, camera_matrix, distortion_coefficients, rvec=None, tvec=None)
            
            #Draw markers
            if self.draw_markers:
                image_markers = cv2.aruco.drawDetectedMarkers(cv_img, marker_corners, marker_ids)

            if retval:

                self.aruco_board_found = True 

                pose.pose.position.x = _tvec[0]
                pose.pose.position.y = _tvec[1]
                pose.pose.position.z = _tvec[2]
                pose.pose.orientation = quaternion_from_euler(*self.rodrigues_to_euler_angles(_rvec))
                #print(self.rodrigues_to_euler_angles(_rvec))
                #Draw detected axis of markers
                if self.draw_marker_axis:
                    image_markers = cv2.aruco.drawAxis(image_markers, camera_matrix, distortion_coefficients, _rvec, _tvec, 0.1)

                #This is only used in GPS2Vision if pose estimate is very unstable
                if self.aruco_board == 1:
                    self.center_aruco_board(marker_ids, marker_corners)

        self.marker_pose = pose
        img = self.bridge.cv2_to_imgmsg(image_markers,"bgr8")
        self.aruco_marker_image_pub.publish(img)

    def estimate_marker_pose(self, aruco_board, T_front_to_ground = None):

        #Transformation matrix from camera to ArUco marker
        r = quaternion_matrix(self.marker_pose.pose.orientation)
        
        T_camera_marker = r.T
        
        t = np.array([self.marker_pose.pose.position.x, self.marker_pose.pose.position.y, self.marker_pose.pose.position.z, 1])
        t = -np.dot(T_camera_marker,t)

        if aruco_board == 2: #If bottom cam is used

            T = euler_matrix(0, np.pi, 0, 'rxyz')
            r = np.matmul(r,T)
            euler = euler_from_matrix(r,'rxyz')

            q_new = Quaternion(*quaternion_from_euler(euler[0], euler[1], euler[2],'rxyz'))
            
            #Update ArUco marker pose
            self.aruco_pose.pose.position.x = t[0][0]
            self.aruco_pose.pose.position.y = t[1][0]
            self.aruco_pose.pose.position.z = t[2][0] + 0.05 + 0.02 #Cam offset of drone and shift of marker board 
            self.aruco_pose.pose.orientation = q_new
        
        else:
            
            T = euler_matrix(np.pi, 0, 0, 'rxyz')
            r = np.matmul(r,T)
            euler = euler_from_matrix(r,'rxyz')
            #print('t' + str(t))
            #print('T_front_to_ground'+ str(T_front_to_ground))
            t = np.dot(T_front_to_ground,t)
            q_new = Quaternion(*quaternion_from_euler(-euler[2], -euler[0], euler[1] + 1.5708,'rxyz'))
            
            #Update ArUco marker pose
            self.aruco_pose.pose.position.x = t[0][0]
            self.aruco_pose.pose.position.y = t[1][0]
            self.aruco_pose.pose.position.z = t[2][0]
            self.aruco_pose.pose.orientation = q_new
        
        #Used for data log of estimated aruco pose vs ground truth
        if self.write_data_log:
            self.log_data.write_marker_detection_data(self.aruco_pose, self.ground_truth)

        #print(euler_from_quaternion([self.aruco_pose.pose.orientation.x,self.aruco_pose.pose.orientation.y,self.aruco_pose.pose.orientation.z,self.aruco_pose.pose.orientation.w]))
        #print "Ori: {} x: {} y: {} z: {} \n".format(euler,self.aruco_pose.pose.position.x,self.aruco_pose.pose.position.y,self.aruco_pose.pose.position.z)
        #self.aruco_marker_found_pub.publish(True)
        
        self.aruco_marker_pose_pub.publish(self.aruco_pose)

        #Only use in the transition from GPS2Vision
        if aruco_board == 1:

            if self.write_rolling_average:

                if len(self.write_aruco_pose_estimate) == self.write_iterations_rolling_average:

                    #Just clear file before writing
                    self.log_data.write_rolling_average(0,0,0,0,0,0,True)
                    
                    for i in self.aruco_pose_estimate:
                        self.log_data.write_rolling_average(i[0], i[1], i[2], i[3], i[4], i[5], False)

                    self.write_aruco_pose_estimate = []

                else:
                    self.write_aruco_pose_estimate.append([self.aruco_pose.pose.position.x,
                                                           self.aruco_pose.pose.position.y,
                                                           self.aruco_pose.pose.position.z,
                                                           euler[0], euler[1], euler[2]])
            
            if len(self.aruco_pose_estimate) == self.iterations_rolling_average:

                x = Series(np.array([item[0] for item in self.aruco_pose_estimate]))
                x_std = x.rolling(5).std()

                x = Series(np.array([item[1] for item in self.aruco_pose_estimate]))
                y_std = x.rolling(5).std()
                
                z = Series(np.array([item[2] for item in self.aruco_pose_estimate]))
                z_std = x.rolling(5).std()
                
                roll = Series(np.array([item[3] for item in self.aruco_pose_estimate]))
                roll_std = x.rolling(5).std()
                
                pitch = Series(np.array([item[4] for item in self.aruco_pose_estimate]))
                pitch_std = x.rolling(5).std()

                yaw = Series(np.array([item[5] for item in self.aruco_pose_estimate]))
                yaw_std = x.rolling(5).std()

                if (x_std[4] and y_std[4] and z_std[4] and roll_std[4] and pitch_std[4] and yaw_std[4]) < self.max_std_rolling_average:
                    self.aruco_marker_pose_stable_pub.publish(True)
                else:
                    self.aruco_marker_pose_stable_pub.publish(False)

                self.aruco_pose_estimate = []

            else:
                self.aruco_pose_estimate.append([self.aruco_pose.pose.position.x,
                                                 self.aruco_pose.pose.position.y,
                                                 self.aruco_pose.pose.position.z,
                                                 euler[0], euler[1], euler[2]])
    
    def write_aruco_pos(self, x, y, z, kf_x, kf_y, kf_z, time):
        
        data = open('../../../../data/estimate_aruco_vs_kf_pose/data.txt','a')
        data.write(str(x) + " " + str(y) + " " + str(z) + " " + str(kf_x) + " " + str(kf_y) + " " + str(kf_z) + " " + str(time))
        data.write('\n')
        data.close()

    def rodrigues_to_euler_angles(self, rvec):
        #https://www.programcreek.com/python/example/89450/cv2.Rodrigues
        mat, jac = cv2.Rodrigues(rvec)
        sy = np.sqrt(mat[0, 0] * mat[0, 0] + mat[1, 0] * mat[1, 0])
        singular = sy < 1e-6
        
        if not singular:
            roll = np.math.atan2(mat[2, 1], mat[2, 2])
            pitch = np.math.atan2(-mat[2, 0], sy)
            yaw = np.math.atan2(mat[1, 0], mat[0, 0])
        else:
            roll = np.math.atan2(-mat[1, 2], mat[1, 1])
            pitch = np.math.atan2(-mat[2, 0], sy)
            yaw = 0
            
        return np.array([roll, pitch, yaw])

    def center_aruco_board(self, marker_ids, marker_corners):
        
        centers_y = []
        centers_x = []

        for corner, id_ in zip(marker_corners, marker_ids):
            if id_ > 99 and id_ < 109: #GPS2Vision marker only
                centerX = (corner[0][0][0] + corner[0][1][0] + corner[0][2][0] + corner[0][3][0]) / 4
                centerY = (corner[0][0][1] + corner[0][1][1] + corner[0][2][1] + corner[0][3][1]) / 4
                
                centers_x.append(centerX)
                centers_y.append(centerY)
        
        if len(centers_x) and len(centers_y):
            center_mean_x = sum(centers_x)/len(centers_x)
            center_mean_y = sum(centers_y)/len(centers_y)

            pose = PoseStamped()
            pose.pose.position.x = center_mean_x
            pose.pose.position.y = center_mean_y
            self.aruco_marker_board_center_pub.publish(pose)
    
    def timer_callback(self,event):
        
        #print(self.aruco_board)
        #Either go from GPS to vision, navigate through ground markers or perform a landing (1, 2, 3) all using vision. Zero means do not detect markers
        aruco_board = self.aruco_board

        if aruco_board == 1:
            self.find_aruco_markers(self.front_img, self.aruco_board_gps2vision, self.camera_matrix_front, self.distortion_coefficients_front)
            if self.aruco_board_found:
                self.estimate_marker_pose(aruco_board, self.T_gps2visionMarker_to_ground)
        elif aruco_board == 2:
            self.find_aruco_markers(self.bottom_img, self.aruco_board_vision, self.camera_matrix_bottom, self.distortion_coefficients_bottom)
            if self.aruco_board_found:
                self.estimate_marker_pose(aruco_board)
        elif aruco_board == 3:
            self.find_aruco_markers(self.front_img, self.aruco_board_landing, self.camera_matrix_front, self.distortion_coefficients_front)
            if self.aruco_board_found:
                self.estimate_marker_pose(aruco_board, self.T_landingMarker1_to_ground)
        elif aruco_board == 4:
            self.find_aruco_markers(self.front_img, self.aruco_board_landing, self.camera_matrix_front, self.distortion_coefficients_front)
            if self.aruco_board_found:
                self.estimate_marker_pose(aruco_board, self.T_landingMarker2_to_ground)
        elif aruco_board == 5:
            self.find_aruco_markers(self.front_img, self.aruco_board_landing, self.camera_matrix_front, self.distortion_coefficients_front)
            if self.aruco_board_found:
                self.estimate_marker_pose(aruco_board, self.T_landingMarker3_to_ground)

        self.aruco_marker_found_pub.publish(self.aruco_board_found)

if __name__ == "__main__":
    node = marker_detection()
