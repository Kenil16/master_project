#!/usr/bin/python

from pathlib import Path
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_matrix, quaternion_from_euler, euler_from_matrix, euler_from_quaternion, identity_matrix
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Bool, Int8, Float32
from pandas import Series
from log_data import*

class marker_detection:

    def __init__(self):

        #Variables for writing data logs for aruco pose and ground truth
        self.log_data = log_data()
        self.write_data_log = True

        #Variables for aruco pose estimation 
        self.aruco_board_found = False
        self.center_of_board = PoseStamped()
        self.marker_pose = PoseStamped()
        self.aruco_marker_pose_stable = False

        #Variable for image output 
        self.ros_img = Image()

        #Variables for rolling average 
        self.write_aruco_pose_estimate = []
        self.aruco_pose_estimate = []
        self.write_iterations_rolling_average = 100
        self.iterations_rolling_average = 5
        self.write_rolling_average = True
        self.max_std_rolling_average = 0.01
        self.time = 0.0
        self.dis_to_GPS2Vision_marker = 0.0

        #Variables for aruco pose estimate before transformations
        self.aruco_pos = [0., 0., 0.]
        self.aruco_angle = None

        #Transformation matrix from gps to vision marker to the ground wrt the drone
        self.T_gps2visionMarker_to_ground = identity_matrix()
        self.T_gps2visionMarker_to_ground = euler_matrix(np.pi/2, 0, 0, 'rxyz')
        self.T_gps2visionMarker_to_ground[0][3] = 3.15 
        self.T_gps2visionMarker_to_ground[1][3] = 2.99
        self.T_gps2visionMarker_to_ground[2][3] = 2.57
        
        #Transformation matrix from landing marker 1 to the ground wrt the drone
        self.T_landingMarker1_to_ground = identity_matrix()
        self.T_landingMarker1_to_ground = euler_matrix(np.pi/2, 0, 0,'rxyz')
        self.T_landingMarker1_to_ground[0][3] = 0.15 #3.4 - 3.3 + 0.05  = 0.15 (Ground marker Placement in gazebo and offset to landing board)
        self.T_landingMarker1_to_ground[1][3] = 8.10 #7.4 + (4.4-3.7) = 8.10 (Size of ground marker and placement in gazebo)
        self.T_landingMarker1_to_ground[2][3] = 0.15 #1 - 1.7/2 = 0.15 (Placement in gazebo and size of marker board)
        
        #Transformation matrix from landing marker 2 to the ground wrt the drone
        self.T_landingMarker2_to_ground = identity_matrix()
        self.T_landingMarker2_to_ground = euler_matrix(np.pi/2, 0, 0,'rxyz')
        self.T_landingMarker2_to_ground[0][3] = 3.425
        self.T_landingMarker2_to_ground[1][3] = 8.10
        self.T_landingMarker2_to_ground[2][3] = 0.125
        
        #Transformation matrix from landing marker 3 to the ground wrt the drone
        self.T_landingMarker3_to_ground = identity_matrix()
        self.T_landingMarker3_to_ground = euler_matrix(np.pi/2, 0, 0,'rxyz')
        self.T_landingMarker3_to_ground[0][3] = 6.475
        self.T_landingMarker3_to_ground[1][3] = 8.10
        self.T_landingMarker3_to_ground[2][3] = 0.125
        
        #Initiate aruco detection (Intinsic and extrinsic camera coefficients can be found in sdu_mono_cam model)
        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_1000)
        
        self.aruco_board_gps2vision = cv2.aruco.GridBoard_create(markersX=4, markersY=2, markerLength=0.2, markerSeparation=0.1, dictionary=self.dictionary,firstMarker=100)
        self.aruco_board_vision = cv2.aruco.GridBoard_create(markersX=25, markersY=25, markerLength=0.2, markerSeparation=0.1, dictionary=self.dictionary,firstMarker=200)
        self.aruco_board_landing1 = cv2.aruco.GridBoard_create(markersX=2, markersY=6, markerLength=0.2, markerSeparation=0.1, dictionary=self.dictionary,firstMarker=1)
        self.aruco_board_landing2 = cv2.aruco.GridBoard_create(markersX=4, markersY=12, markerLength=0.1, markerSeparation=0.05, dictionary=self.dictionary,firstMarker=1)
        self.aruco_board_landing3 = cv2.aruco.GridBoard_create(markersX=8, markersY=12, markerLength=0.1, markerSeparation=0.05, dictionary=self.dictionary,firstMarker=1)
        
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        
        #Bottom camera
        self.camera_matrix_bottom = np.array([[623.680552, 0, (720/2)], [0, 623.680552, (480/2)], [0, 0, 1]], dtype=np.float)
        #self.camera_matrix_bottom = np.array([[415.787035, 0, (480/2)], [0, 415.787035, (240/2)], [0, 0, 1]], dtype=np.float)
        self.distortion_coefficients_bottom = np.array([[0, 0, 0, 0]], dtype=np.float)
        
        #Front camera
        self.camera_matrix_front = np.array([[623.680552, 0, (720/2)], [0, 623.680552, (480/2)], [0, 0, 1]], dtype=np.float)
        self.distortion_coefficients_front = np.array([[0, 0, 0, 0]], dtype=np.float)
        
        self.bridge = CvBridge()
        self.image_id = 0
        
    def find_aruco_markers(self, img, aruco_board, camera_matrix, distortion_coefficients, draw_markers, calculate_center_of_board = False):
       
        #Only proceed if image is giving and transform image from ros to CV2
        cv_img = None
        if img == None:
            return
        try:
            cv_img = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            print(e)

        image_markers = cv_img
        
        #Detect aruco markers in the image
        marker_corners, marker_ids, rejected_candidates = cv2.aruco.detectMarkers(cv_img, self.dictionary, parameters=self.parameters, cameraMatrix=camera_matrix, distCoeff=distortion_coefficients)
        
        if len(marker_corners) > 0:

            retval, _rvec, _tvec = cv2.aruco.estimatePoseBoard(marker_corners, marker_ids, aruco_board, camera_matrix, distortion_coefficients, rvec=None, tvec=None)
            
            if retval:

                self.aruco_board_found = True 
                self.aruco_pos = [_tvec[0][0], _tvec[1][0], _tvec[2][0]]
                self.aruco_angle, _ = cv2.Rodrigues(_rvec)

                #Draw marker and detected axis if wanted
                if draw_markers:
                    image_markers = cv2.aruco.drawDetectedMarkers(cv_img, marker_corners, marker_ids)
                    image_markers = cv2.aruco.drawAxis(image_markers, camera_matrix, distortion_coefficients, _rvec, _tvec, 0.1)

                #This is only used in GPS2Vision if pose estimate is very unstable
                if calculate_center_of_board == True:
                    self.center_of_board = self.calculate_center_of_board(marker_ids, marker_corners)

        else:
            self.aruco_board_found = False

        if draw_markers:
            self.ros_img = self.bridge.cv2_to_imgmsg(image_markers,"bgr8")

        self.image_id = img.header.seq

    def estimate_marker_pose(self, aruco_board_config, T_front_to_ground = None, ground_truth = None):
        
        r = self.aruco_angle.T
        t = np.array([self.aruco_pos[0], self.aruco_pos[1], self.aruco_pos[2]])
        t = -np.matmul(r,t)

        if aruco_board_config == 2: #If bottom cam is used
            
            T = euler_matrix(0, np.pi, 0, 'rxyz')
            r = np.matmul(r.T, T[:3, :3])
            euler = euler_from_matrix(r,'rxyz')
            q_new = Quaternion(*quaternion_from_euler(euler[0], euler[1], euler[2],'rxyz'))
            
            #Update ArUco marker pose
            self.marker_pose.pose.position.x = t[0]
            self.marker_pose.pose.position.y = t[1]
            self.marker_pose.pose.position.z = t[2] + 0.05 + 0.02 #Cam offset of drone and shift of marker board 
            self.marker_pose.pose.orientation = q_new
        
        else:
            T = euler_matrix(np.pi, 0, 0, 'rxyz')
            r = np.matmul(r.T,T[:3, :3])
            t = np.dot(T_front_to_ground[:3, :3],t)
            euler = euler_from_matrix(r,'rxyz')
            q_new = Quaternion(*quaternion_from_euler(-euler[2], -euler[0], euler[1] + np.deg2rad(90),'rxyz'))
            
            #Update ArUco marker pose
            self.marker_pose.pose.position.x = t[0] + T_front_to_ground[0][3]
            self.marker_pose.pose.position.y = t[1] + T_front_to_ground[1][3]
            self.marker_pose.pose.position.z = t[2] + T_front_to_ground[2][3]
            self.marker_pose.pose.orientation = q_new

        #To set time between estimations 
        self.marker_pose.header.stamp = rospy.Time.now()
        
        #Only used for valification of marker pose estimate 
        #print(euler_from_quaternion([self.marker_pose.pose.orientation.x, self.marker_pose.pose.orientation.y, self.marker_pose.pose.orientation.z, self.marker_pose.pose.orientation.w]))
        #euler = euler_from_matrix(r,'rxyz')
        #print "Ori: {} x: {} y: {} z: {} \n".format(euler, self.marker_pose.pose.position.x, self.marker_pose.pose.position.y, self.marker_pose.pose.position.z)

        #Used for data log of estimated aruco pose vs ground truth
        if self.write_data_log:
            self.log_data.write_marker_detection_data(aruco_board_config, self.marker_pose, ground_truth)
    
    def calculate_rolling_average(self, pose):

        #Get euclidean coordinates
        x = pose.pose.position.x
        y = pose.pose.position.y
        z = pose.pose.position.z
        
        #Get angle in euler coordinates (roll, pitch, yaw)
        euler = euler_from_quaternion([pose.pose.orientation.x,
                                       pose.pose.orientation.y,
                                       pose.pose.orientation.z,
                                       pose.pose.orientation.w])

        #Save data and write to file for analysis if wanted 
        if self.write_rolling_average:

            if len(self.write_aruco_pose_estimate) == self.write_iterations_rolling_average:

                #Just clear file before writing
                self.log_data.write_rolling_average(0,0,0,0,0,0,True)
                
                for i in self.aruco_pose_estimate:
                    self.log_data.write_rolling_average(i[0], i[1], i[2], i[3], i[4], i[5], False)

                self.write_aruco_pose_estimate = []

            else:
                self.write_aruco_pose_estimate.append([x, y, z, euler[0], euler[1], euler[2]])
        
        #Calculate rolling average for the wanted number of iterations of pose
        if len(self.aruco_pose_estimate) == self.iterations_rolling_average:
            #print(self.aruco_pose_estimate)
            x = Series(np.array([item[0] for item in self.aruco_pose_estimate]))
            x_std = x.rolling(5).std()

            y = Series(np.array([item[1] for item in self.aruco_pose_estimate]))
            y_std = y.rolling(5).std()
            
            z = Series(np.array([item[2] for item in self.aruco_pose_estimate]))
            z_std = z.rolling(5).std()
            
            roll = Series(np.array([item[3] for item in self.aruco_pose_estimate]))
            roll_std = roll.rolling(5).std()
            
            pitch = Series(np.array([item[4] for item in self.aruco_pose_estimate]))
            pitch_std = pitch.rolling(5).std()

            yaw = Series(np.array([item[5] for item in self.aruco_pose_estimate]))
            yaw_std = yaw.rolling(5).std()

            if (x_std[4] and y_std[4] and z_std[4] and roll_std[4] and pitch_std[4] and yaw_std[4]) < self.max_std_rolling_average:
                self.aruco_marker_pose_stable = True
            else:
                self.aruco_marker_pose_stable = False

            self.aruco_pose_estimate = []
            self.dis_to_GPS2Vision_marker = np.sqrt((x.rolling(5).mean()[4]-self.T_gps2visionMarker_to_ground[0][3])**2 + 
                                                    (y.rolling(5).mean()[4]-self.T_gps2visionMarker_to_ground[1][3])**2 + 
                                                    (z.rolling(5).mean()[4]-self.T_gps2visionMarker_to_ground[0][3])**2)

        else:
            self.aruco_pose_estimate.append([x, y, z, euler[0], euler[1], euler[2]])

    def calculate_center_of_board(self, marker_ids, marker_corners):
        
        centers_y = []
        centers_x = []
        pose = PoseStamped()
        
        for corner, id_ in zip(marker_corners, marker_ids):
            if id_ > 99 and id_ < 109: #GPS2Vision marker only
                centerX = (corner[0][0][0] + corner[0][1][0] + corner[0][2][0] + corner[0][3][0]) / 4
                centerY = (corner[0][0][1] + corner[0][1][1] + corner[0][2][1] + corner[0][3][1]) / 4
                
                centers_x.append(centerX)
                centers_y.append(centerY)
        
        if len(centers_x) and len(centers_y):
            center_mean_x = sum(centers_x)/len(centers_x)
            center_mean_y = sum(centers_y)/len(centers_y)

            pose.pose.position.x = center_mean_x
            pose.pose.position.y = center_mean_y
        
        return pose

if __name__ == "__main__":
    node = marker_detection()
