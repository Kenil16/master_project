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
from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovariance, PoseWithCovarianceStamped
from std_msgs.msg import Float64, Bool, Int8
from mavlink_msgs.msg import mavlink_lora_aruco

from sensor_fusion import*
#from graphics_plot import graphics_plot

import time

class marker_detection:

    def __init__(self):

        #Init ROS node
        rospy.init_node('marker_detection')

        #Init data test plotting
        self.plot_data = True
        self.cycle_time = (1./10.) #1./40.
        self.plot_timer = int(((20)/self.cycle_time)) #Set to run 10 seconds before plot

        self.plot_time = []

        #self.enable_aruco_detection = False
        self.draw_markers = True
        self.draw_marker_axis = True
        self.aruco_board_found = False

        self.bottom_img = None 
        self.front_img = None

        #Init data textfile
        data = Path('../../../../data/estimate_aruco_vs_kf_posedata.txt')
        if not data.is_file:
            data = open('../../../../data/estimate_aruco_vs_kf_pose/data.txt','r+')
            data.truncate(0)
            data.close
        else:
            data = open('../../../../data/estimate_aruco_vs_kf_pose/data.txt','w+')
            data.close()

        self.time = 0.0
        
        #Transformation matrix from drone to camera
        self.T_drone_camera_front = euler_matrix(np.pi/2, -np.pi/2, 0, 'rxyz')
        self.T_drone_camera_bottom = euler_matrix(np.pi, np.pi, 0, 'rxyz')

        #Transformation matrix from gps to vision marker to the ground wrt the drone
        self.T_gps2visionMarker_to_ground = identity_matrix()
        self.T_gps2visionMarker_to_ground = euler_matrix(0, 0, -np.pi/2,'rxyz')
        self.T_gps2visionMarker_to_ground[0][3] = 3.036
        self.T_gps2visionMarker_to_ground[1][3] = 3.023
        self.T_gps2visionMarker_to_ground[2][3] = -2.36

        #Transformation matrix from landing marker 1 to the ground wrt the drone
        self.T_landingMarker1_to_ground = identity_matrix()
        self.T_landingMarker1_to_ground = euler_matrix(0, 0, -np.pi/2,'rxyz')
        self.T_landingMarker1_to_ground[0][3] = 0.18 
        self.T_landingMarker1_to_ground[1][3] = 8.64
        self.T_landingMarker1_to_ground[2][3] = -0.08
        
        #Transformation matrix from landing marker 2 to the ground wrt the drone
        self.T_landingMarker2_to_ground = identity_matrix()
        self.T_landingMarker2_to_ground = euler_matrix(0, 0, -np.pi/2,'rxyz')
        self.T_landingMarker2_to_ground[0][3] = 3.53
        self.T_landingMarker2_to_ground[1][3] = 8.64
        self.T_landingMarker2_to_ground[2][3] = -0.08
        
        #Transformation matrix from landing marker 3 to the ground wrt the drone
        self.T_landingMarker3_to_ground = identity_matrix()
        self.T_landingMarker3_to_ground = euler_matrix(0, 0, -np.pi/2,'rxyz')
        self.T_landingMarker3_to_ground[0][3] = 6.88
        self.T_landingMarker3_to_ground[1][3] = 8.64
        self.T_landingMarker3_to_ground[2][3] = -0.08
        
        #Local drone pose
        self.local_position = PoseStamped()
        self.aruco_pose = PoseStamped()
        self.aruco_pose_without_kf = PoseStamped()

        #self.use_bottom_cam = False
        self.aruco_board = 2
        
        #Subscribers
        rospy.Subscriber("/mono_cam_bottom/image_raw", Image, self.bottom_img_callback)
        rospy.Subscriber("/mono_cam_front/image_raw", Image, self.front_img_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_position_callback)
        #rospy.Subscriber('/onboard/aruco_ids', mavlink_lora_aruco, self.aruco_ids_callback)
        
        #Publishers
        self.aruco_marker_image_pub = rospy.Publisher('/onboard/aruco_marker_image', Image, queue_size=1)
        self.aruco_marker_pose_pub = rospy.Publisher('/onboard/aruco_marker_pose', PoseStamped, queue_size=1)
        self.aruco_marker_found_pub = rospy.Publisher('/onboard/aruco_board_found', Bool, queue_size=1)
        #self.aruco_marker_pose_cov = rospy.Publisher('/onboard/aruco_marker_pose_cov', PoseWithCovarianceStamped, queue_size=1)

        #Initiate aruco detection (Intinsic and extrinsic camera coefficients can be found in sdu_mono_cam model)
        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_1000)
        
        self.aruco_board_gps2vision = cv2.aruco.GridBoard_create(markersX=4, markersY=2, markerLength=0.2, markerSeparation=0.08, dictionary=self.dictionary,firstMarker=100)
        self.aruco_board_vision = cv2.aruco.GridBoard_create(markersX=25, markersY=25, markerLength=0.2, markerSeparation=0.1, dictionary=self.dictionary,firstMarker=200)
        self.aruco_board_landing = cv2.aruco.GridBoard_create(markersX=2, markersY=4, markerLength=0.2, markerSeparation=0.1, dictionary=self.dictionary,firstMarker=1)
        
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

    def local_position_callback(self, data):
        self.local_position = data
 
    def aruco_board_callback(self, data):
        self.aruco_board = data.data

    def bottom_img_callback(self, data):
        self.bottom_img = data

    def front_img_callback(self, data):
        self.front_img = data
    
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
                self.aruco_marker_found_pub.publish(True)

                pose.pose.position.x = _tvec[0]
                pose.pose.position.y = _tvec[1]
                pose.pose.position.z = _tvec[2]
                pose.pose.orientation = quaternion_from_euler(*self.rodrigues_to_euler_angles(_rvec))

                #Draw detected axis of markers
                if self.draw_marker_axis:
                    image_markers = cv2.aruco.drawAxis(image_markers, camera_matrix, distortion_coefficients, _rvec, _tvec, 0.1)
            else:
                self.aruco_marker_found_pub.publish(False)
        
        self.marker_pose = pose
        img = self.bridge.cv2_to_imgmsg(image_markers,"bgr8")
        self.aruco_marker_image_pub.publish(img)

    def estimate_marker_pose(self, cam, T_drone_camera, T_front_to_ground = None):

        #Transformation matrix from camera to ArUco marker
        r = quaternion_matrix(self.marker_pose.pose.orientation)
        
        #T_camera_marker = np.linalg.inv(r)
        T_camera_marker = r.T
        
        t = np.array([self.marker_pose.pose.position.x, self.marker_pose.pose.position.y, self.marker_pose.pose.position.z, 1])
        t = -np.dot(T_camera_marker,t)

        #New 3d vector from drone to marker using rotation from drone to camera and vector from cam to marker 
        T_drone_marker = np.dot(T_drone_camera ,t)
        
        if not cam == 'bottom':
            T_drone_marker = np.dot(T_front_to_ground, T_drone_marker)
        
        #Get euler from rotation matrix
        euler = euler_from_matrix(r,'rxyz')

        #Update ArUco marker position based on Kalman filter
        self.aruco_pose.pose.position.x = T_drone_marker[0][0] #self.kf_pos.tracker.x[0][0]
        self.aruco_pose.pose.position.y = T_drone_marker[1][0] #self.kf_pos.tracker.x[3][0]
        self.aruco_pose.pose.position.z = T_drone_marker[2][0] #self.kf_pos.tracker.x[6][0]

        #Because roll angle is offset 180 degress with respect to the camera 
        euler = list(euler)
        if euler[0] > 0:
            euler[0] = euler[0] - np.pi
        else:
            euler[0] = euler[0] + np.pi
        euler = tuple(euler)
        
        if cam == 'bottom':
            self.aruco_pose.pose.orientation = Quaternion(*quaternion_from_euler(-euler[0], -euler[1], -euler[2],'rxyz'))
        else:
            self.aruco_pose.pose.orientation = Quaternion(*quaternion_from_euler(-euler[0], -euler[2], -euler[1] - 1.5708,'rxyz'))#Because yaw is offset -90 to front marker

        """
        test = PoseWithCovarianceStamped()
        pose_cov = PoseWithCovariance()
        pose_cov.pose.position = self.aruco_pose.pose.position
        pose_cov.pose.orientation = self.aruco_pose.pose.orientation
        pose_cov.covariance[0] = 500.5
        pose_cov.covariance[7] = 500.5
        pose_cov.covariance[14] = 500.5
        pose_cov.covariance[21] = 500.5
        pose_cov.covariance[28] = 500.5
        pose_cov.covariance[34] = 500.5
        test.pose.pose = pose_cov.pose
        test.pose.covariance = pose_cov.covariance
        self.aruco_marker_pose_cov.publish(test)
        """

        #print( euler_from_quaternion([self.aruco_pose.pose.orientation.x,self.aruco_pose.pose.orientation.y,self.aruco_pose.pose.orientation.z,self.aruco_pose.pose.orientation.w]))
        #print "Ori: {} x: {} y: {} z: {} \n".format(euler,self.aruco_pose.pose.position.x,self.aruco_pose.pose.position.y,self.aruco_pose.pose.position.z)
        #self.aruco_marker_found_pub.publish(True)
        self.aruco_marker_pose_pub.publish(self.aruco_pose)
    
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
    
    def timer_callback(self,event):
        
        #print(self.aruco_board)
        #Either go from GPS to vision, navigate through ground markers or perform a landing (1, 2, 3) all using vision. Zero means do not detect markers
        if self.aruco_board == 1:
            self.find_aruco_markers(self.front_img, self.aruco_board_gps2vision, self.camera_matrix_front, self.distortion_coefficients_front)
            if self.aruco_board_found:
                self.estimate_marker_pose('front', self.T_drone_camera_front, self.T_gps2visionMarker_to_ground)
        elif self.aruco_board == 2:
            self.find_aruco_markers(self.bottom_img, self.aruco_board_vision, self.camera_matrix_bottom, self.distortion_coefficients_bottom)
            if self.aruco_board_found:
                self.estimate_marker_pose('bottom', self.T_drone_camera_bottom)
        elif self.aruco_board == 3:
            self.find_aruco_markers(self.front_img, self.aruco_board_landing, self.camera_matrix_front, self.distortion_coefficients_front)
            if self.aruco_board_found:
                self.estimate_marker_pose('front', self.T_drone_camera_front, self.T_landingMarker1_to_ground)
        elif self.aruco_board == 4:
            self.find_aruco_markers(self.front_img, self.aruco_board_landing, self.camera_matrix_front, self.distortion_coefficients_front)
            if self.aruco_board_found:
                self.estimate_marker_pose('front', self.T_drone_camera_front, self.T_landingMarker2_to_ground)
        elif self.aruco_board == 5:
            self.find_aruco_markers(self.front_img, self.aruco_board_landing, self.camera_matrix_front, self.distortion_coefficients_front)
            if self.aruco_board_found:
                self.estimate_marker_pose('front', self.T_drone_camera_front, self.T_landingMarker3_to_ground)

if __name__ == "__main__":
    node = marker_detection()
