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
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Float64, Bool
from mavlink_msgs.msg import mavlink_lora_aruco

from tracking import*
from graphics_plot import graphics_plot

class marker_detection:

    def __init__(self):

        #Init ROS node
        rospy.init_node('marker_detection')

        self.graphics = graphics_plot()

        #Init data test plotting
        self.plot_data = False
        self.plot_aruco_pos_x = []
        self.plot_aruco_pos_y = []
        self.plot_aruco_pos_z = []

        self.plot_aruco_pos_kf_x = []
        self.plot_aruco_pos_kf_y = []
        self.plot_aruco_pos_kf_z = []

        self.cycle_time = (1./20.)
        self.plot_timer = int(((10)/self.cycle_time)) #Set to run 10 seconds before plot

        self.plot_time = []

        self.enable_aruco_detection = False
        self.draw_markers = False
        self.draw_marker_axis = False
        
        #Init data textfile
        data = Path('../../../../data/aruco_pos_kf/data.txt')
        if not data.is_file:
            data = open('../../../../data/aruco_pos_kf/data.txt','r+')
            data.truncate(0)
            data.close
        else:
            data = open('../../../../data/aruco_pos_kf/data.txt','w+')
            data.close()

        
        #Init Kalman filters
        self.kf_x = kalman_filter(self.cycle_time)
        self.kf_y = kalman_filter(self.cycle_time)
        self.kf_z = kalman_filter(self.cycle_time)
        
        self.kf_roll = kalman_filter(self.cycle_time)
        self.kf_pitch = kalman_filter(self.cycle_time)
        self.kf_yaw = kalman_filter(self.cycle_time)

        self.time = 0.0
        
        #Transformation matrix from drone to camera
        self.T_drone_camera = euler_matrix(-np.pi/2, np.pi/2,0,'rxyz')

        #Local drone pose
        self.local_position = PoseStamped()
        self.aruco_pose = PoseStamped()
        self.aruco_pose_without_kf = PoseStamped()

        self.find_aruco_board = True

        self.aruco_ids = []
        
        #Subscribers
        rospy.Subscriber("/mono_cam/image_raw", Image, self.find_aruco_markers)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_position_callback)
        rospy.Subscriber('/onboard/aruco_ids', mavlink_lora_aruco, self.aruco_ids_callback)
        rospy.Subscriber('/onboard/enable_aruco_detection', Bool, self.enable_aruco_detection_callback)

        #Publishers
        self.aruco_marker_image_pub = rospy.Publisher('/onboard/aruco_marker_image', Image, queue_size=1)
        self.aruco_marker_pose_pub = rospy.Publisher('/onboard/aruco_marker_pose', PoseStamped, queue_size=1)
        self.aruco_marker_found_pub = rospy.Publisher('/onboard/aruco_marker_found', Bool, queue_size=1)

        #Initiate aruco detection (Intinsic and extrinsic camera coefficients can be found in sdu_mono_cam model)
        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_1000)
        self.aruco_board = cv2.aruco.GridBoard_create(markersX=4, markersY=2, markerLength=0.2, markerSeparation=0.02, dictionary=self.dictionary,firstMarker=200)
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.camera_matrix = np.array([[1108.765426, 0, (1280/2)], [0, 1108.765426, (720/2)], [0, 0, 1]], dtype=np.float)
        self.distortion_coefficients = np.array([[0, 0, 0, 0]], dtype=np.float)
        self.marker_pose = []
        self.bridge = CvBridge()
        
        rospy.Timer(rospy.Duration(self.cycle_time), self.timer_callback)
        rospy.spin()

    def local_position_callback(self, data):
        self.local_position = data
 
    def aruco_ids_callback(self, data):
        self.aruco_ids = data.elements

    def enable_aruco_detection_callback(self,data):
        self.enable_aruco_detection = data

    def find_aruco_markers(self,img):
       
        #Load in the marker image
        try:
            cv_img = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            print(e)
            
        image_markers = cv_img
        
        #Detect aruco markers in the image
        marker_corners, marker_ids, rejected_candidates = cv2.aruco.detectMarkers(cv_img, self.dictionary, parameters=self.parameters)
        
        marker_pose = []
        if len(marker_corners) > 0:
            
            #Estimate markers pose
            if not self.find_aruco_board:
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(marker_corners, 0.2, self.camera_matrix, self.distortion_coefficients)
            else:
                rvec, tvec, _ = cv2.aruco.estimatePoseBoard(marker_corners, marker_ids, self.aruco_board, self.camera_matrix, self.distortion_coefficients)
                #corners, ids, rejectedImgPoints, recoveredIds = aruco.refineDetectedMarkers(cv_image,self.aruco_board,marker_corners, marker_ids,self.camera_matrix,self.distortion_coefficients)
            #Draw markers
            if self.draw_markers:
                image_markers = cv2.aruco.drawDetectedMarkers(cv_img, marker_corners, marker_ids)
            
            #Find marker pose
            for (_rvec,_tvec,marker_id) in zip(rvec,tvec,marker_ids):
                
                marker_id_pose = []
                pose = PoseStamped()
                
                pose.pose.position.x = _tvec[0][0]
                pose.pose.position.y = _tvec[0][1]
                pose.pose.position.z = _tvec[0][2]
                pose.pose.orientation = quaternion_from_euler(*self.rodrigues_to_euler_angles(_rvec))

                marker_id_pose.append(marker_id)
                marker_id_pose.append(pose)
                marker_pose.append(marker_id_pose)

                #Draw detected axis of markers
                if self.draw_marker_axis:
                    image_markers = cv2.aruco.drawAxis(image_markers, self.camera_matrix, self.distortion_coefficients, _rvec, _tvec, 0.1)

        self.marker_pose = marker_pose
        img = self.bridge.cv2_to_imgmsg(image_markers,"bgr8")
        self.aruco_marker_image_pub.publish(img)

    def estimate_marker_pose(self):

        
        if not len(self.aruco_ids):
            return

        for pose in self.marker_pose:
            
            if self.find_aruco_board or pose[0] == self.aruco_ids[0]:
                
                #Transformation matrix from camera to ArUco marker
                r = quaternion_matrix(pose[1].pose.orientation)
                T_camera_marker = np.linalg.inv(r)
                t = np.array([pose[1].pose.position.x, pose[1].pose.position.y, pose[1].pose.position.z, 1])
                t = np.dot(t,T_camera_marker) #-

                T_camera_marker[0][3] =  t[0]
                T_camera_marker[1][3] =  t[1]
                T_camera_marker[2][3] =  t[2]

                #Transformation matrix from drone to camera
                T_drone_marker = np.dot(self.T_drone_camera, T_camera_marker)
                
                #Update KF
                euler = euler_from_quaternion(pose[1].pose.orientation,'rxyz')
                #euler = euler_from_matrix(T_drone_marker,'rxyz')

                self.kf_x.get_measurement(T_drone_marker[0][3])
                self.kf_y.get_measurement(T_drone_marker[1][3])
                self.kf_z.get_measurement(T_drone_marker[2][3])
                
                self.kf_roll.get_measurement(np.mod((euler[0]+np.pi),2*np.pi) - np.pi)
                self.kf_pitch.get_measurement(np.mod((euler[1]+np.pi),2*np.pi) - np.pi)
                self.kf_yaw.get_measurement(np.mod((euler[2]+np.pi),2*np.pi) - np.pi) 
                 
                self.aruco_pose.pose.position.x = self.kf_x.tracker.x[0] #-
                self.aruco_pose.pose.position.y = -self.kf_y.tracker.x[0]
                self.aruco_pose.pose.position.z = self.kf_z.tracker.x[0] #-

                self.aruco_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, (-self.kf_pitch.tracker.x[0]),'rxyz'))
                
                #print("Roll: " + str(self.kf_roll.tracker.x[0]) + " Pitch: " + str(self.kf_pitch.tracker.x[0]+np.pi/2) + " Yaw: " + str(self.kf_yaw.tracker.x[0]))
                #print("Roll: " + str(self.kf_roll.tracker.x[0]) + " Pitch: " + str(self.kf_pitch.tracker.x[0]) + " Yaw: " + str(self.kf_yaw.tracker.x[0]))
                self.aruco_marker_pose_pub.publish(self.aruco_pose)

                if self.plot_data and self.plot_timer > 0:
                    
                    self.plot_timer -= 1

                    x = T_drone_marker[0][3]
                    y = T_drone_marker[1][3]
                    z = T_drone_marker[2][3]
                    kf_x = self.kf_x.tracker.x[0]
                    kf_y = self.kf_y.tracker.x[0]
                    kf_z = self.kf_z.tracker.x[0]

                    self.write_aruco_pos(x, y, z, kf_x, kf_y, kf_z, self.time)
                    self.time = self.time + self.cycle_time
                #print "Marker id: {} Ori: {} x: {} y: {} z: {} \n".format(pose[0],euler,self.aruco_pose.pose.position.x,self.aruco_pose.pose.position.y,self.aruco_pose.pose.position.z)
                self.aruco_marker_found_pub.publish(True)
            else:
                self.aruco_marker_found_pub.publish(False)

    def write_aruco_pos(self, x, y, z, kf_x, kf_y, kf_z, time):
        
        data = open('../../../../data/aruco_pos_kf/data.txt','a')
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
        #if self.enable_aruco_detection:
        self.estimate_marker_pose()
        

if __name__ == "__main__":
    #rospy.init_node('marker_detection', anonymous=True)
    node = marker_detection()
    node.print_aruco_board()

