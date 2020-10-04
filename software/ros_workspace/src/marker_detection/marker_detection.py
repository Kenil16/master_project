#!/usr/bin/python

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
        self.plot_data = True 
        self.plot_aruco_pos_x = []
        self.plot_aruco_pos_y = []
        self.plot_aruco_pos_z = []

        self.plot_aruco_pos_kf_x = []
        self.plot_aruco_pos_kf_y = []
        self.plot_aruco_pos_kf_z = []

        self.cycle_time = (1./50.)
        self.plot_timer = int(((30)/self.cycle_time)) #Set to run 10 seconds before plot

        self.plot_time = []
        
        #Init Kalman filters
        self.kf_x = kalman_filter(self.cycle_time)
        self.kf_y = kalman_filter(self.cycle_time)
        self.kf_z = kalman_filter(self.cycle_time)
        
        self.kf_roll = kalman_filter(self.cycle_time)
        self.kf_pitch = kalman_filter(self.cycle_time)
        self.kf_yaw = kalman_filter(self.cycle_time)

        #Local drone pose
        self.local_position = PoseStamped()
        self.aruco_pose = PoseStamped()
        self.aruco_pose_without_kf = PoseStamped()

        self.aruco_ids = mavlink_lora_aruco()
        
        #Subscribers
        self.image_sub = rospy.Subscriber("/mono_cam/image_raw", Image, self.find_aruco_markers)
        self.local_pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_position_callback)
        self.aruco_ids_sub = rospy.Subscriber('aruco_ids', mavlink_lora_aruco, self.aruco_ids_callback)

        #Publishers
        self.aruco_marker_image_pub = rospy.Publisher("aruco_marker_image", Image, queue_size=1)
        self.aruco_marker_pose_pub = rospy.Publisher("/mavros/vision_pose/pose", PoseStamped, queue_size=1)
        self.aruco_marker_found_pub = rospy.Publisher("aruco_marker_found", Bool, queue_size=1)

        #Initiate aruco detection (Intinsic and extrinsic camera coefficients can be found in sdu_mono_cam model)
        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_1000)
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
        self.aruco_ids.elements = data.elements
        
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
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(marker_corners, 0.2, self.camera_matrix, self.distortion_coefficients)

            #Draw markers
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
                image_markers = cv2.aruco.drawAxis(image_markers, self.camera_matrix, self.distortion_coefficients, _rvec, _tvec, 0.1)

        self.marker_pose = marker_pose
        img = self.bridge.cv2_to_imgmsg(image_markers,"bgr8")
        self.aruco_marker_image_pub.publish(img)

    def estimate_marker_pose(self, marker_id):

        if not marker_id:
            return
        else:
            _id = marker_id.elements

        for pose in self.marker_pose:
            
            if pose[0] == _id:
                
                """
                #Transformation matrix from world to drone
                x = self.local_position.pose.orientation.x
                y = self.local_position.pose.orientation.y
                z = self.local_position.pose.orientation.z
                w = self.local_position.pose.orientation.w
                
                T_world_drone = quaternion_matrix(np.array([x,y,z,w]))
                 
                T_world_drone[0][3] = self.local_position.pose.position.x + 20 #Drone shiftet 20 meters wrt the world origin
                T_world_drone[1][3] = self.local_position.pose.position.y
                T_world_drone[2][3] = self.local_position.pose.position.z
                """

                #Transformation matrix from drone to camera
                T_drone_camera = euler_matrix(-np.pi/2, np.pi/2,0,'rxyz')

                #Transformation matrix from camera to ArUco marker
                r = quaternion_matrix(pose[1].pose.orientation)
                T_camera_marker = np.linalg.inv(r)
                t = np.array([pose[1].pose.position.x, pose[1].pose.position.y, pose[1].pose.position.z, 1])
                t = -np.dot(t,T_camera_marker)

                T_camera_marker = euler_matrix(0,0,0,'rxyz')
                T_camera_marker[0][3] = t[0]
                T_camera_marker[1][3] = t[1]
                T_camera_marker[2][3] = t[2]

                #Transformation matrix from drone to camera
                T_drone_marker = np.dot(T_drone_camera, T_camera_marker)
                
                #T_drone_marker = np.dot(np.dot(T_world_drone,T_drone_camera), T_camera_marker)

                #Get euler and update Kalman filter
                euler = euler_from_matrix(T_drone_marker,'rxyz')

                self.kf_x.get_measurement(T_drone_marker[0][3])
                self.kf_y.get_measurement(T_drone_marker[1][3])
                self.kf_z.get_measurement(T_drone_marker[2][3])
                
                self.kf_roll.get_measurement(euler[0])
                self.kf_pitch.get_measurement(euler[1])
                self.kf_yaw.get_measurement(euler[2])
                 
                self.aruco_pose.pose.position.x = self.kf_x.tracker.x[0]
                self.aruco_pose.pose.position.y = self.kf_y.tracker.x[0]
                self.aruco_pose.pose.position.z = self.kf_z.tracker.x[0]

                self.aruco_pose.pose.orientation = Quaternion(*quaternion_from_euler(self.kf_roll.tracker.x[0],self.kf_pitch.tracker.x[0],self.kf_yaw.tracker.x[0],'rxyz'))
                
                self.aruco_marker_pose_pub.publish(self.aruco_pose)

                #Plot position data (Just for testing)
                if self.plot_data and self.plot_timer > 0:
                    self.plot_timer -= 1

                    self.plot_aruco_pos_x.append(T_drone_marker[0][3])
                    self.plot_aruco_pos_y.append(T_drone_marker[1][3])
                    self.plot_aruco_pos_z.append(T_drone_marker[2][3])

                    self.plot_aruco_pos_kf_x.append(self.kf_x.tracker.x[0])
                    self.plot_aruco_pos_kf_y.append(self.kf_y.tracker.x[0])
                    self.plot_aruco_pos_kf_z.append(self.kf_z.tracker.x[0])
                    
                    if not self.plot_time:
                        self.plot_time.append(self.cycle_time)
                    else:
                        self.plot_time.append(self.cycle_time + self.plot_time[-1])

                    if not self.plot_timer:
                        self.graphics.plot_kf_data(self.plot_aruco_pos_x,self.plot_aruco_pos_kf_x,self.plot_time,'Estimated ArUco marker position','Time [s]','x [m]','../../../../data/plots/kf_pos_x.png')
                        self.graphics.plot_kf_data(self.plot_aruco_pos_y,self.plot_aruco_pos_kf_y,self.plot_time,'Estimated ArUco marker position','Time [s]','y [m]','../../../../data/plots/kf_pos_y.png')
                        self.graphics.plot_kf_data(self.plot_aruco_pos_z,self.plot_aruco_pos_kf_z,self.plot_time,'Estimated ArUco marker position','Time [s]','z [m]','../../../../data/plots/kf_pos_z.png')
                print "Marker id: {} Ori: {} x: {} y: {} z: {} \n".format(pose[0],euler,self.aruco_pose.pose.position.x,self.aruco_pose.pose.position.y,self.aruco_pose.pose.position.z)
                self.aruco_marker_found_pub.publish(True)
            else:
                self.aruco_marker_found_pub.publish(False)
            

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
        self.estimate_marker_pose(self.aruco_ids)
        

if __name__ == "__main__":
    #rospy.init_node('marker_detection', anonymous=True)
    node = marker_detection()

