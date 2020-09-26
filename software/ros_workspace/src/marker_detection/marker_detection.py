#!/usr/bin/python

import cv2
import numpy as np
import rospy
from os import system
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_matrix,quaternion_from_matrix
import tf
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Float64, Bool

class marker_detection:

    def __init__(self):

        rospy.init_node('marker_detection')
        
        #Local drone pose
        self.local_position = PoseStamped()
        self.aruco_pose = PoseStamped()
        
        #Subscribers
        self.image_sub = rospy.Subscriber("/mono_cam/image_raw", Image, self.find_aruco_markers)
        self.local_pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_position_callback)

        #Publishers
        self.aruco_marker_image_pub = rospy.Publisher("aruco_marker_image", Image, queue_size=1)
        self.aruco_marker_pose_pub = rospy.Publisher("marker_pose", PoseStamped, queue_size=1)
        
        #self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        #self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        #Initiate aruco detection (Intinsic and extrinsic camera coefficients can be found in sdu_mono_cam model)
        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_1000)
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.camera_matrix = np.array([[1108.765426, 0, (1280/2)], [0, 1108.765426, (720/2)], [0, 0, 1]], dtype=np.float)
        self.distortion_coefficients = np.array([[0, 0, 0, 0]], dtype=np.float)
        self.marker_pose = []
        self.bridge = CvBridge()
        
        rospy.Timer(rospy.Duration(2), self.timer_callback)
        rospy.spin()

    def local_position_callback(self, data):
        self.local_position = data
        
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
                image_markers = cv2.aruco.drawAxis(image_markers, self.camera_matrix, self.distortion_coefficients, _rvec, _tvec, 0.05)

        self.marker_pose = marker_pose
        img = self.bridge.cv2_to_imgmsg(image_markers,"bgr8")
        self.aruco_marker_image_pub.publish(img)

    def estimate_marker_pose(self, marker_id):

        for pose in self.marker_pose:
            
            if pose[0] == marker_id:
                
                #Transformation matrix because the drone if shifted 180 (3.16 radi) degress of yaw relative to world
                x = self.local_position.pose.orientation.x
                y = self.local_position.pose.orientation.y
                z = self.local_position.pose.orientation.z
                w = self.local_position.pose.orientation.w
            
                T_world_drone = quaternion_matrix([x,y,z,w])
                
                T_world_drone[0][3] = self.local_position.pose.position.x
                T_world_drone[1][3] = self.local_position.pose.position.y
                T_world_drone[2][3] = self.local_position.pose.position.z
            
                x = pose[1].pose.orientation[0]
                y = pose[1].pose.orientation[1]
                z = pose[1].pose.orientation[2]
                w = pose[1].pose.orientation[3]
                
                T_drone_aruco = quaternion_matrix([x,y,z,w])
                #print(euler_from_quaternion([x,y,z,w]))
                
            
                T_drone_aruco[0][3] = pose[1].pose.position.x
                T_drone_aruco[1][3] = pose[1].pose.position.y
                T_drone_aruco[2][3] = pose[1].pose.position.z
                

                #T_inverse = np.linalg.inv(T_drone_aruco)
                #T = -T_inverse.dot(np.array([pose[1].pose.position.x,pose[1].pose.position.y,pose[1].pose.position.z,1]))
                #T = T_world_drone.dot(T_drone_aruco)
                print(T_drone_aruco)
                """
                
                self.aruco_pose.pose.position.x = T[0][3]
                self.aruco_pose.pose.position.y = T[1][3]
                self.aruco_pose.pose.position.z = T[2][3]
                #self.aruco_pose.pose.orientation =  quaternion_from_matrix(T)
                """
                self.aruco_marker_pose_pub.publish(self.aruco_pose)
                
                #print("T_world_world: " + T_world_drone)
                #print(T)
                
                #print "Marker id: {} Position: {} Orientation: {} \n".format(pose[0],pose[1].pose.position,eular_angles)


            

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
    
    
    def eulerAnglesToRotationMatrix(self,theta):
        
        R_x = np.array([[1,0,0],
                        [0,math.cos(theta[0]),-math.sin(theta[0]) ],
                        [0,math.sin(theta[0]),math.cos(theta[0])]])
        
        R_y = np.array([[math.cos(theta[1]),0,math.sin(theta[1])],
                        [0,1,0],
                        [-math.sin(theta[1]),0,math.cos(theta[1])]])
        
        R_z = np.array([[math.cos(theta[2]),-math.sin(theta[2]),0],
                        [math.sin(theta[2]),math.cos(theta[2]),0],
                        [0,0,1]])
        
        R = np.dot(R_z, np.dot( R_y, R_x ))
        
        return R
    
    def timer_callback(self,event):
        self.estimate_marker_pose(101)
        

if __name__ == "__main__":
    #rospy.init_node('marker_detection', anonymous=True)
    node = marker_detection()

