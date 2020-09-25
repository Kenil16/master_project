#!/usr/bin/python

import cv2
import numpy as np
from os import system
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from uav_flight_modes import*

class marker_detection:

    def __init__(self):

        self.uav_flight_modes = flight_modes()
        
        #Subscribers
        self.image_sub = rospy.Subscriber("/mono_cam/image_raw", Image, self.find_aruco_markers)

        #Publishers
        self.aruco_marker_image_pub = rospy.Publisher("aruco_marker_image", Image, queue_size=1)

        #Initiate aruco detection (Intinsic and extrinsic camera coefficients can be found in sdu_mono_cam model)
        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_1000)
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.camera_matrix = np.array([[277.191356, 0, (320.5/2)], [0, 277.191356, (340.5/2)], [0, 0, 1]], dtype=np.float)
        self.distortion_coefficients = np.array([[0, 0, 0, 0]], dtype=np.float)
        self.aruco_detected = False
        self.marker_pose = []
        self.bridge = CvBridge()
        
    def find_aruco_markers(self,img):
        
        #Load in the marker image
        try:
            cv_img = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            print(e)
            
        image_markers = cv_img
        
        #Detect aruco markers in the image
        marker_corners, marker_ids, rejected_candidates = cv2.aruco.detectMarkers(cv_img, self.dictionary, parameters=self.parameters)
        
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
                self.marker_pose.append(marker_id_pose)
                
                #Draw detected axis of markers
                image_markers = cv2.aruco.drawAxis(image_markers, self.camera_matrix, self.distortion_coefficients, _rvec, _tvec, 0.05)

        img = self.bridge.cv2_to_imgmsg(image_markers,"bgr8")
        self.aruco_marker_image_pub.publish(img)

    def estimate_marker_pose(self, marker_id):

        if self.marker_pose:
            
            for pose in self.marker_pose:
                eular_angles = euler_from_quaternion(pose[1].pose.orientation)
                print "Marker id: {} Position: {} Orientation: {} \n".format(pose[0],pose[1].pose.position,eular_angles)
                


            

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
        print "Working"

if __name__ == "__main__":
    rospy.init_node('marker_detection', anonymous=True)
    node = marker_detection()

