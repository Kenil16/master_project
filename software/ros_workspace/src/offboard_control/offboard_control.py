#!/usr/bin/env python2

##!/usr/bin/python 

import cv2
import numpy as np
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from mavlink_msgs.msg import mavlink_lora_command_land
from uav_flight_modes import*

class offboard_control:

    def __init__(self):        
        
        rospy.init_node('offboard_control')

        self.flight_mode = flight_modes()

        #Initialize objects for uav commands and status 
        self.uav_local_pose = PoseStamped()
        
        #Initialize publishers
        self.publish_local_pose = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)

        
        #Initiate Kalman filter
        #self.kf = kalman_filter()

        self.new_uav_local_pose = PoseStamped()
        self.new_uav_local_pose.pose.position.z = 2

        #Initiate aruco detection 
        self.cx = 320/2
        self.cy = 240/2
        self.fx = 277#2367.646
        self.fy = 277#2367.646
        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.camera_matrix = np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]], dtype=np.float)
        self.distortion_coefficients = np.array([[0, 0, 0, 0]], dtype=np.float)
        self.image_sub = rospy.Subscriber("/stereo/right/image_raw", Image, self.estimate_pose_from_aruco_markers)
        self.aruco_marker_image_pub = rospy.Publisher("aruco_marker_image", Image, queue_size=1)
        self.aruco_detected = False
        self.aruco_marker_ori = None
        self.aruco_marker_pos = None
        self.bridge = CvBridge()

        self.marker_img_center_x = 300.
        self.marker_img_center_y = 300.
        self.perform_landing = False
        self.landing_pose = PoseStamped()
        
        rospy.Timer(rospy.Duration(1./100.), self.timer_callback)

        #Initialize uav flightmodes
        self.flight_mode.wait_for_topics(60)
        
        self.publish_local_pose.publish(self.new_uav_local_pose)
        self.publish_local_pose.publish(self.new_uav_local_pose)
        
        self.flight_mode.set_mode("OFFBOARD",5)
        self.flight_mode.set_arm(True,5)

        rospy.spin()
        
    def estimate_pose_from_aruco_markers(self, img):

        #Load in the marker image
        try:
            cv_img = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            print(e)

        image_markers = cv_img

        # Detect aruco markers in the image
        marker_corners, marker_ids, rejected_candidates = cv2.aruco.detectMarkers(cv_img, self.dictionary, parameters=self.parameters)

        if len(marker_corners) > 0:
            image_markers = cv2.aruco.drawDetectedMarkers(cv_img, marker_corners, marker_ids)

            # Estimate pose and draw axis
            pose = cv2.aruco.estimatePoseSingleMarkers(marker_corners, 2, self.camera_matrix, self.distortion_coefficients)

            #image_markers = cv2.aruco.drawAxis(image_markers, self.camera_matrix, self.distortion_coefficients, pose[0], pose[1], 0.5)
            self.aruco_marker_ori = self.rodrigues_to_euler_angles(pose[0])
            self.aruco_marker_pos = pose[1]
            
            if len(marker_corners) > 0:
                center_x = int((marker_corners[0][0][0][0] + marker_corners[0][0][3][0]) / 2)
                center_y = int((marker_corners[0][0][0][1] + marker_corners[0][0][1][1]) / 2)
                self.marker_img_center_x = center_x - self.img_width/2
                self.marker_img_center_y = center_y - self.img_height/2
                cv2.circle(image_markers,(center_x,center_y), 2, (0,0,255), -1)
            
            
            image_point = np.array([center_x,center_y,1], dtype=np.float)

            #Get roll, pitch and yaw of drone 
            angles = self.get_rpy_orientation(self.uav_pose.pose.orientation)
            
            """
            #Extrincic 
            rz = np.array([[np.cos(-angles[2]+np.pi/2),-np.sin(-angles[2]+np.pi/2),0,           self.uav_pose.pose.position.x],
                       [np.sin(-angles[2]+np.pi/2),np.cos(-angles[2]+np.pi/2),0,                self.uav_pose.pose.position.y],
                       [0,                  0,                  1,                              self.uav_pose.pose.position.z],
                       [0,0,0,1]])

            #Projection matrix
            X = ((center_x-self.cx)/self.fx)*self.uav_pose.pose.position.z
            Y = ((center_y-self.cy)/self.fy)*self.uav_pose.pose.position.z

            P = np.array([X,Y,self.uav_pose.pose.position.z,1],dtype=np.float)
            

            #Now move drone towards f-fold marker
            img_r = np.array([[np.cos(-angles[2]),-np.sin(angles[2]),0],
                        [np.sin(angles[2]),np.cos(angles[2]),0],
                        [0,0,1]])
            
            img_r_inv = np.linalg.inv(img_r)
            
            #Get ground sample distance
            H_GSD = (self.H_FOV*self.uav_pose.pose.position.z)/self.img_width
            V_GSD = (self.V_FOV*self.uav_pose.pose.position.z)/self.img_height
            
            #Found n-fold marker position
            marker_pose_img_x = V_GSD*(self.img_height/2-center_y)
            marker_pose_img_y = H_GSD*(self.img_width/2-center_x)
            
            #Use transformation to align marker position to world
            t_points = np.array([marker_pose_img_x,marker_pose_img_y,1])
            t_points = t_points.dot(img_r_inv)
            
            #Use Kalman filtering for tracking of n-fold marker
            self.kf.get_measurement(np.array([[t_points[0],t_points[1]]]))

            """    

        """
        if len(self.kf.points) > 0:
            self.new_pos_x = self.kf.points[0][0][0]
            self.new_pos_y = self.kf.points[0][1][0]

            #marker_pos = np.dot(rz,P)
            #print("Estimated marker pose: " + str(marker_pos))
            #print("Before transformation: " + str(X) + " " + str(Y) + " " + str(self.uav_pose.pose.position.z))
            print("UAV (x,y,z): " + str(self.uav_pose.pose.position.x) + " " + str(self.uav_pose.pose.position.y) + " " + str(self.uav_pose.pose.position.z))
            print("Marker new pos (x,y): " + str(self.new_pos_x) + " " + str(self.new_pos_y))
        """

        try:
            self.aruco_marker_image_pub.publish(self.bridge.cv2_to_imgmsg(image_markers, "bgr8"))
        except CvBridgeError as e:
            print(e)
            
            
    def get_rpy_orientation (self,orientation_q):
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        self.current_uav_roll = roll
        self.current_uav_pitch = pitch
        self.current_uav_yaw = yaw
        return [roll, pitch, yaw]
            
            
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


    def get_uav_pose(self,msg):
        self.uav_pose = msg
    
    def get_uav_mode(self,msg):
        self.uav_mode = msg

    def get_uav_state(self,msg):
        self.uav_state = msg

    def get_uav_arming(self,msg):
        self.uav_arm = msg

    def timer_callback(self,event):
        
        #Increase altitude 
        if self.perform_landing == False and self.flight_mode.local_position.pose.position.z > 2:
            self.perform_landing = True
            self.flight_mode.set_mode("AUTO.LAND",5)
        
        self.publish_local_pose.publish(self.new_uav_local_pose)
        
        
if __name__ == "__main__":
    node = offboard_control()
