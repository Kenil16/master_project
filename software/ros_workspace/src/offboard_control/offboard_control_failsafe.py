##!/usr/bin/env python2

PKG = 'px4'
import rospy
import cv2
import math
import numpy as np
#from mavros_test_common import MavrosTestCommon
from pymavlink import mavutil
from six.moves import xrange
from std_msgs.msg import Header
from threading import Thread
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, \
                            WaypointList, LandingTarget, RCOut
from mavros_msgs.srv import CommandBool, ParamGet, SetMode, WaypointClear, \
                            WaypointPush, CommandLong
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from tracking import*
from std_msgs.msg import Float64
from mavlink_msgs.msg import mavlink_lora_command_land

class offboard_control_new:

    def __init__(self):
        
        rospy.init_node('offboard_control')
        
        # ROS services
        service_timeout = 300
        rospy.loginfo("waiting for ROS services")
        try:
            rospy.wait_for_service('mavros/param/get', service_timeout)
            rospy.wait_for_service('mavros/cmd/arming', service_timeout)
            rospy.wait_for_service('mavros/set_mode', service_timeout)
            rospy.wait_for_service('mavros/cmd/land', service_timeout)
            #rospy.wait_for_service('mavros/rc/out', service_timeout)
            rospy.loginfo("ROS services are up")
        except rospy.ROSException:
            self.fail("failed to connect to services")
        
        #Services
        self.uavpos_mode_pub = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.uavpos_arming_pub = rospy.ServiceProxy ("/mavros/cmd/arming", CommandBool)
        self.uavpos_parachute_pub = rospy.ServiceProxy ("/mavros/cmd/command", CommandLong)
        
        #Publish
        self.uavpos_pose_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.uav_pose_x_pub = rospy.Publisher("/uav_current_pos_x", Float64, queue_size=1)
        self.uav_pose_y_pub = rospy.Publisher("/uav_current_pos_y", Float64, queue_size=1)
        self.uav_pose_z_pub = rospy.Publisher("/uav_current_pos_z", Float64, queue_size=1)
        self.uav_motor_failure_pub = rospy.Publisher("/motor_failure", Float64, queue_size=1)
        
        #Subcribes 
        self.uavpos_pose_sub = rospy.Subscriber("/mavros/local_position/pose",PoseStamped,self.get_uav_pose)
        self.uavpos_state_sub = rospy.Subscriber("/mavros/state", State, self.get_uav_state)
        self.uavpos_motor_output_sub = rospy.Subscriber("/mavros/rc/out", RCOut, self.get_motor_output)
        self.uavpos_motor_failure_sub = rospy.Subscriber("/motor_failure", Float64, self.get_motor_failure)

        self.uav_mode = None
        self.uav_pose = PoseStamped()
        self.uav_arm = None
        self.uav_state = None
        self.motor_output = RCOut()
        self.uav_motor_failure = Float64()
        self.uav_motor_failure_enable = False

        self.uavpos_arming_pub(True)
        
        self.img_width = 320
        self.img_height = 240
        self.H_FOV = 1.047
        self.V_FOV = 0.817109
        
        #Initiate Kalman filter
        self.kf = kalman_filter()

        self.new_pos_x = 0
        self.new_pos_y = 0

        #Initiate aruco detection 
        self.cx = 320/2
        self.cy = 240/2
        self.fx = 277#2367.646
        self.fy = 277#2367.646
        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.camera_matrix = np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]], dtype=np.float)
        self.distortion_coefficients = np.array([[0, 0, 0, 0]], dtype=np.float)
        self.image_sub = rospy.Subscriber("/iris_fpv_cam/usb_cam/image_raw", Image, self.estimate_pose_from_aruco_markers)
        self.aruco_marker_image_pub = rospy.Publisher("aruco_marker_image", Image, queue_size=1)
        self.aruco_detected = False
        self.aruco_marker_ori = None
        self.aruco_marker_pos = None
        self.bridge = CvBridge()

        self.marker_img_center_x = 300.
        self.marker_img_center_y = 300.
        
        self.perform_landing = False
        self.motor_cutoff = False

        rospy.Timer(rospy.Duration(1./100.), self.timer_callback)
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

        if len(self.kf.points) > 0:
            self.new_pos_x = self.kf.points[0][0][0]
            self.new_pos_y = self.kf.points[0][1][0]

            print("UAV (x,y,z): " + str(self.uav_pose.pose.position.x) + " " + str(self.uav_pose.pose.position.y) + " " + str(self.uav_pose.pose.position.z))
            print("UAV new pos (x,y): " + str(self.new_pos_x) + " " + str(self.new_pos_y))

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
        self.uav_pose_x_pub.publish(self.uav_pose.pose.position.x)
        self.uav_pose_y_pub.publish(self.uav_pose.pose.position.y)
        self.uav_pose_z_pub.publish(self.uav_pose.pose.position.z)

    def get_uav_mode(self,msg):
        self.uav_mode = msg
    
    def get_motor_output(self,msg):
        self.motor_output = msg

        if self.uav_pose.pose.position.z > 10:
            for motor in range(4):
                if self.motor_output.channels[motor] < 1100:
                    self.uav_motor_failure_enable = True

        if self.uav_motor_failure_enable:
            self.uav_motor_failure_pub.publish(1.0)
        else:
            self.uav_motor_failure_pub.publish(0.0)

        #print("Motor signals: " + str(self.motor_output.channels))
        #print("Motor failure: " + str(self.uav_motor_failure))

    def get_uav_state(self,msg):
        self.uav_state = msg

    def get_motor_failure(self,msg):
        self.uav_motor_failure = msg

    def get_uav_arming(self,msg):
        self.uav_arm = msg

    def timer_callback(self,event):
        new_pose = PoseStamped()
    
        #Increase altitude 
        if self.motor_cutoff == False:
            new_pose.pose.position.z = 20
        
        #Now find marker when above certain altitude using p controllers  
        if (self.uav_pose.pose.position.z > 18) and (self.motor_cutoff == False):
            new_pose.pose.position.x =  self.uav_pose.pose.position.x + self.new_pos_x*0.35
            new_pose.pose.position.y =  self.uav_pose.pose.position.y + self.new_pos_y*0.35

        #Now init disable motors and init parachute 
        if (self.marker_img_center_x < 5 and self.marker_img_center_x > - 5) and (self.marker_img_center_y < 5 and self.marker_img_center_y > - 5):
            self.motor_cutoff = True
        
        #If a failure occurs activate parachute
        if self.motor_cutoff == False:
            self.uavpos_pose_pub.publish(new_pose)
            self.uavpos_mode_pub(0,'OFFBOARD')
        else:
            self.uavpos_mode_pub(0,'MANUAL')
            self.uavpos_arming_pub(False)
            #self.uavpos_parachute_pub(False,185,0,1,0,0,0,0,0,0)

        
if __name__ == "__main__":
    node = offboard_control_new()
