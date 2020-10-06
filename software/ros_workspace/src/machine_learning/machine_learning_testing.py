#!/usr/bin/python

#Make relative path to other modules
import sys
sys.path.append('../offboard_control')

from uav_flight_modes import*
from pid import*

import cv2
import numpy as np
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from mavlink_msgs.msg import mavlink_lora_aruco
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Float64, Bool
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, WaypointList, ParamValue
from mavros_msgs.srv import CommandBool, CommandTOL, ParamGet, SetMode, WaypointClear, WaypointPush, ParamSet

class machine_learning_testing:

    def __init__(self):        
        
        rospy.init_node('machine_learning_testing')

        #Init flight modes
        self.flight_mode = flight_modes()
        
        #Init PID controllers
        self.pid_x = PID(1.,1.,1.,1,-1)
        self.pid_y = PID(1.,1.,1.,1,-1)
        self.pid_z = PID(1.,1.,1.,1,-1)
        self.pid_yaw = PID(1.,1.,1.,1,-1)

        #Number of ArUco markers to track (ID)
        self.aruco_ids = mavlink_lora_aruco()
        self.aruco_ids.elements.append(101)
        self.vision_based_navigation = False
        self.aruco_marker_found = False
        self.distance_to_marker = 100
        self.set_mode_param = rospy.ServiceProxy('/mavros/param/set', ParamSet)
        
        #Initialize publishers
        self.publish_local_pose = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.publish_aruco_ids = rospy.Publisher("aruco_ids", mavlink_lora_aruco, queue_size=10)

        #Initialize suscribers
        self.subscibe_aruco_pose = rospy.Subscriber("/mavros/vision_pose/pose", PoseStamped, self.aruco_marker_pose_callback)
        self.aruco_marker_found_sub = rospy.Subscriber("aruco_marker_found", Bool, self.aruco_marker_found_callback)

        #Set initial uav pose
        self.new_uav_local_pose = PoseStamped()
        self.new_uav_local_pose.pose.position.z = 2
        self.new_uav_local_pose.pose.position.x = -8
        self.new_uav_local_pose.pose.orientation = Quaternion(*quaternion_from_euler(0,0,3.14))

        self.aruco_marker_pose = PoseStamped()

        #Initialize uav flightmodes
        self.flight_mode.wait_for_topics(60)
        
        self.publish_local_pose.publish(self.new_uav_local_pose)
        self.publish_local_pose.publish(self.new_uav_local_pose)
        
        self.flight_mode.set_mode("OFFBOARD",5)
        self.flight_mode.set_arm(True,5)
        
        self.cycle_time = (1./100.)
        rospy.Timer(rospy.Duration(self.cycle_time), self.timer_callback)
        rospy.spin()

    def aruco_marker_pose_callback(self, data):
        self.aruco_marker_pose = data
        
    def aruco_marker_found_callback(self, data):
        self.aruco_marker_found = data
        
    def timer_callback(self,event):
        
        if self.aruco_marker_found:
            x = np.power(self.aruco_marker_pose.pose.position.x,2)
            y = np.power(self.aruco_marker_pose.pose.position.y,2)
            z = np.power(self.aruco_marker_pose.pose.position.z,2)
            self.distance_to_marker = np.sqrt((x+y+z))
            
        #Go from GPS to vision based navigation
        if not self.vision_based_navigation and (self.distance_to_marker > 0.0) and (self.distance_to_marker < 5):
            
            #self.set_mode_param("SIM_GPS_BLOCK",ParamValue(integer=1, real=0.0))
            self.set_mode_param("EKF2_AID_MASK",ParamValue(integer=24, real=0.0))
            #self.flight_mode.set_mode_param("EKF2_HGT_MODE",ParamValue(integer=3, real=0.0),10)
            self.vision_based_navigation = True
            print "Aruco Found"
        
        self.publish_aruco_ids.publish(self.aruco_ids)

        if self.vision_based_navigation:
            self.new_uav_local_pose.pose.position.z = 2
            self.new_uav_local_pose.pose.position.x = -8
            self.publish_local_pose.publish(self.new_uav_local_pose)
        else:
            self.publish_local_pose.publish(self.new_uav_local_pose)
        
if __name__ == "__main__":
    node = machine_learning_testing()
