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
from geometry_msgs.msg import PoseStamped, Quaternion

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
        self.new_uav_local_pose.pose.position.x = -8
        self.new_uav_local_pose.pose.orientation = Quaternion(*quaternion_from_euler(0,0,3.14))
        
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
        
    def timer_callback(self,event):
        
        """
        #Increase altitude 
        if self.perform_landing == False and self.flight_mode.local_position.pose.position.z > 2:
            self.perform_landing = True
            self.flight_mode.set_mode("AUTO.LAND",5)
        """
        
        self.publish_local_pose.publish(self.new_uav_local_pose)
        
        
if __name__ == "__main__":
    node = offboard_control()
