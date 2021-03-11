#!/usr/bin/env python

import numpy as np
import rospy
from  mavros_msgs.msg import State
from mavros_msgs.srv import SetMode

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import (String, Int8, Float64, Bool)
from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import sys

class drone_control():
    def __init__(self):
        
        #Arguments for which initial uav state to be in 
        self.arg = sys.argv

        #Init ROS
        rospy.init_node('drone_control')
        self.rate = rospy.Rate(20)
        
        #Variables
        print(self.arg[1])
        
        self.uav_state = 'idle' 
        
        self.mavros_state = State()
        
        self.autonomous_flight_pose_msg = PoseStamped()
        self.aruco_marker_pose_msg = PoseStamped()
        self.autonomous_fligt_state_msg = None
        self.loiter_pilot_msg = PoseStamped()
        self.sensor_fusion = PoseStamped()

        self.aruco_pose_covariance = PoseWithCovarianceStamped()

        #Subscribers
        rospy.Subscriber('/mavros/state', State, self.cb_uav_state) 
        rospy.Subscriber('/gcs/command', Int8, self.on_command)
        rospy.Subscriber('/onboard/setpoint/autonomous_flight', PoseStamped, self.af_setpoint_change)
        rospy.Subscriber('/onboard/state', String, self.on_uav_state)
        rospy.Subscriber('/onboard/setpoint/loiter_pilot', PoseStamped, self.lp_setpoint_change)
        rospy.Subscriber('/onboard/aruco_marker_pose', PoseStamped, self.amp_change)
        
        rospy.Subscriber('/onboard/sensor_fusion', PoseStamped, self.sensor_fusion_callback)
        
        #Publishers
        self.pub_state = rospy.Publisher('/onboard/state', String, queue_size=1)
        self.pub_local_pose = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.pub_vision_pose = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=1)

        #self.aruco_marker_pose_cov = rospy.Publisher('/mavros/vision_pose/pose_cov', PoseWithCovarianceStamped, queue_size=1)
        
        self.set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        #Perform MAVROS handshake   
        self.mavros_handshake()

        self.set_state(self.arg[1])

    def mavros_handshake(self): 
        rospy.loginfo('Drone_control: Waiting for MAVROS Connection.')
        i=0
        time = rospy.Time.now()
        for i in range(0,3):
            print'.',
            if self.mavros_state.connected:
                rospy.loginfo("Drone_control: MAVROS Connected!")
                break
            rospy.sleep(1)
        if not self.mavros_state.connected:
            errorMsg = "Drone_control: MAVROS not connected!"
            rospy.logfatal(errorMsg)
            rospy.signal_shutdown(errorMsg)
    
    def pub_msg(self, msg, topic):
        msg.header.frame_id = "att_pose"
        msg.header.stamp = rospy.Time.now()
        topic.publish(msg)
        self.rate.sleep()
    
    #Fuction for updating onboard state
    def set_state(self, state):
        self.uav_state = state
        self.pub_state.publish(state)
        rospy.loginfo('Drone_control: state = {}'.format(state))

    def af_setpoint_change(self,msg):
        self.autonomous_flight_pose_msg = msg

    def sensor_fusion_callback(self,msg):
        self.sensor_fusion = msg 
    
    def on_uav_state(self,msg):
        self.uav_state = msg.data

    def lp_setpoint_change(self,msg):
        self.loiter_pilot_msg = msg

    def amp_change(self,msg):
        self.aruco_marker_pose_msg = msg

    def on_command(self, msg):
        
        #Change state according to GC command
        command = str(chr(msg.data))
        command.lower()
    
        if command == 't': #Takeoff
            self.set_state('takeoff')

        if command == 'h': #Returns the drone to home
            self.set_state('home')
        
        if command == 'l': #Execute mission
            self.set_state('loiter')

        if command == 'k': # Kill drone
            rospy.signal_shutdown("test")

        #Execute a number of mission tests
        if command == '1':
            self.set_state('estimate_aruco_pose_front_test')

        if command == '2':
            self.set_state('follow_aruco_pose_bottom_test')

        if command == '3':
            self.set_state('hold_aruco_pose_test')

        if command == '4':
            self.set_state('GPS2Vision_test')

    def message_control(self):

        if not self.uav_state == 'idle':

            output_msg = None

            if self.uav_state == 'loiter':
                output_msg = self.loiter_pilot_msg
                self.pub_msg(output_msg, self.pub_local_pose)

            if self.uav_state == 'takeoff':
                output_msg = self.autonomous_flight_pose_msg
                self.pub_msg(output_msg, self.pub_local_pose)
            
            if self.uav_state == 'home':
                output_msg = self.autonomous_flight_pose_msg
                self.pub_msg(output_msg, self.pub_local_pose)
            
            if self.uav_state == 'estimate_aruco_pose_front_test':
                output_msg = self.autonomous_flight_pose_msg
                self.pub_msg(output_msg, self.pub_local_pose)

            if self.uav_state == 'GPS2Vision_test':
                output_msg = self.autonomous_flight_pose_msg
                self.pub_msg(output_msg, self.pub_local_pose)
                self.pub_msg(self.aruco_marker_pose_msg, self.pub_vision_pose)

            if self.uav_state == 'follow_aruco_pose_bottom_test':
                output_msg = self.autonomous_flight_pose_msg
                self.pub_msg(output_msg, self.pub_local_pose)
                self.pub_msg(self.sensor_fusion, self.pub_vision_pose)
                
                #self.pub_msg(self.aruco_marker_pose_msg, self.pub_vision_pose)
            
            if self.uav_state == 'hold_aruco_pose_test':
                output_msg = self.autonomous_flight_pose_msg
                self.pub_msg(output_msg, self.pub_local_pose)
                self.pub_msg(self.aruco_marker_pose_msg, self.pub_vision_pose)
            
            if output_msg == None:
                rospy.logfatal_once("Drone control received no message: Has a pilot crashed?")
                self.set_mode(0, "AUTO.LOITER")
                rospy.loginfo('Drone_control: PX4 mode = AUTO.LOITER')

    def cb_uav_state(self, msg):
        self.mavros_state = msg
        
    def run(self):
        while not rospy.is_shutdown():
            self.message_control()
            self.rate.sleep()

if __name__ == "__main__":
    dc = drone_control()
    dc.run()
