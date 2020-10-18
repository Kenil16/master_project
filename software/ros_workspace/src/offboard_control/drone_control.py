#!/usr/bin/env python

import numpy as np
import rospy
from  mavros_msgs.msg import State

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import (String, Int8, Float64, Bool)
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class drone_control():
    def __init__(self):
        
        #Init ROS
        rospy.init_node('drone_control')
        self.rate = rospy.Rate(20)
        
        #Variables
        self.uav_state = 'idle' 
        self.mavros_state = State()
        
        self.autonomous_flight_pose_msg = PoseStamped()
        self.autonomous_fligt_state_msg = None
        self.loiter_pilot_msg = PoseStamped()

        #Subscribers
        rospy.Subscriber('/mavros/state', State, self.cb_uav_state) 
        rospy.Subscriber('/gcs/command', Int8, self.on_command)
        rospy.Subscriber('/onboard/setpoint/autonomous_flight', PoseStamped, self.af_setpoint_change)
        rospy.Subscriber('/onboard/state', String, self.on_uav_state)
        rospy.Subscriber('/onboard/setpoint/loiter_pilot', PoseStamped, self.lp_setpoint_change)

        #Publishers
        self.pub_state = rospy.Publisher('/onboard/state', String, queue_size=1)
        self.pub_local_pose = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=5)

        #Perform MAVROS handshake   
        self.mavros_handshake()

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
        rospy.loginfo('DroneCore: state = {}'.format(state))

    def af_setpoint_change(self,msg):
        self.autonomous_flight_pose_msg = msg

    def on_uav_state(self,msg):
        self.uav_state = msg.data

    def lp_setpoint_change(self,msg):
        self.loiter_pilot_msg = msg

    def on_command(self, msg):
        command = str(chr(msg.data))
        command.lower()
    
        #Change state according to GC command
        
        if command == 't': #Takeoff
            self.set_state('takeoff')
        if command == 'h': #Returns the drone to home
            pass
        
        if command == 'm': #Execute mission
            pass

        if command == 'k': # Kill drone
            self._setState('idle')       

    def message_control(self):

        if not self.uav_state == 'idle':

            output_msg = None

            if self.uav_state == 'loiter':
                output_msg = self.loiter_pilot_msg

            if self.uav_state == 'takeoff':
                output_msg = self.autonomous_flight_pose_msg
            
            if output_msg == None:
                rospy.logfatal_once("Message control received no message: Has a pilot crashed?")
                self.setMode(0, "AUTO.LOITER")
                rospy.loginfo('Message_control: PX4 mode = AUTO.LOITER')
            
            self.pub_msg(output_msg, self.pub_local_pose)

    def cb_uav_state(self, msg):
        self.mavros_state = msg
        
    def run(self):
        while not rospy.is_shutdown():
            self.message_control()
            self.rate.sleep()

if __name__ == "__main__":
    dc = drone_control()
    dc.run()
