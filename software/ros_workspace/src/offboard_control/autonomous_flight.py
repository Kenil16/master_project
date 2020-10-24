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
from pid import*
from std_msgs.msg import (String, Int8, Float64, Bool)

class autonomous_flight():

    def __init__(self):        
        
        rospy.init_node('autonomous_flight')
        self.rate = rospy.Rate(20)

        #Init flight modes 
        self.flight_mode = flight_modes()
        
        #Init PID controllers
        self.pid_x = PID(1.,1.,1.,1,-1)
        self.pid_y = PID(1.,1.,1.,1,-1)
        self.pid_z = PID(1.,1.,1.,1,-1)
        self.pid_yaw = PID(1.,1.,1.,1,-1)

        #Initialize objects for uav commands and status 
        self.uav_local_pose = PoseStamped()
        self.uav_local_setpoint = PoseStamped()
        self.uav_state = None
        self.uav_home_pose = PoseStamped()
        self.init_uav_home_pose = False

        #Initialize publishers
        self.pub_local_pose = rospy.Publisher('/onboard/setpoint/autonomous_flight', PoseStamped, queue_size=1)
        self.pub_state = rospy.Publisher('/onboard/state', String, queue_size=10)

        #Subscribers
        rospy.Subscriber('/onboard/state', String, self.on_uav_state)
        #rospy.Subscriber('/mavros/setpoint_position/local', PoseStamped, self.on_setpoint_change)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.on_position_change)

        self.new_uav_local_pose = PoseStamped()
        """
        self.new_uav_local_pose.pose.position.z = 2
        self.new_uav_local_pose.pose.position.x = -8
        self.new_uav_local_pose.pose.orientation = Quaternion(*quaternion_from_euler(0,0,3.14))
        """

        #Initialize uav flightmodes
        self.flight_mode.wait_for_topics(60)
        
    def on_position_change(self, msg):
        self.uav_local_pose = msg
        if not self.init_uav_home_pose:
            self.uav_home_pose = self.uav_local_pose
            self.init_uav_home_pose = True

    def on_setpoint_change(self, msg):
        self.uav_local_setpoint = msg
        pass

    #Fuction for updating onboard state
    def set_state(self, state):
        self.sys_state = state
        self.pub_state.publish(state)
        rospy.loginfo('Autonomous_flight: state = {}'.format(state))

    def on_uav_state(self, msg):
        self.uav_state = msg.data

    def pub_msg(self, msg, topic):
        msg.header.frame_id = "att_pose"
        msg.header.stamp = rospy.Time.now()
        topic.publish(msg)
        self.rate.sleep()

    def waypoint_check(self, setpoint, threshold=0.25):
        
        delta_x = self.uav_local_pose.pose.position.x - setpoint[0]
        delta_y = self.uav_local_pose.pose.position.y - setpoint[1]
        delta_z = self.uav_local_pose.pose.position.z - setpoint[2]
        error = np.sqrt(np.power(delta_x,2) + np.power(delta_y,2) + np.power(delta_z,2))
        
        if error < threshold:
            return True
        else:
            return False

    def drone_takeoff(self, alt=1.5):
        
        self.flight_mode.set_arm(True,5)

        pre_pose = self.uav_local_pose
        pre_pose.pose.position.z = alt
        rospy.loginfo('Autonomous_flight: Takeoff altitude = {} m'.format(pre_pose.pose.position.z))

        for i in range(0,5):
            self.pub_msg(pre_pose, self.pub_local_pose)
            
        self.flight_mode.set_mode('OFFBOARD',5)

        rospy.loginfo('Autonomous_flight: UAV takeoff')

        #Wait until takeoff has occurred
        waypoint = [self.uav_local_pose.pose.position.x, self.uav_local_pose.pose.position.y, alt]
        while(not self.waypoint_check(setpoint = waypoint)):
            if self.uav_state == 'loiter' or self.uav_state == 'home':
                rospy.loginfo('Autonomous_flight: Takeoff disrupted!')
                return
            self.pub_msg(pre_pose, self.pub_local_pose)
            
        rospy.loginfo('Autonomous_flight: Takeoff complete')
        self.set_state('loiter')

    def drone_return_home(self):

        x_cur = self.uav_local_pose.pose.position.x
        y_cur = self.uav_local_pose.pose.position.y
        z_cur = self.uav_local_pose.pose.position.z

        x_home = self.uav_home_pose.pose.position.x
        y_home = self.uav_home_pose.pose.position.y
        z_home = self.uav_home_pose.pose.position.z

        x_delta = abs(x_cur-x_home)
        y_delta = abs(y_cur-y_home)
        z_delta = abs(z_cur-z_home)

        threshold = 0.25
        alt = 1.5
        if x_delta > threshold or y_delta > threshold or z_delta > threshold:

            if not self.flight_mode.state.armed:
                self.flight_mode.set_arm(True,5)
                for i in range(0,5):
                    self.pub_msg(pre_pose, self.uav_local_pose)
                self.flight_mode.set_mode('OFFBOARD',5)
                waypoints = [[x_cur, y_cur, alt], [x_home, y_home, alt], [0, 0, 0]]
            else:
                waypoints = [[x_home, y_home, alt], [0, 0, 0]]

            rospy.loginfo('Autonomous_flight: UAV returning home')
        
            for waypoint in waypoints:

                pre_pose = PoseStamped()
                pre_pose.pose.position.x = waypoint[0]
                pre_pose.pose.position.y = waypoint[1]
                pre_pose.pose.position.z = waypoint[2]

                #wait until waypoint reached
                while(not self.waypoint_check(setpoint=waypoint)):
                    if not self.uav_state == 'home':
                        rospy.loginfo('Autonomous_flight: Return home disrupted!')
                        return
                    self.pub_msg(pre_pose, self.pub_local_pose)
                    
            rospy.loginfo('Autonomous_flight: UAV has returned home')
            self.set_state('idle')
        else:
            rospy.loginfo('Autonomous_flight: UAV already at home position')
            self.set_state('idle')

    def aruco_pose_estimation_test(self):

        #UAV valocity
        self.flight_mode.set_param('MPC_XY_VEL_MAX', 2.0, 5)
        self.flight_mode.set_param('MPC_Z_VEL_MAX_DN', 0.5, 5)
        self.flight_mode.set_param('MPC_Z_VEL_MAX_UP', 0.5, 5)

        alt_ = 1
        self.drone_takeoff(alt = alt_)

        self.set_state('aruco_pose_estimation_test')
        rospy.loginfo('Autonomous_flight: UAV aruco pose estimation test startet')

        x = self.uav_home_pose.pose.position.x
        y = self.uav_home_pose.pose.position.y

        waypoints = [[x, y-5, alt_], [x, y+5, alt_], [x, y, alt_]]
        angle = Quaternion(*quaternion_from_euler(0, 0, 0))

        for waypoint in waypoints:

            pre_pose = PoseStamped()
            pre_pose.pose.position.x = waypoint[0]
            pre_pose.pose.position.y = waypoint[1]
            pre_pose.pose.position.z = waypoint[2]
            pre_pose.pose.orientation = angle

            #wait until waypoint reached
            while(not self.waypoint_check(setpoint = waypoint)):
                if not self.uav_state == 'aruco_pose_estimation_test':
                    rospy.loginfo('Autonomous_flight: Aruco pose estimation test disrupted!')
                    return
                self.pub_msg(pre_pose, self.pub_local_pose)
        
        rospy.loginfo('Autonomous_flight: Aruco pose estimation test complete')
        self.set_state('loiter')

    def gps_to_vision_test(self):

        #UAV valocity
        self.flight_mode.set_param('MPC_XY_VEL_MAX', 0.1, 5)
        self.flight_mode.set_param('MPC_Z_VEL_MAX_DN', 0.1, 5)
        self.flight_mode.set_param('MPC_Z_VEL_MAX_UP', 0.1, 5)

        alt_ = 1
        self.drone_takeoff(alt = alt_)

        self.set_state('gps_to_vision_test')
        rospy.loginfo('Autonomous_flight: Gps to vision test startet')

        self.flight_mode.set_param('EKF2_AID_MASK', 24, 5)
        self.flight_mode.set_param('EKF2_HGT_MODE', 3, 5)
        self.flight_mode.set_param('EKF2_EV_DELAY', 50., 5)

        while True:

            
            if not self.uav_state == 'gps_to_vision_test':
                rospy.loginfo('Autonomous_flight: Gps to vision test disrupted!')
                return
            
            new_pose = PoseStamped()
            
            new_pose.pose.position.x = -4
            new_pose.pose.position.y = 0
            new_pose.pose.position.z = 0

            self.pub_msg(new_pose, self.pub_local_pose)
        
        rospy.loginfo('Autonomous_flight: Gps to vision test complete')
        self.set_state('loiter')

    def run(self):
        while not rospy.is_shutdown():
            if self.uav_state == 'takeoff':
                self.drone_takeoff()
            elif self.uav_state == 'home':
                self.drone_return_home()
            elif self.uav_state == 'aruco_pose_estimation_test':
                self.aruco_pose_estimation_test()
            elif self.uav_state == 'gps_to_vision_test': 
                self.gps_to_vision_test()

            self.rate.sleep()

if __name__ == "__main__":
    af = autonomous_flight()
    af.run() 
