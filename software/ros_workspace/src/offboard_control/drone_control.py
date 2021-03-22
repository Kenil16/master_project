#!/usr/bin/env python

import numpy as np
import rospy
from  mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from tf.transformations import*
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
        self.uav_pose = PoseStamped()
        self.autonomous_fligt_state_msg = None
        self.loiter_pilot_msg = PoseStamped()
        self.sensor_fusion = PoseStamped()
        self.aruco_ofset_mapping = [[0,0],
                                    [0,0],
                                    [0,0],
                                    [0,0]]

        self.GPS2Vision_offset =[[0,0], [0,0], [0,0],[0,0]]

        self.aruco_pose_covariance = PoseWithCovarianceStamped()
        self.aruco_pose_corrected = PoseStamped()

        self.aruco_offset = PoseStamped()
        self.uav_offset = PoseStamped()
        self.aruco_offset_init = False
        self.uav_offset_init = False
        self.use_GPS2Vision_offset = False

        self.r = euler_matrix(0, 0, np.deg2rad(-90), 'rxyz')

        #Subscribers
        rospy.Subscriber('/mavros/state', State, self.cb_uav_state) 
        rospy.Subscriber('/gcs/command', Int8, self.on_command)
        rospy.Subscriber('/onboard/setpoint/autonomous_flight', PoseStamped, self.af_setpoint_change)
        rospy.Subscriber('/onboard/state', String, self.on_uav_state)
        rospy.Subscriber('/onboard/setpoint/loiter_pilot', PoseStamped, self.lp_setpoint_change)
        rospy.Subscriber('/onboard/aruco_marker_pose', PoseStamped, self.amp_change)
        
        rospy.Subscriber('/onboard/aruco_offset', PoseStamped, self.aruco_offset_callback)
        rospy.Subscriber('/onboard/uav_offset', PoseStamped, self.uav_offset_callback)
        
        rospy.Subscriber('/onboard/sensor_fusion', PoseStamped, self.sensor_fusion_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_position_callback)        
        
        #Publishers
        self.pub_state = rospy.Publisher('/onboard/state', String, queue_size=1)
        self.pub_local_pose = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.pub_vision_pose = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=1)
        self.pub_waypoint_check = rospy.Publisher('/onboard/waypoint_check', Bool, queue_size=1)

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

    def local_position_callback(self,msg):
        self.uav_pose = msg

    #Fuction for updating onboard state
    def set_state(self, state):
        self.uav_state = state
        self.pub_state.publish(state)
        rospy.loginfo('Drone_control: state = {}'.format(state))

    def af_setpoint_change(self,msg):
        self.autonomous_flight_pose_msg = msg

    def sensor_fusion_callback(self,msg):
        self.sensor_fusion = msg 
    
    def aruco_offset_callback(self,msg):
        offset = msg
        self.aruco_offset = msg
        self.aruco_offset_init = True

    def uav_offset_callback(self,msg):
        offset = msg
        self.uav_offset = msg
        self.uav_offset_init = True

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
        
        #Execute missions from GPS to landing stations or vision2GPS locations
        if command == '0':
            self.set_state('move2GPS_locations_from_vision')
        
        if command == '1':
            self.set_state('return_to_landing_station_one')
        
        if command == '2':
            self.set_state('return_to_landing_station_two')
        
        if command == '3':
            self.set_state('return_to_landing_station_three')
        
        #Execute tests from substates of missions 
        if command == '4':
            self.set_state('GPS2Vision_aruco_pose_estimation_test')

        if command == '5':
            self.set_state('hold_aruco_pose_test')

        if command == '6':
            self.set_state('GPS2Vision_test')

        if command == '7':
            self.set_state('vision_navigation_test')

        if command == '8':
            self.set_state('landing_test')

    def message_control(self):

        if not self.uav_state == 'idle':

            output_msg = None

            #If wanted map aruco marker estimate to UAV globle pose
            if self.aruco_offset_init and self.uav_offset_init:
                self.map_GPS_pose_to_vision(self.aruco_offset, self.uav_offset)

            if self.uav_state == 'loiter':
                output_msg = self.loiter_pilot_msg
                self.pub_msg(output_msg, self.pub_local_pose)

            if self.uav_state == 'takeoff':
                output_msg = self.autonomous_flight_pose_msg
                self.pub_msg(output_msg, self.pub_local_pose)
            
            if self.uav_state == 'home':
                output_msg = self.autonomous_flight_pose_msg
                self.pub_msg(output_msg, self.pub_local_pose)
            
            if self.uav_state == 'move2GPS_locations_from_vision':
                output_msg = self.autonomous_flight_pose_msg
                self.pub_msg(output_msg, self.pub_local_pose)
                new_pose = self.vision2local(self.aruco_ofset_mapping, self.aruco_marker_pose_msg) 
                self.pub_msg(new_pose, self.pub_vision_pose)
            
            if self.uav_state == 'return_to_landing_station_one':
                output_msg = self.autonomous_flight_pose_msg
                if output_msg.header.frame_id == "GPS2Vision_setpoint_offset":
                    output_msg = self.calculate_GPS2Vision_offset(self.GPS2Vision_offset, output_msg, True)
                new_pose = self.calculate_GPS2Vision_offset(self.GPS2Vision_offset, self.aruco_marker_pose_msg)
                #new_pose = self.vision2local(self.aruco_ofset_mapping, self.aruco_marker_pose_msg) 
                self.pub_msg(output_msg, self.pub_local_pose)
                self.pub_msg(new_pose, self.pub_vision_pose)
            
            if self.uav_state == 'return_to_landing_station_two':
                output_msg = self.autonomous_flight_pose_msg
                self.pub_msg(output_msg, self.pub_local_pose)
                new_pose = self.vision2local(self.aruco_ofset_mapping, self.aruco_marker_pose_msg) 
                self.pub_msg(new_pose, self.pub_vision_pose)
            
            if self.uav_state == 'return_to_landing_station_three':
                output_msg = self.autonomous_flight_pose_msg
                self.pub_msg(output_msg, self.pub_local_pose)
                new_pose = self.vision2local(self.aruco_ofset_mapping, self.aruco_marker_pose_msg) 
                self.pub_msg(new_pose, self.pub_vision_pose)
            
            if self.uav_state == 'GPS2Vision_aruco_pose_estimation_test':
                output_msg = self.autonomous_flight_pose_msg
                self.pub_msg(output_msg, self.pub_local_pose)
            
            if self.uav_state == 'hold_aruco_pose_test':
                output_msg = self.autonomous_flight_pose_msg
                self.pub_msg(output_msg, self.pub_local_pose)
                new_pose = self.vision2local(self.aruco_ofset_mapping, self.aruco_marker_pose_msg)
                self.pub_msg(new_pose, self.pub_vision_pose)

            if self.uav_state == 'GPS2Vision_test':
                output_msg = self.autonomous_flight_pose_msg
                self.pub_msg(output_msg, self.pub_local_pose)
                new_pose = self.vision2local(self.aruco_ofset_mapping, self.aruco_marker_pose_msg)
                self.pub_msg(new_pose, self.pub_vision_pose)
            
            if self.uav_state == 'vision_navigation_test':
                output_msg = self.autonomous_flight_pose_msg
                self.pub_msg(output_msg, self.pub_local_pose)
                new_pose = self.vision2local(self.aruco_ofset_mapping, self.aruco_marker_pose_msg)
                self.pub_msg(new_pose, self.pub_vision_pose)
            
            if self.uav_state == 'landing_test':
                output_msg = self.autonomous_flight_pose_msg
                self.pub_msg(output_msg, self.pub_local_pose)
                new_pose = self.vision2local(self.aruco_ofset_mapping, self.aruco_marker_pose_msg)
                self.pub_msg(new_pose, self.pub_vision_pose)
            
            if self.uav_state == 'vision_landed':
                output_msg = self.autonomous_flight_pose_msg
                self.pub_msg(output_msg, self.pub_local_pose)
                new_pose = self.vision2local(self.aruco_ofset_mapping, self.aruco_marker_pose_msg)
                self.pub_msg(new_pose, self.pub_vision_pose)
            
            if output_msg == None:
                rospy.logfatal_once("Drone control received no message: Has a pilot crashed?")
                self.set_mode(0, "AUTO.LOITER")
                rospy.loginfo('Drone_control: PX4 mode = AUTO.LOITER')

            self.waypoint_check(self.ua

    def cb_uav_state(self, msg):
        self.mavros_state = msg

    """
    def vision2local(self, aruco_ofset_mapping, aruco_pose):

        if self.aruco_offset_init and self.uav_offset_init:
            
            angle_uav = euler_from_quaternion([self.uav_offset.pose.orientation.x,
                                               self.uav_offset.pose.orientation.y,
                                               self.uav_offset.pose.orientation.z,
                                               self.uav_offset.pose.orientation.w])

            angle_aruco = euler_from_quaternion([self.aruco_offset.pose.orientation.x,
                                                 self.aruco_offset.pose.orientation.y,
                                                 self.aruco_offset.pose.orientation.z,
                                                 self.aruco_offset.pose.orientation.w])

            r_uav = quaternion_matrix([self.uav_offset.pose.orientation.x,
                                       self.uav_offset.pose.orientation.y,
                                       self.uav_offset.pose.orientation.z,
                                       self.uav_offset.pose.orientation.w])


            r_aruco = quaternion_matrix([self.aruco_offset.pose.orientation.x,
                                         self.aruco_offset.pose.orientation.y,
                                         self.aruco_offset.pose.orientation.z,
                                         self.aruco_offset.pose.orientation.w])

            t = np.array([self.aruco_offset.pose.position.x,
                          self.aruco_offset.pose.position.y,
                          self.aruco_offset.pose.position.z, 1])
            
            self.r = np.matmul(r_aruco.T, r_uav)
            print("teeeeees:" + str(euler_from_matrix(self.r,'rxyz')))
            #self.r = euler_matrix(0, 0, np.deg2rad(-90), 'rxyz')
            
            t = np.dot(self.r,t)

            self.aruco_ofset_mapping = [[self.uav_offset.pose.position.x, t[0]],
                                        [self.uav_offset.pose.position.y, t[1]],
                                        [self.uav_offset.pose.position.z, t[2]],
                                        [angle_uav[2], angle_aruco[2]]]
            
            self.aruco_offset_init = False
            self.uav_offset_init = False

        
        angle = euler_from_quaternion([aruco_pose.pose.orientation.x,
                               aruco_pose.pose.orientation.y,
                               aruco_pose.pose.orientation.z,
                               aruco_pose.pose.orientation.w])
        
        
        new_yaw = (aruco_ofset_mapping[3][0] - (aruco_ofset_mapping[3][1] - angle[2]))
        #print("NEW YAW: " + str(new_yaw) + " " + str(aruco_ofset_mapping[3][0]) + " " + str(aruco_ofset_mapping[3][1]) + " " + str(angle[2]))
        
        self.r = euler_matrix(0, 0, np.deg2rad(-90), 'rxyz')
        t = np.array([aruco_pose.pose.position.x,
                      aruco_pose.pose.position.y,
                      aruco_pose.pose.position.z, 1])
        #print(t)
        t = np.dot(self.r,t)

        new_pose = PoseStamped()
        new_pose.pose.position.x = aruco_ofset_mapping[0][0] - (aruco_ofset_mapping[0][1] - t[0])
        new_pose.pose.position.y = aruco_ofset_mapping[1][0] - (aruco_ofset_mapping[1][1] - t[1])
        new_pose.pose.position.z = aruco_ofset_mapping[2][0] - (aruco_ofset_mapping[2][1] - t[2])
        new_pose.pose.orientation = Quaternion(*quaternion_from_euler(angle[0], angle[1], new_yaw,'rxyz'))
        
        
        #new_pose = PoseStamped()
        #new_pose.pose.position.x = aruco_ofset_mapping[0][0] - (aruco_ofset_mapping[0][1] - aruco_pose.pose.position.x)
        #new_pose.pose.position.y = aruco_ofset_mapping[1][0] - (aruco_ofset_mapping[1][1] - aruco_pose.pose.position.y)
        #new_pose.pose.position.z = aruco_ofset_mapping[2][0] - (aruco_ofset_mapping[2][1] - aruco_pose.pose.position.z)
        #new_pose.pose.orientation = Quaternion(*quaternion_from_euler(angle[0], angle[1], new_yaw,'rxyz'))
        
        
        
        #new_pose = PoseStamped()
        #new_pose.pose.position.x = aruco_ofset_mapping[0][0] - (aruco_ofset_mapping[0][1] - aruco_pose.pose.position.x)
        #new_pose.pose.position.y = aruco_ofset_mapping[1][0] - (aruco_ofset_mapping[1][1] - aruco_pose.pose.position.y)
        #new_pose.pose.position.z = aruco_ofset_mapping[2][0] - (aruco_ofset_mapping[2][1] - aruco_pose.pose.position.z)
        ####new_pose.pose.orientation = aruco_pose.pose.orientation
        #new_pose.pose.orientation = Quaternion(*quaternion_from_euler(angle[0], angle[1], angle[2],'rxyz'))
        

        return new_pose
    """

    def map_GPS_pose_to_vision(self, aruco_pose, uav_pose):

        #This method maps the GPS pose to that of the aruco board estimate.
        #This insures that the drone follows the same coordinate system when going
        #from GPS to vision which insures a smooth transition between GPS2Vision

        #Get euler angles and rotation matrix
        angle_uav = euler_from_quaternion([uav_pose.pose.orientation.x,
                                           uav_pose.pose.orientation.y,
                                           uav_pose.pose.orientation.z,
                                           uav_pose.pose.orientation.w])

        angle_aruco = euler_from_quaternion([aruco_pose.pose.orientation.x,
                                             aruco_pose.pose.orientation.y,
                                             aruco_pose.pose.orientation.z,
                                             aruco_pose.pose.orientation.w])

        r_uav = quaternion_matrix([uav_pose.pose.orientation.x,
                                   uav_pose.pose.orientation.y,
                                   uav_pose.pose.orientation.z,
                                   uav_pose.pose.orientation.w])


        r_aruco = quaternion_matrix([aruco_pose.pose.orientation.x,
                                     aruco_pose.pose.orientation.y,
                                     aruco_pose.pose.orientation.z,
                                     aruco_pose.pose.orientation.w])

        #Find the rotation between the aruco board and that of the drone to map 
        #vision to the original coordinate system used by the drone 
        self.r = np.matmul(r_aruco.T, r_uav)
        t = np.array([aruco_pose.pose.position.x,
                      aruco_pose.pose.position.y,
                      aruco_pose.pose.position.z, 1])
        t = np.dot(self.r,t)

        #Initialize offset between original drone pose and aruco board 
        self.GPS2Vision_offset = [[uav_pose.pose.position.x, t[0]],
                                  [uav_pose.pose.position.y, t[1]],
                                  [uav_pose.pose.position.z, t[2]],
                                  [angle_uav[2], angle_aruco[2]]]

        self.aruco_offset_init = False
        self.uav_offset_init = False
        self.use_GPS2Vision_offset = True
    
    def calculate_GPS2Vision_offset(self, GPS2Vision_offset, aruco_pose, calculate_setpoint = False):

        #Get current aruco orientation 
        angle = euler_from_quaternion([aruco_pose.pose.orientation.x,
                                       aruco_pose.pose.orientation.y,
                                       aruco_pose.pose.orientation.z,
                                       aruco_pose.pose.orientation.w])

        #Get new yaw from the GPS2Vision offset and aruco yaw estimate 
        yaw = (GPS2Vision_offset[3][0] - (GPS2Vision_offset[3][1] - angle[2]))

        #Calculate the aruco pos relative to the original GPS coordinate system
        t = np.array([aruco_pose.pose.position.x,
                      aruco_pose.pose.position.y,
                      aruco_pose.pose.position.z, 1])
        t = np.dot(self.r,t)

        #Get new transformed pose using GPS2Vision offset and current aruco pos estimate
        pose = PoseStamped()
        pose.pose.position.x = GPS2Vision_offset[0][0] - (GPS2Vision_offset[0][1] - t[0])
        pose.pose.position.y = GPS2Vision_offset[1][0] - (GPS2Vision_offset[1][1] - t[1])
        pose.pose.position.z = GPS2Vision_offset[2][0] - (GPS2Vision_offset[2][1] - t[2])
        
        if calculate_setpoint:
            pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, yaw,'rxyz'))
        else:
            pose.pose.orientation = Quaternion(*quaternion_from_euler(angle[0], angle[1], yaw,'rxyz'))

        return pose

    def waypoint_check(self, uav_pose, setpoint, threshold=0.25):

        angle = euler_from_quaternion([setpoint.pose.orientation.x,
                                       setpoint.pose.orientation.y,
                                       setpoint.pose.orientation.z,
                                       setpoint.pose.orientation.w])

        setpoint_x = setpoint.pose.position.x
        setpoint_y = setpoint.pose.position.y
        setpoint_z = setpoint.pose.position.z
        setpoint_yaw = angle[2]
        
        delta_x = uav_pose.pose.position.x - setpoint_x
        delta_y = uav_pose.pose.position.y - setpoint_y
        delta_z = uav_pose.pose.position.z - setpoint_z

        ori =  euler_from_quaternion([uav_pose.pose.orientation.x,
                                      uav_pose.pose.orientation.y,
                                      uav_pose.pose.orientation.z,
                                      uav_pose.pose.orientation.w])

        delta_yaw = ori[2]-setpoint_yaw
        error = np.sqrt(np.power(delta_x,2) + np.power(delta_y,2) + np.power(delta_z,2) + np.power(delta_yaw,2))
        #print(error)

        if error < threshold:
            self.pub_waypoint_check.publish(False)
        else:
            self.pub_waypoint_check.publish(True)
        
    def run(self):
        while not rospy.is_shutdown():
            self.message_control()
            self.rate.sleep()

if __name__ == "__main__":
    dc = drone_control()
    dc.run()
