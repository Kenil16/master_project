#!/usr/bin/env python

import rospy
from  mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from std_msgs.msg import String, Int8, Bool
from geometry_msgs.msg import PoseStamped
from transformations_calculations import*

class drone_control():
    def __init__(self):
        
        #Arguments for which initial uav state to be in 
        self.arg = sys.argv

        #Init ROS
        rospy.init_node('drone_control')
        self.rate = rospy.Rate(30)
        
        #Make object to make uav2aruco offset calculations
        self.tc = transformations_calculations()

        #Variables
        self.uav_state = 'idle'         
        self.mavros_state = State()
        self.aruco_marker_pose = PoseStamped()
        self.uav_pose = PoseStamped()
        #self.autonomous_fligt_state = None
        self.sensor_fusion = PoseStamped()
        self.aruco_offset = PoseStamped()

        #Used for UAV2Aruco offset calculations 
        self.aruco_offset_init = False
        self.uav_offset = PoseStamped()
        self.uav_offset_init = False
        self.use_GPS2Vision_offset = False

        #Setpoints from nodes
        self.autonomous_flight_pose_setpoint = PoseStamped()
        self.loiterpilot_pose_setpoint = PoseStamped()

        #Subscribers
        rospy.Subscriber('/mavros/state', State, self.uav_state_callback) 
        rospy.Subscriber('/gcs/command', Int8, self.gcs_command_callback)
        rospy.Subscriber('/onboard/setpoint/autonomous_flight', PoseStamped, self.af_setpoint_callback)
        rospy.Subscriber('/onboard/state', String, self.onboard_uav_state_callback)
        rospy.Subscriber('/onboard/setpoint/loiter_pilot', PoseStamped, self.lp_setpoint_callback)
        rospy.Subscriber('/onboard/aruco_marker_pose', PoseStamped, self.aruco_pose_callback)
        rospy.Subscriber('/onboard/aruco_offset', PoseStamped, self.aruco_offset_callback)
        rospy.Subscriber('/onboard/uav_offset', PoseStamped, self.uav_offset_callback)
        rospy.Subscriber('/onboard/sensor_fusion', PoseStamped, self.sensor_fusion_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_position_callback)        
        
        #Publishers
        self.pub_state = rospy.Publisher('/onboard/state', String, queue_size=1)
        self.pub_local_pose = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.pub_vision_pose = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=1)
        self.pub_waypoint_check = rospy.Publisher('/onboard/waypoint_check', Bool, queue_size=1)

        #Services 
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

    def af_setpoint_callback(self,msg):
        self.autonomous_flight_pose_setpoint = msg

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

    def onboard_uav_state_callback(self,msg):
        self.uav_state = msg.data

    def lp_setpoint_callback(self,msg):
        self.loiterpilot_pose_setpoint = msg

    def aruco_pose_callback(self,msg):
        self.aruco_marker_pose = msg

    def uav_state_callback(self, msg):
        self.mavros_state = msg
    
    def gcs_command_callback(self, msg):
        
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

            #self.pub_msg(self.aruco_marker_pose, self.pub_vision_pose)
            
            #If wanted map aruco marker estimate to UAV globle pose
            if self.aruco_offset_init and self.uav_offset_init:
                self.tc.map_GPS_pose_to_vision(self.aruco_offset, self.uav_offset)
                self.aruco_offset_init = False
                self.uav_offset_init = False

            if self.uav_state == 'loiter':
                output_msg = self.loiterpilot_pose_setpoint
                self.pub_msg(output_msg, self.pub_local_pose)

            elif self.uav_state == 'takeoff':
                output_msg = self.autonomous_flight_pose_setpoint
                self.pub_msg(output_msg, self.pub_local_pose)
            
            elif self.uav_state == 'home':
                output_msg = self.autonomous_flight_pose_setpoint
                self.pub_msg(output_msg, self.pub_local_pose)
            
            elif self.uav_state == 'GPS2Vision_aruco_pose_estimation_test':
                output_msg = self.autonomous_flight_pose_setpoint
                self.pub_msg(output_msg, self.pub_local_pose)
            
            elif self.uav_state == 'move2GPS_locations_from_vision':
                output_msg = self.autonomous_flight_pose_setpoint
                new_pose = self.tc.calculate_GPS2Vision_offset(self.tc.GPS2Vision_offset, self.sensor_fusion)
                self.pub_msg(output_msg, self.pub_local_pose)
                self.pub_msg(new_pose, self.pub_vision_pose)
            
            elif self.uav_state == 'return_to_landing_station_one':
                output_msg = self.autonomous_flight_pose_setpoint
                new_pose = self.tc.calculate_GPS2Vision_offset(self.tc.GPS2Vision_offset, self.sensor_fusion)
                self.pub_msg(output_msg, self.pub_local_pose)
                self.pub_msg(new_pose, self.pub_vision_pose)
            
            elif self.uav_state == 'return_to_landing_station_two':
                output_msg = self.autonomous_flight_pose_setpoint
                self.pub_msg(output_msg, self.pub_local_pose)
                new_pose = self.tc.calculate_GPS2Vision_offset(self.tc.GPS2Vision_offset, self.aruco_marker_pose)
                self.pub_msg(new_pose, self.pub_vision_pose)
            
            elif self.uav_state == 'return_to_landing_station_three':
                output_msg = self.autonomous_flight_pose_setpoint
                self.pub_msg(output_msg, self.pub_local_pose)
                new_pose = self.tc.calculate_GPS2Vision_offset(self.tc.GPS2Vision_offset, self.aruco_marker_pose)
                self.pub_msg(new_pose, self.pub_vision_pose)
            
            elif self.uav_state == 'GPS2Vision_aruco_pose_estimation_test':
                output_msg = self.autonomous_flight_pose_setpoint
                self.pub_msg(output_msg, self.pub_local_pose)
            
            elif self.uav_state == 'hold_aruco_pose_test':
                output_msg = self.autonomous_flight_pose_setpoint
                self.pub_msg(output_msg, self.pub_local_pose)
                new_pose = self.tc.calculate_GPS2Vision_offset(self.tc.GPS2Vision_offset, self.aruco_marker_pose)
                self.pub_msg(new_pose, self.pub_vision_pose)

            elif self.uav_state == 'GPS2Vision_test':
                output_msg = self.autonomous_flight_pose_setpoint
                self.pub_msg(output_msg, self.pub_local_pose)
                new_pose = self.tc.calculate_GPS2Vision_offset(self.tc.GPS2Vision_offset, self.aruco_marker_pose)
                self.pub_msg(new_pose, self.pub_vision_pose)
            
            elif self.uav_state == 'vision_navigation_test':
                output_msg = self.autonomous_flight_pose_setpoint
                self.pub_msg(output_msg, self.pub_local_pose)
                new_pose = self.tc.calculate_GPS2Vision_offset(self.tc.GPS2Vision_offset, self.sensor_fusion)
                self.pub_msg(new_pose, self.pub_vision_pose)
            
            elif self.uav_state == 'landing_test':
                output_msg = self.autonomous_flight_pose_setpoint
                self.pub_msg(output_msg, self.pub_local_pose)
                new_pose = self.tc.calculate_GPS2Vision_offset(self.tc.GPS2Vision_offset, self.aruco_marker_pose)
                self.pub_msg(new_pose, self.pub_vision_pose)
            
            elif self.uav_state == 'vision_landed':
                output_msg = self.autonomous_flight_pose_setpoint
                self.pub_msg(output_msg, self.pub_local_pose)
                new_pose = self.tc.calculate_GPS2Vision_offset(self.tc.GPS2Vision_offset, self.aruco_marker_pose)
                self.pub_msg(new_pose, self.pub_vision_pose)

            if output_msg == None:
                rospy.logfatal_once("Drone control received no message: Has a pilot crashed?")
                self.set_mode(0, "AUTO.LOITER")
                rospy.loginfo('Drone_control: PX4 mode = AUTO.LOITER')
    
    def run(self):
        while not rospy.is_shutdown():
            self.message_control()
            self.rate.sleep()

if __name__ == "__main__":
    dc = drone_control()
    dc.run()
