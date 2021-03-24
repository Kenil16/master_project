#!/usr/bin/env python2

from std_msgs.msg import String, Int8, Float32, Bool
from os import sys
from random import randrange, uniform
from uav_flight_modes import*
from log_data import* 
from transformations_calculations import*
from waypoint_checking import*

class autonomous_flight():

    def __init__(self):        
        
        #Arguments for which mission plan to load 
        self.args = sys.argv

        #Initiate node and timer
        rospy.init_node('autonomous_flight')
        self.rate = rospy.Rate(10)

        #Init flight modes and log data scripts
        self.flight_mode = flight_modes()
        self.log_data = log_data()
        self.tc = transformations_calculations()
        self.wc = waypoint_checking()
        
        #Variables for scrip
        self.uav_local_pose = PoseStamped()
        self.uav_local_setpoint = PoseStamped()
        self.uav_state = None
        self.uav_home_pose = PoseStamped()
        self.init_uav_home_pose = False
        self.aruco_pose = PoseStamped()
        self.aruco_board_found = Bool()
        self.wind_offset = Float32()
        self.aruco_marker_pose_stable = False
        self.aruco_marker_board_center = PoseStamped()
        self.waypoint_check_pose_error = 0.25
        self.aruco_offset = PoseStamped()
        self.uav_offset = PoseStamped()
        self.dis_to_GPS2Vision_marker = 100. #Init to big number 
        
        #used only in simulation
        self.ground_truth = Odometry()
        
        #Waypoints for routes to landing station one, two and three
        self.landing_station_one_waypoints = [[[3.65, 1.10, 1.5, 90], [3.65, 4.25, 1.5, 90], [3.65, 4.25, 1.5, 90],
                                               [0.40, 4.25, 1.5, 90], [0.40, 7.10, 1.5, 90]],
                                              [[0.40, 7.10, 1.5, 90], [0.40, 4.25, 1.5, 90], [3.65, 4.25, 1.5, 90],
                                               [3.65, 4.25, 1.5, 90], [3.65, 1.10, 1.5, 90]]]

        self.landing_station_two_waypoints = [[[3.65, 1.10, 1.5, 90], [3.65, 7.10, 1.5, 90]],
                                              [[3.65, 7.10, 1.5, 90], [3.65, 1.10, 1.5, 90]]]

        self.landing_station_three_waypoints = [[[3.65, 1.10, 1.5, 90], [3.65, 4.25, 1.5, 90], [3.65, 4.25, 1.5, 90],
                                                 [7.10, 4.25, 1.5, 90], [7.10, 7.10, 1.5, 90]],
                                                [[7.10, 7.10, 1.5, 90], [7.10, 4.25, 1.5, 90], [3.65, 4.25, 1.5, 90],
                                                 [3.65, 4.25, 1.5, 90], [3.65, 1.10, 1.5, 90]]]

        #Variable to keep track of the used landing station 
        self.landing_station = 3
        
        #Publishers
        self.pub_setpoint = rospy.Publisher('/onboard/setpoint/autonomous_flight', PoseStamped, queue_size=1)
        self.pub_state = rospy.Publisher('/onboard/state', String, queue_size=10)
        self.pub_aruco_board = rospy.Publisher('/onboard/aruco_board', Int8, queue_size=1)
        self.pub_aruco_offset = rospy.Publisher('/onboard/aruco_offset', PoseStamped, queue_size=1)
        self.pub_uav_offset = rospy.Publisher('/onboard/uav_offset', PoseStamped, queue_size=1)
        self.pub_wind_offset = rospy.Publisher('/wind/offset', Float32, queue_size=1)
        
        #Subscribers
        rospy.Subscriber('/onboard/state', String, self.uav_state_callback)
        rospy.Subscriber('/onboard/aruco_marker_pose', PoseStamped, self.aruco_pose_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_position_callback)
        rospy.Subscriber('/onboard/aruco_board_found', Bool, self.aruco_board_found_callback)
        rospy.Subscriber('/onboard/aruco_marker_pose_stable', Bool, self.aruco_marker_pose_stable_callback)
        rospy.Subscriber('/onboard/aruco_marker_board_center', PoseStamped, self.aruco_marker_board_center_callback)
        rospy.Subscriber('/odom', Odometry, self.ground_truth_callback)
        rospy.Subscriber('/onboard/dis_to_GPS2Vision_marker', Float32, self.dis_to_GPS2Vision_marker_callback)

        #Initialize uav flightmodes
        self.flight_mode.wait_for_topics(60)
        
    def local_position_callback(self, msg):
        self.uav_local_pose = msg
        if not self.init_uav_home_pose:
            self.uav_home_pose = self.uav_local_pose
            self.init_uav_home_pose = True
        
    def ground_truth_callback(self, data):
        self.ground_truth = data

    def dis_to_GPS2Vision_marker_callback(self,msg):
        self.dis_to_GPS2Vision_marker = msg.data

    def aruco_board_found_callback(self, msg):
        self.aruco_board_found = msg.data

    def aruco_marker_pose_stable_callback(self,msg):
        self.aruco_marker_pose_stable = msg.data

    def aruco_marker_board_center_callback(self,msg):
        self.aruco_marker_board_center = msg

    #Fuction for updating onboard state
    def set_state(self, state):
        self.sys_state = state
        self.pub_state.publish(state)
        rospy.loginfo('Autonomous_flight: state = {}'.format(state))

    def uav_state_callback(self, msg):
        self.uav_state = msg.data

    def aruco_pose_callback(self,msg):
        self.aruco_pose = msg

    def pub_msg(self, msg, topic, frame_id_gps=True):
        
        if frame_id_gps:
            msg.header.frame_id = "att_pose_gps"
        else:
            msg.header.frame_id = "att_pose_vision"
        
        msg.header.stamp = rospy.Time.now()
        topic.publish(msg)

    """Methods for navigation the drone in offboard mode for takeoff and landing"""
    def drone_takeoff(self, alt=1.5):
        
        self.flight_mode.set_arm(True,5)

        pre_pose = self.uav_local_pose
        pre_pose.pose.position.z = alt
        rospy.loginfo('Autonomous_flight: Takeoff altitude = {} m'.format(pre_pose.pose.position.z))

        for i in range(0,5):
            self.pub_msg(pre_pose, self.pub_setpoint)
        
        #self.flight_mode.set_mode('AUTO.TAKEOFF',10)
        self.flight_mode.set_mode('OFFBOARD',5)
    
        rospy.loginfo('Autonomous_flight: UAV takeoff')
        
        #Wait until takeoff has occurred
        waypoint = [self.uav_local_pose.pose.position.x, self.uav_local_pose.pose.position.y, alt, 0]
        print(pre_pose)
        while(not self.wc.waypoint_check(uav_local_pose=self.uav_local_pose, setpoint_xyzYaw = waypoint, threshold=0.20)): #Set t0 0.05 in loiter to avoid ascilations
            if self.uav_state == 'loiter' or self.uav_state == 'home':
                rospy.loginfo('Autonomous_flight: Takeoff disrupted!')
                return
            self.pub_msg(pre_pose, self.pub_setpoint)
         
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
                    self.pub_msg(pre_pose, self.pub_setpoint)
                self.flight_mode.set_mode('OFFBOARD',5)
                waypoints = [[x_cur, y_cur, alt, 0], [x_home, y_home, alt, 0], [0, 0, 0, 0]]
            else:
                waypoints = [[x_home, y_home, alt, 0], [0, 0, 0, 0]]

            rospy.loginfo('Autonomous_flight: UAV returning home')
        
            for waypoint in waypoints:

                pre_pose = PoseStamped()
                pre_pose.pose.position.x = waypoint[0]
                pre_pose.pose.position.y = waypoint[1]
                pre_pose.pose.position.z = waypoint[2]

                #wait until waypoint reached
                while(not self.wc.waypoint_check(uav_local_pose=self.uav_local_pose, setpoint_poseStamped=pre_pose)):
                    self.pub_msg(pre_pose, self.pub_setpoint)
                    
            rospy.loginfo('Autonomous_flight: UAV has returned home')
            self.set_state('idle')
        else:
            rospy.loginfo('Autonomous_flight: UAV already at home position')
            self.set_state('idle')

    """Methods for testing the autonomous flight system"""
    def GPS2Vision_aruco_pose_estimation_test(self):

        #Save points as the home position of the drone 
        x = self.uav_home_pose.pose.position.x
        y = self.uav_home_pose.pose.position.y
        
        alt_ = 2.5
        self.drone_takeoff(alt = alt_)
        
        #UAV valocity
        self.flight_mode.set_param('MPC_XY_VEL_MAX', 2.5, 5)
        self.flight_mode.set_param('MPC_Z_VEL_MAX_DN', 1.0, 5)
        self.flight_mode.set_param('MPC_Z_VEL_MAX_UP', 1.0, 5)
        
        self.set_state('GPS2Vision_aruco_pose_estimation_test')
        rospy.loginfo('Autonomous_flight: GPS2Vision aruco pose_estimation test startet')

        #Generate the grid struture of waypoints to the drone for testing
        waypoints = self.log_data.generate_waypoints(x, y, alt_)

        #Now move the drone between the waypoints and write the error between aruco poses estimates and ground truth
        for waypoint in waypoints:
            while(not self.wc.waypoint_check(uav_local_pose=self.uav_local_pose,setpoint_poseStamped = waypoint)):
                self.pub_msg(waypoint, self.pub_setpoint)

            #Make each calculation of error be run for x seconds 
            start_time = rospy.get_rostime()
            timeout = rospy.Duration(2) #Test each position for x seconds     
            seq = 0 #Only update when new aruco pose estimates arrives 

            while (rospy.get_rostime() - start_time) < timeout:
                if not seq == self.aruco_pose.header.seq:
                    self.log_data.calculate_error_pose(self.aruco_pose, self.ground_truth)
                seq = self.aruco_pose.header.seq
                self.pub_msg(waypoint, self.pub_setpoint)
            
            #Now find mean error at giving position and write 
            x = waypoint.pose.position.x
            y = waypoint.pose.position.y
            self.log_data.write_GPS2Vision_marker_detection_data(x, y)
        
        #Return home
        self.set_state('home')
        self.drone_return_home()

        rospy.loginfo('Autonomous_flight: GPS2Vision aruco pose estimation test complete')
        self.set_state('loiter')

    def GPS2Vision_test(self):

        self.drone_takeoff(alt = 2.5)
        self.set_state('GPS2Vision_test')
        rospy.loginfo('Autonomous_flight: GPS2Vision test started!')
        
        #Location for entrance to landing stations when drone starts at (-16, 0, 2.5) in the simulation
        entrance_x = 10
        entrance_y = 0
        entrance_z = 2.5

        #Test for x times 
        for _ in range(10):
            #Random error in x, y and z to illustrate uncertainty in GPS 
            new_x = entrance_x + uniform(-2, 2)
            new_y = entrance_y + uniform(-2, 2)
            new_z = entrance_z + uniform(-1, 1)

            print(str(new_x) + " " + str(new_y) + " " + str(new_z))

            self.GPS_navigation(waypoints_xyzYaw=[[new_x, new_y, new_z, 0]])
            self.GPS2Vision_navigation()
            self.vision_navigation(waypoints_xyzYaw=[[3.65, 1.10, 1.5, 90]])
        
        rospy.loginfo('Autonomous_flight: GPS2Vision test complete')
        
        self.flight_mode.set_param('EKF2_AID_MASK', 1, 5)
        self.flight_mode.set_param('EKF2_HGT_MODE', 0, 5)
        self.set_state('loiter')

    def hold_aruco_pose_test(self):

        self.drone_takeoff(alt = 2.5)

        self.set_state('hold_aruco_pose_test')
        rospy.loginfo('Autonomous_flight: Hold position by using the aruco marker test startet')
        
        self.GPS_navigation(waypoints_xyzYaw=[[0, 0, 2.5, 0]])
        self.GPS2Vision_navigation()

        self.pub_aruco_board.publish(1)
        
        #Make the test continue for x seconds 
        start_time = rospy.get_rostime()
        timeout = rospy.Duration(20.0)
        
        setpoint = [3.65, 1.10, 2.50, 0, 0, 90] #x, y, z, roll, pitch, yaw
        #setpoint = [7.1, 7.1, 2.50, 0, 0, 90] #x, y, z, roll, pitch, yaw
        #setpoint = [3.65, -1, 2.50, 0, 0, 90] #x, y, z, roll, pitch, yaw
        
        pose = PoseStamped()
        pose.pose.position.x = setpoint[0]
        pose.pose.position.y = setpoint[1]
        pose.pose.position.z = setpoint[2]
        pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, np.deg2rad(setpoint[5]),'rxyz'))

        pose = self.tc.calculate_GPS2Vision_offset(self.tc.GPS2Vision_offset, pose, True)
        seq = 0 #Only write when new updates from aruco pose arives 
        while (rospy.get_rostime() - start_time) < timeout:
            self.fly_route(waypoints_poseStamped=[pose])
            if not seq == self.aruco_pose.header.seq:
                self.log_data.write_hold_pose_using_aruco_pose_estimation_data(self.aruco_pose, setpoint[0], setpoint[1], setpoint[2], setpoint[3], setpoint[4], setpoint[5], self.ground_truth)
            seq = self.aruco_pose.header.seq

        self.flight_mode.set_param('EKF2_AID_MASK', 1, 5)
        self.flight_mode.set_param('EKF2_HGT_MODE', 0, 5)
        
        rospy.loginfo('Autonomous_flight: Hold position by using the aruco marker test complete!')
        self.set_state('loiter')
    
    def vision_navigation_test(self):

        self.drone_takeoff(alt = 2.5)

        self.set_state('vision_navigation_test')
        rospy.loginfo('Autonomous_flight: Vision navigation test startet')

        self.GPS_navigation(waypoints_xyzYaw=[[0, 0, 2.5, 0]])
        self.GPS2Vision_navigation()
        self.vision_navigation(waypoints_xyzYaw=self.landing_station_one_waypoints[0])

        self.flight_mode.set_param('EKF2_AID_MASK', 1, 5)
        self.flight_mode.set_param('EKF2_HGT_MODE', 0, 5)

        rospy.loginfo('Autonomous_flight: Vision navigation test complete!')
        self.set_state('loiter')

    def landing_test(self):

        rospy.loginfo('Autonomous_flight: Landing test startet')
        
        #Waypoints for routes between landing stations one, two and three
        landing_station_one2two = [[0.40, 7.10, 1.5, 90], [0.40, 4.25, 1.5, 90], [3.65, 4.25, 1.5, 90], [3.65, 7.10, 1.5, 90]]
        landing_station_one2three = [[0.40, 7.10, 1.5, 90], [0.40, 4.25, 1.5, 90], [7.10, 4.25, 1.5, 90], [7.10, 7.10, 1.5, 90]]
        landing_station_one_moves = [landing_station_one2two, landing_station_one2three]

        landing_station_two2one = [[3.65, 7.10, 1.5, 90], [3.65, 4.25, 1.5, 90], [0.40, 4.25, 1.5, 90], [0.40, 7.10, 1.5, 90]]
        landing_station_two2three = [[3.65, 7.10, 1.5, 90], [3.65, 4.25, 1.5, 90], [7.10, 4.25, 1.5, 90], [7.10, 7.10, 1.5, 90]]
        landing_station_two_moves = [landing_station_two2one, landing_station_two2three]

        landing_station_three2one = [[7.10, 7.10, 1.5, 90], [7.10, 4.25, 1.5, 90], [0.40, 4.25, 1.5, 90], [0.40, 7.10, 1.5, 90]]
        landing_station_three2two = [[7.10, 7.10, 1.5, 90], [7.10, 4.25, 1.5, 90], [3.65, 4.25, 1.5, 90], [3.65, 7.10, 1.5, 90]]
        landing_station_three_moves = [landing_station_three2one, landing_station_three2two]
        
        #Let starting location be at landing station 3
        self.landing_station = 3

        #Test the landing precision for x times 
        for _ in range(5):
            
            #Make random move to a landing station
            move = randrange(0,2)

            #Moves to pick for a specifig landing station
            new_landing_station = 0
            if self.landing_station == 3:
                waypoints = landing_station_one_moves[move]
                if not move:
                    new_landing_station = 4
                else:
                    new_landing_station = 5
            
            if self.landing_station == 4:
                waypoints = landing_station_two_moves[move]
                if not move:
                    new_landing_station = 3
                else:
                    new_landing_station = 5
            
            if self.landing_station == 5:
                waypoints = landing_station_three_moves[move]
                if not move:
                    new_landing_station = 3
                else:
                    new_landing_station = 4

            #Then takeoff, move and land
            self.vision_takeoff_navigation(self.landing_station)
            self.vision_navigation(waypoints_xyzYaw=waypoints)
            self.vision_landing_navigation(new_landing_station)
            
            #Wait for x seconds to start new takeoff and landing 
            start_time = rospy.get_rostime()
            timeout = rospy.Duration(4.0)
            while (rospy.get_rostime() - start_time) < timeout:
                pass

        rospy.loginfo('Autonomous_flight: Landing test complete!')
        self.set_state('vision_landed')

    """Methods to be used for navigating the drone in offboard mode"""
    def move2GPS_locations_from_vision(self):
        
        rospy.loginfo('Autonomous_flight: Drone navigation from vision to GPS locations started!')

        self.vision2GPS_navigation()
        #self.fly_route(waypoints_xyzYaw=[[-8, 0, 2.5, 0],[-8, -5, 2.5, 0]])
        
        rospy.loginfo('Autonomous_flight: Drone navigation from vision to GPS locations completed!')
        self.set_state('loiter')
    
    def return_to_landing_station_one(self):

        self.drone_takeoff(alt = 2.5)
        self.set_state('return_to_landing_station_one')
        rospy.loginfo('Autonomous_flight: Returning to landing station one!')

        self.GPS_navigation(waypoints_xyzYaw=[[8, 0, 2.5, 0]])
        self.GPS2Vision_navigation()
        self.vision_navigation(waypoints_xyzYaw=self.landing_station_one_waypoints[0])
        self.vision_landing_navigation(3)

        self.landing_station = 3

        rospy.loginfo('Autonomous_flight: Drone returned to landing station one complete')
        self.set_state('vision_landed')
    
    def return_to_landing_station_two(self):

        self.drone_takeoff(alt = 2.5)
        self.set_state('return_to_landing_station_two')
        rospy.loginfo('Autonomous_flight: Returning to landing station two!')

        self.GPS_navigation(waypoints_xyzYaw=[[8, 0, 2.5, 0]])
        self.GPS2Vision_navigation()
        self.vision_navigation(waypoints_xyzYaw=self.landing_station_two_waypoints[0])
        self.vision_landing_navigation(4)
        
        self.landing_station = 4

        rospy.loginfo('Autonomous_flight: Drone returned to landing station two complete')
        self.set_state('vision_landed')


    def return_to_landing_station_three(self):

        self.drone_takeoff(alt = 2.5)
        self.set_state('return_to_landing_station_three')
        rospy.loginfo('Autonomous_flight: Returning to landing station three!')

        self.GPS_navigation(waypoints_xyzYaw=[[8, 0, 2.5, 0]])
        self.GPS2Vision_navigation()
        self.vision_navigation(waypoints_xyzYaw=self.landing_station_three_waypoints[0])
        self.vision_landing_navigation(5)

        self.landing_station = 5

        rospy.loginfo('Autonomous_flight: Drone returned to landing station three complete')
        self.set_state('vision_landed')

    """Methods used in autonomous flight for giving substate of drone"""
    def GPS_navigation(self, waypoints_poseStamped = None, waypoints_xyzYaw = None):

        #This methods navigates the drone between waypoints using GPS
        rospy.loginfo('Autonomous_flight: GPS navigation started!')

        #Set to use GPS and barometer in the EKF2
        self.flight_mode.set_param('EKF2_AID_MASK', 1, 5)
        self.flight_mode.set_param('EKF2_HGT_MODE', 0, 5)
        #Set maximum horizontal and vertical volocities 
        self.flight_mode.set_param('MPC_XY_VEL_MAX', 2.0, 5)
        self.flight_mode.set_param('MPC_Z_VEL_MAX_DN', 1.0, 5)
        self.flight_mode.set_param('MPC_Z_VEL_MAX_UP', 1.0, 5)
        #Set maximum angular velocities
        self.flight_mode.set_param('MC_ROLLRATE_MAX', 90.0, 5)
        self.flight_mode.set_param('MC_PITCHRATE_MAX', 90.0, 5)
        self.flight_mode.set_param('MC_YAWRATE_MAX', 135.0, 5)

        self.fly_route(waypoints_poseStamped, waypoints_xyzYaw)

        rospy.loginfo('Autonomous_flight: GPS navigation complete!')

    def GPS2Vision_navigation(self):

        #This methods performs a smooth transition between using GPS to camera based navigation (vision)
        rospy.loginfo('Autonomous_flight: GPS2Vision started!')

        #Using data from front camera 
        self.pub_aruco_board.publish(1)

        #Locate marker board in the image using the front camera 
        marker_board_found = self.locate_marker_board(self.uav_local_pose, 0, 5, 1, 0, 1, 1, -45, 90, 45)

        if not marker_board_found:
            rospy.loginfo('Autonomous_flight: Critical error! Marker board NOT found - GPS2Vision!')
            return

        #Navigate the drone to the marker board and switch only if the marker board pose estimation is good
        
        #Set maximum horizontal and vertical volocities
        self.flight_mode.set_param('MPC_XY_VEL_MAX', 1.0, 5)
        self.navigate_to_marker_board()

        #Now make transformation of the aruco pose to that of the GPS to insure a smooth GPS2Vision transition
        self.tc.map_GPS_pose_to_vision(self.aruco_pose, self.uav_local_pose)
        self.pub_uav_offset.publish(self.uav_local_pose)
        self.pub_aruco_offset.publish(self.aruco_pose)

        #Set maximum horizontal and vertical volocities
        self.flight_mode.set_param('MPC_XY_VEL_MAX', 1.0, 5)
        self.flight_mode.set_param('MPC_Z_VEL_MAX_DN', 0.5, 5)
        self.flight_mode.set_param('MPC_Z_VEL_MAX_UP', 0.5, 5)
        #Set maximum angular velocities
        self.flight_mode.set_param('MC_ROLLRATE_MAX', 90.0, 5)
        self.flight_mode.set_param('MC_PITCHRATE_MAX', 90.0, 5)
        self.flight_mode.set_param('MC_YAWRATE_MAX', 135.0, 5)
        #Set to use vision as pose estimate
        self.flight_mode.set_param('EKF2_AID_MASK', 24, 5)
        self.flight_mode.set_param('EKF2_HGT_MODE', 3, 5)

        #Giving start position of entrance in front of the GPS2Vision marker 
        pose = PoseStamped()
        pose.pose.position.x = 3.65
        pose.pose.position.y = 1.10
        pose.pose.position.z = 3.00
        pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, np.deg2rad(90),'rxyz'))
        
        pose = self.tc.calculate_GPS2Vision_offset(self.tc.GPS2Vision_offset, pose, calculate_setpoint=True)

        #Move the drone to the entrance in front of the GPS2Vision marker board 
        while(not self.wc.waypoint_check(uav_local_pose=self.uav_local_pose, setpoint_poseStamped = pose)):
            
            #Known GPS2Vision marker pos with 0.5 offset to center. This way the orientation 
            #of the drone always faces that of the GPS2Vision marker board
            delta_x = self.aruco_pose.pose.position.x - 3.2 - 0.5
            delta_y = self.aruco_pose.pose.position.y - 3.0
            
            theta = np.arctan2(delta_y, delta_x)
            if theta < 0:
                theta = theta + np.pi

            theta = (self.tc.GPS2Vision_offset[3][0] - (self.tc.GPS2Vision_offset[3][1] - theta))
            pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, theta,'rxyz'))
            self.pub_msg(pose, self.pub_setpoint, True)

        rospy.loginfo('Autonomous_flight: GPS2Vision complete!')

    def vision2GPS_navigation(self):
        
        #This substate performs a smooth transition between using vision to GPS navigation
        rospy.loginfo('Autonomous_flight: Vision2GPS started!')

        #Use the landing station where the drone is positioned
        self.pub_aruco_board.publish(self.landing_station)

        self.vision_takeoff_navigation(self.landing_station)
        
        if self.landing_station == 3:
            waypoints_xyzYaw = self.landing_station_one_waypoints[1]

        if self.landing_station == 4:
            waypoints_xyzYaw = self.landing_station_two_waypoints[1]
        
        if self.landing_station == 5:
            waypoints_xyzYaw = self.landing_station_three_waypoints[1]
        print(waypoints_xyzYaw)
        self.vision_navigation(waypoints_xyzYaw=waypoints_xyzYaw)
        self.GPS_navigation(waypoints_poseStamped=[self.uav_local_pose])

        rospy.loginfo('Autonomous_flight: Vision2GPS complete!')

    def locate_marker_board(self, uav_pose, start_x, end_x, steps_x, start_y, end_y, steps_y, start_yaw, end_yaw, steps_yaw):
        
        #This methods tries to find the GPS2Vision marker board if it can not be seen using the front camera from 
        #the last waypoint giving using GPS
        rospy.loginfo('Autonomous_flight: Locate marker board started!')

        if self.aruco_board_found:
            rospy.loginfo('Autonomous_flight: Marker board found - Locate marker board complete!')
            return True

        #Directions for initializing new places to search the GPS2Vision marker in meters if not found from last waypoint
        new_waypoints = []

        x_ = [i for i in range(start_x, end_x, steps_x)]
        y_ = [i for i in range(start_y, end_y, steps_y)]
        yaw_ = [i for i in range(start_yaw, end_yaw, steps_yaw)]
        
        #UAV pose
        uav_x = uav_pose.pose.position.x
        uav_y = uav_pose.pose.position.y
        uav_z = uav_pose.pose.position.z
        uav_angle = euler_from_quaternion([uav_pose.pose.orientation.x,
                                           uav_pose.pose.orientation.y,
                                           uav_pose.pose.orientation.z,
                                           uav_pose.pose.orientation.w])
        
        #Create new locations to search for the marker board 
        for x in x_:
            for y in y_:
                for yaw in yaw_:
                    pose = PoseStamped()
                    pose.pose.position.x = uav_x + x
                    pose.pose.position.y = uav_y + y
                    pose.pose.position.z = uav_z
                    pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, np.deg2rad(uav_angle[2] + yaw),'rxyz'))
                    new_waypoints.append(pose)

        #Search for marker board for the giving locations 
        for waypoint in new_waypoints:
            while(not self.wc.waypoint_check(uav_local_pose=self.uav_local_pose, setpoint_poseStamped = waypoint, threshold = 0.15)):
                if self.aruco_board_found:
                    rospy.loginfo('Autonomous_flight: Marker board found - Locate marker board complete!')
                    return True
                self.pub_msg(waypoint, self.pub_setpoint)

        #Marker board not found!
        rospy.loginfo('Autonomous_flight: Marker board NOT found! Locate marker board complete!')
        return False

    def navigate_to_marker_board(self):

        #This methods guides the drone to the detected marker board until the pose estimate is found stable goving a threshold.
        #The determimation  of stability is based the rolling average from the marker detection class
        rospy.loginfo('Autonomous_flight: Navigate drone to marker board started!')

        #Only orient to aruco board first time
        increment_init = True
        increment_x = 2
        increment_y = 2
        increment_z = 2
        
        pose = PoseStamped()
        min_dis = 7
        
        while not self.aruco_marker_pose_stable:
            
            while self.dis_to_GPS2Vision_marker > min_dis:

                #Simple method to keep the drone orientated towards the marker board 
                board_center_x = self.aruco_marker_board_center.pose.position.x
                board_center_y = self.aruco_marker_board_center.pose.position.y
                error = (360 - board_center_x)/360 #Because image width is 720 and normalize (0 to 1)
                #error_z = (240 - board_center_y)/240 #Because image height is 480 and normalize (0 to 1)
                angle_yaw = np.deg2rad(error*30) #Max 30 degress change in yaw angle

                angle = euler_from_quaternion([self.uav_local_pose.pose.orientation.x,
                                               self.uav_local_pose.pose.orientation.y,
                                               self.uav_local_pose.pose.orientation.z,
                                               self.uav_local_pose.pose.orientation.w])


                #Move towards the marker basked on the current orientation of the drone 
                if increment_init:
                    new_x = self.uav_local_pose.pose.position.x
                    new_y = self.uav_local_pose.pose.position.y
                    new_z = self.uav_local_pose.pose.position.z
                    increment_init = False
                else:
                    new_x = self.uav_local_pose.pose.position.x + np.cos(angle[2])*increment_x
                    new_y = self.uav_local_pose.pose.position.y + np.sin(angle[2])*increment_y
                    new_z = 2.5 #self.uav_local_pose.pose.position.z + error_z*increment_z

                pose.pose.position.x = new_x
                pose.pose.position.y = new_y
                pose.pose.position.z = new_z
                pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, angle[2]+angle_yaw,'rxyz'))

                self.pub_msg(pose, self.pub_setpoint)

            #Wait x seconds to evaluate evaluation from rolling average 
            start_time = rospy.get_rostime()
            timeout = rospy.Duration(3.0)
            while (rospy.get_rostime() - start_time) < timeout:
                if self.aruco_marker_pose_stable:
                    break
            min_dis -= 0.5

            while(not self.wc.waypoint_check(uav_local_pose=self.uav_local_pose, setpoint_poseStamped = pose, threshold = 0.25)):
                self.pub_msg(pose, self.pub_setpoint)
        
        rospy.loginfo('Autonomous_flight: Navigate drone to marker board complete!')
    
    def vision_navigation(self, waypoints_poseStamped = None, waypoints_xyzYaw = None):

        #This substate navigates the drone using vision
        rospy.loginfo('Autonomous_flight: Vision navigation started!')

        #Using data from bottom camera
        self.pub_aruco_board.publish(2)
        
        self.fly_route(waypoints_poseStamped=waypoints_poseStamped, waypoints_xyzYaw=waypoints_xyzYaw, GPS2Vision_offset=self.tc.GPS2Vision_offset)
        
        rospy.loginfo('Autonomous_flight: Vision navigation completed!')

    def vision_landing_navigation(self, landing_station):
        
        #This substate navigates the drone using vision
        rospy.loginfo('Autonomous_flight: Vision landing started!')
        
        #Using data from front camera to landing marker 
        self.pub_aruco_board.publish(landing_station)
    
        if landing_station == 3:
            waypoints_xyzYaw = [[0.4, 7.1, 1.5, 90], [0.4, 7.1, 1.5, 90]]
        elif landing_station == 4:
            waypoints_xyzYaw = [[3.75, 7.1, 1.5, 90], [3.75, 7.1, 0.2, 90]]
        elif landing_station == 5:
            waypoints_xyzYaw = [[7.1, 7.1, 1.5, 90], [7.1, 7.1, 0.2, 90]]

        self.fly_route(waypoints_xyzYaw=waypoints_xyzYaw, GPS2Vision_offset=self.tc.GPS2Vision_offset)
        self.flight_mode.set_mode('AUTO.LAND',10)

        self.landing_station = landing_station

        rospy.loginfo('Autonomous_flight: Vision landing completed!')
    
    def vision_takeoff_navigation(self, landing_station):

        #This substate navigates the drone using vision
        rospy.loginfo('Autonomous_flight: Vision takeoff started!')

        #Using data from front camera to landing marker
        self.pub_aruco_board.publish(landing_station)
        
        #See if vision is already used as pose estimation 
        ret = self.flight_mode.get_param_uav(param_id='EKF2_AID_MASK')
        if ret == None or not ret.success:
            rospy.sleep(5)

        #Arm drone and go into offboard control
        self.flight_mode.set_arm(True,5)
        self.flight_mode.set_mode('OFFBOARD',5)

        #Set to vision estimates if not enabled
        EKF2_AID_MASK = self.flight_mode.get_param('EKF2_AID_MASK')

        if not EKF2_AID_MASK.value.integer == 24:

            #Now make transformation of the aruco pose to that of the GPS to insure a smooth GPS2Vision transition
            self.pub_uav_offset.publish(self.uav_local_pose)
            self.pub_aruco_offset.publish(self.aruco_pose)

            self.tc.map_GPS_pose_to_vision(self.aruco_pose, self.uav_local_pose)
            
            #Set maximum horizontal and vertical volocities
            self.flight_mode.set_param('MPC_XY_VEL_MAX', 1.0, 5)
            self.flight_mode.set_param('MPC_Z_VEL_MAX_DN', 1.0, 5)
            self.flight_mode.set_param('MPC_Z_VEL_MAX_UP', 1.0, 5)
            #Set maximum angular velocities
            self.flight_mode.set_param('MC_ROLLRATE_MAX', 90.0, 5)
            self.flight_mode.set_param('MC_PITCHRATE_MAX', 90.0, 5)
            self.flight_mode.set_param('MC_YAWRATE_MAX', 135.0, 5)
            #Set to use vision as pose estimate
            self.flight_mode.set_param('EKF2_AID_MASK', 24, 5)
            self.flight_mode.set_param('EKF2_HGT_MODE', 3, 5)

        if landing_station == 3:
            waypoints_xyzYaw = [[0.4, 7.1, 1.5, 90]]
        elif landing_station == 4:
            waypoints_xyzYaw = [[3.75, 7.1, 1.5, 90]]
        elif landing_station == 5:
            waypoints_xyzYaw = [[7.1, 7.1, 1.5, 90]]
        self.fly_route(waypoints_xyzYaw=waypoints_xyzYaw, GPS2Vision_offset=self.tc.GPS2Vision_offset)

        rospy.loginfo('Autonomous_flight: Vision takeoff completed!')

    def fly_route(self, waypoints_poseStamped = None, waypoints_xyzYaw = None, GPS2Vision_offset = None):

        #List of waypoints giving as PoseStamped
        frame_id_gps = True
        if not waypoints_poseStamped == None:

            for waypoint in waypoints_poseStamped:
                if not GPS2Vision_offset == None:
                    waypoint = self.tc.calculate_GPS2Vision_offset(GPS2Vision_offset, waypoint, True)
                    frame_id_gps = False
                while(not self.wc.waypoint_check(uav_local_pose=self.uav_local_pose, setpoint_poseStamped=waypoint, threshold = self.waypoint_check_pose_error)):
                    self.pub_msg(waypoint, self.pub_setpoint, frame_id_gps)
        
        #List of waypoints giving as [x,y,z,yaw]
        else:
            for waypoint in waypoints_xyzYaw:
                pose = PoseStamped()
                pose.pose.position.x = waypoint[0]
                pose.pose.position.y = waypoint[1]
                pose.pose.position.z = waypoint[2]
                pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, np.deg2rad(waypoint[3]),'rxyz'))
                
                if not GPS2Vision_offset == None:
                    pose = self.tc.calculate_GPS2Vision_offset(GPS2Vision_offset, pose, True)
                    frame_id_gps = False
                while(not self.wc.waypoint_check(uav_local_pose=self.uav_local_pose, setpoint_poseStamped=pose, threshold = self.waypoint_check_pose_error)):
                    self.pub_msg(pose, self.pub_setpoint, frame_id_gps)

    def run(self):
        while not rospy.is_shutdown():
            #For takeoff and return to home position
            if self.uav_state == 'takeoff':
                self.drone_takeoff()
            elif self.uav_state == 'home':
                self.drone_return_home()
            #For tesing the autonomous flight system
            elif self.uav_state == 'GPS2Vision_aruco_pose_estimation_test':
                self.GPS2Vision_aruco_pose_estimation_test()
            elif self.uav_state == 'hold_aruco_pose_test': 
                self.hold_aruco_pose_test()
            elif self.uav_state == 'GPS2Vision_test': 
                self.GPS2Vision_test()
            elif self.uav_state == 'vision_navigation_test':
                self.vision_navigation_test()
            elif self.uav_state == 'landing_test':
                self.landing_test()
            #For navigating the drone in offboard autonomous flight
            elif self.uav_state == 'move2GPS_locations_from_vision':
                self.move2GPS_locations_from_vision()
            elif self.uav_state == 'return_to_landing_station_one':
                self.return_to_landing_station_one()
            elif self.uav_state == 'return_to_landing_station_two':
                self.return_to_landing_station_two()
            elif self.uav_state == 'return_to_landing_station_three':
                self.return_to_landing_station_three()
            self.rate.sleep()

if __name__ == "__main__":
    af = autonomous_flight()
    af.run() 
