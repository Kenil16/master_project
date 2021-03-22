#!/usr/bin/env python2

from std_msgs.msg import String, Int8, Float32, Bool
from os import sys
from random import randrange, uniform
from uav_flight_modes import*
from log_data import* 

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
        self.GPS2Vision_offset = [[0,0],
                                  [0,0],
                                  [0,0],
                                  [0,0]]
        self.publish_local_pose_with_offset = False
        self.GPS2Vision_setpoint_offset = True

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
        
        self.mission = self.read_mission(self.args[1])
        #self.mission = self.read_mission('../../../../missions/mission3.txt')
        
        self.next_waypoint = PoseStamped()
        self.ground_truth = Odometry()

        self.r = euler_matrix(0, 0, np.deg2rad(-90), 'rxyz')

        #Publishers
        self.pub_setpoint = rospy.Publisher('/onboard/setpoint/autonomous_flight', PoseStamped, queue_size=1)
        self.pub_state = rospy.Publisher('/onboard/state', String, queue_size=10)
        self.pub_aruco_board = rospy.Publisher('/onboard/aruco_board', Int8, queue_size=1)
        self.pub_aruco_offset = rospy.Publisher('/onboard/aruco_offset', PoseStamped, queue_size=1)
        self.pub_uav_offset = rospy.Publisher('/onboard/uav_offset', PoseStamped, queue_size=1)
        self.pub_wind_offset = rospy.Publisher('/wind/offset', Float32, queue_size=1)
        
        #Subscribers
        rospy.Subscriber('/onboard/state', String, self.on_uav_state)
        rospy.Subscriber('/onboard/aruco_marker_pose', PoseStamped, self.on_aruco_change)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.on_position_change)
        rospy.Subscriber('/onboard/aruco_board_found', Bool, self.aruco_board_found_callback)
        rospy.Subscriber('/onboard/aruco_marker_pose_stable', Bool, self.aruco_marker_pose_stable_callback)
        rospy.Subscriber('/onboard/aruco_marker_board_center', PoseStamped, self.aruco_marker_board_center_callback)
        rospy.Subscriber('/odom', Odometry, self.ground_truth_callback)
        self.new_uav_local_pose = PoseStamped()

        #Initialize uav flightmodes
        self.flight_mode.wait_for_topics(60)
        
    def on_position_change(self, msg):
        self.uav_local_pose = msg
        if not self.init_uav_home_pose:
            print(self.uav_local_pose)
            self.uav_home_pose = self.uav_local_pose
            self.init_uav_home_pose = True
        
    def on_setpoint_change(self, msg):
        self.uav_local_setpoint = msg

    def ground_truth_callback(self, data):
        self.ground_truth = data

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

    def on_uav_state(self, msg):
        self.uav_state = msg.data

    def on_aruco_change(self,msg):
        self.aruco_pose = msg

    def pub_msg(self, msg, topic, frame_id = None):
        if frame_id == None:
            msg.header.frame_id = "att_pose"
        else:
            msg.header.frame_id = frame_id
        msg.header.stamp = rospy.Time.now()
        topic.publish(msg)
        #self.rate.sleep()

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
        while(not self.waypoint_check(setpoint_xyzYaw = waypoint, threshold=0.20)): #Set t0 0.05 in loiter to avoid ascilations
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
                while(not self.waypoint_check(setpoint_poseStamped=pre_pose)):
                    self.pub_msg(pre_pose, self.pub_setpoint)
                    
            rospy.loginfo('Autonomous_flight: UAV has returned home')
            self.set_state('idle')
        else:
            rospy.loginfo('Autonomous_flight: UAV already at home position')
            self.set_state('idle')

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
        
        self.set_state('estimate_aruco_pose_front_test')
        rospy.loginfo('Autonomous_flight: Estimate the aruco pose utilising the front camera test startet')

        #Generate the grid struture of waypoints to the drone for testing
        waypoints = self.log_data.generate_waypoints(x, y, alt_)

        #Now move the drone between the waypoints and write the error between aruco poses estimates and ground truth
        for waypoint in waypoints:
            while(not self.waypoint_check(setpoint_poseStamped = waypoint)):
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

        rospy.loginfo('Autonomous_flight: Estimate the aruco pose utilising the front camera test complete')
        self.set_state('loiter')

    def GPS2Vision_test(self):

        self.drone_takeoff(alt = 2.5)
        self.set_state('GPS2Vision_test')
        rospy.loginfo('Autonomous_flight: GPS2Vision test started!')
        
        #Location for entrance to landing stations when drone starts at (-16, 0, 2.5) in the simulation
        entrance_x = 10
        entrance_y = 0
        entrance_z = 2.5

        #Random error in x, y and z to illustrate uncertainty in GPS 
        error_x = uniform(-5, 5)
        error_y = uniform(-5, 5)
        error_z = uniform(-1, 1)

        self.GPS_navigation(waypoints_xyzYaw=[[entrance_x+error_x, entrance_y+error_y, entrance_z+error_z, 0]])
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
        timeout = rospy.Duration(80.0)
        
        setpoint = [3.65, 1.10, 2.50, 0, 0, 90] #x, y, z, roll, pitch, yaw
        pose = PoseStamped()
        pose.pose.position.x = setpoint[0]
        pose.pose.position.y = setpoint[1]
        pose.pose.position.z = setpoint[2]
        pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, np.deg2rad(setpoint[5]),'rxyz'))

        #pose = self.calculate_GPS2Vision_offset(self.GPS2Vision_offset, pose)
        
        seq = 0 #Only write when new updates from aruco pose arives 
        while (rospy.get_rostime() - start_time) < timeout:
            self.fly_route(waypoints_poseStamped=[pose], frame_id = "GPS2Vision_setpoint_offset")
            if not seq == self.aruco_pose.header.seq:
                self.log_data.write_hold_pose_using_aruco_pose_estimation_data(self.aruco_pose,setpoint[0], setpoint[1], setpoint[2], setpoint[3], setpoint[4], setpoint[5])
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
        self.vision_navigation(waypoints_xyzYaw=self.landing_station_one_waypoints)

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
        self.navigate_to_marker_board()

        #Now make transformation of the aruco pose to that of the GPS to insure a smooth GPS2Vision transition
        #self.map_GPS_pose_to_vision(self.aruco_pose, self.uav_local_pose)
        self.pub_uav_offset.publish(self.uav_local_pose)
        self.pub_aruco_offset.publish(self.aruco_pose)

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
        
        #self.pub_uav_offset.publish(self.uav_local_pose)
        #self.pub_aruco_offset.publish(self.aruco_pose)
        
        #Giving start position of entrance in front of the GPS2Vision marker 
        pose = PoseStamped()
        pose.pose.position.x = 3.65
        pose.pose.position.y = 1.10
        pose.pose.position.z = 2.36
        pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, np.deg2rad(90),'rxyz'))
        
        #pose = self.calculate_GPS2Vision_offset(self.GPS2Vision_offset, pose)

        #Move the drone to the entrance in front of the GPS2Vision marker board 
        while(not self.waypoint_check(setpoint_poseStamped = pose)):
            
            #Known GPS2Vision marker pos with 0.5 offset to center. This way the orientation 
            #of the drone always faces that of the GPS2Vision marker board
            delta_x = self.aruco_pose.pose.position.x - 3.2 - 0.5
            delta_y = self.aruco_pose.pose.position.y - 3.0
            
            theta = np.arctan2(delta_y, delta_x)
            if theta < 0:
                theta = theta + np.pi

            #theta = (self.GPS2Vision_offset[3][0] - (self.GPS2Vision_offset[3][1] - theta))
            pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, theta,'rxyz'))
            self.pub_msg(pose, self.pub_setpoint, frame_id = "GPS2Vision_setpoint_offset")

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
            while(not self.waypoint_check(setpoint_poseStamped = waypoint, threshold = 0.15)):
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

        while not self.aruco_marker_pose_stable:
            
            #Simple method to keep the drone orientated towards the marker board 
            board_center_x = self.aruco_marker_board_center.pose.position.x
            error = (360 - board_center_x)/360 #Because image width is 720 and normalize (0 to 1)
            angle_yaw = np.deg2rad(error*30) #Max 30 degress change in yaw angle

            angle = euler_from_quaternion([self.uav_local_pose.pose.orientation.x,
                                           self.uav_local_pose.pose.orientation.y,
                                           self.uav_local_pose.pose.orientation.z,
                                           self.uav_local_pose.pose.orientation.w])


            #Move towards the marker basked on the current orientation of the drone 
            if increment_init:
                new_x = self.uav_local_pose.pose.position.x
                new_y = self.uav_local_pose.pose.position.y
                increment_init = False
            else:
                new_x = self.uav_local_pose.pose.position.x + np.cos(angle[2])*increment_x
                new_y = self.uav_local_pose.pose.position.y + np.sin(angle[2])*increment_y
            new_z = self.uav_local_pose.pose.position.z

            pose = PoseStamped()
            pose.pose.position.x = new_x
            pose.pose.position.y = new_y
            pose.pose.position.z = new_z
            pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, angle[2]+angle_yaw,'rxyz'))

            while(not self.waypoint_check(setpoint_poseStamped = pose, threshold = 0.25)):
                self.pub_msg(pose, self.pub_setpoint)

            #Wait x seconds to evaluate evaluation from rolling average 
            start_time = rospy.get_rostime()
            timeout = rospy.Duration(3.0)
            while (rospy.get_rostime() - start_time) < timeout:
                if self.aruco_marker_pose_stable:
                    break
        
        rospy.loginfo('Autonomous_flight: Navigate drone to marker board complete!')
    
    def vision_navigation(self, waypoints_poseStamped = None, waypoints_xyzYaw = None):

        #This substate navigates the drone using vision
        rospy.loginfo('Autonomous_flight: Vision navigation started!')

        #Using data from bottom camera
        self.pub_aruco_board.publish(2)
        
        self.fly_route(waypoints_poseStamped=waypoints_poseStamped,  waypoints_xyzYaw=waypoints_xyzYaw, frame_id="GPS2Vision_setpoint_offset")
        
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

        self.fly_route(waypoints_xyzYaw=waypoints_xyzYaw, frame_id=" GPS2Vision_setpoint_offset")
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

            #self.map_GPS_pose_to_vision(self.aruco_pose, self.uav_local_pose)
            
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
        self.fly_route(waypoints_xyzYaw=waypoints_xyzYaw, frame_id ="GPS2Vision_setpoint_offset")

        rospy.loginfo('Autonomous_flight: Vision takeoff completed!')


    def waypoint_check(self, setpoint_poseStamped = None, setpoint_xyzYaw = None, threshold=0.25):

        if not setpoint_poseStamped == None:
            angle = euler_from_quaternion([setpoint_poseStamped.pose.orientation.x,
                                           setpoint_poseStamped.pose.orientation.y,
                                           setpoint_poseStamped.pose.orientation.z,
                                           setpoint_poseStamped.pose.orientation.w])

            setpoint_x = setpoint_poseStamped.pose.position.x
            setpoint_y = setpoint_poseStamped.pose.position.y
            setpoint_z = setpoint_poseStamped.pose.position.z
            setpoint_yaw = angle[2]
        else:
            setpoint_x = setpoint_xyzYaw[0]
            setpoint_y = setpoint_xyzYaw[1]
            setpoint_z = setpoint_xyzYaw[2]
            setpoint_yaw = np.deg2rad(setpoint_xyzYaw[3])

        delta_x = self.uav_local_pose.pose.position.x - setpoint_x
        delta_y = self.uav_local_pose.pose.position.y - setpoint_y
        delta_z = self.uav_local_pose.pose.position.z - setpoint_z

        ori =  euler_from_quaternion([self.uav_local_pose.pose.orientation.x,
                                      self.uav_local_pose.pose.orientation.y,
                                      self.uav_local_pose.pose.orientation.z,
                                      self.uav_local_pose.pose.orientation.w])

        delta_yaw = ori[2]-setpoint_yaw
        error = np.sqrt(np.power(delta_x,2) + np.power(delta_y,2) + np.power(delta_z,2) + np.power(delta_yaw,2))
        #print(error)

        if error < threshold:
            return True
        else:
            return False
    
    def fly_route(self, waypoints_poseStamped = None, waypoints_xyzYaw = None, frame_id = None):

        #List of waypoints giving as PoseStamped
        if not waypoints_poseStamped == None:

            for waypoint in waypoints_poseStamped:
                while(not self.waypoint_check(setpoint_poseStamped=waypoint, threshold = self.waypoint_check_pose_error)):
                    self.pub_msg(waypoint, self.pub_setpoint, frame_id)
        
        #List of waypoints giving as [x,y,z,yaw]
        else:
            for waypoint in waypoints_xyzYaw:
                pose = PoseStamped()
                pose.pose.position.x = waypoint[0]
                pose.pose.position.y = waypoint[1]
                pose.pose.position.z = waypoint[2]
                pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, np.deg2rad(waypoint[3]),'rxyz'))
                
                while(not self.waypoint_check(setpoint_poseStamped=pose, threshold = self.waypoint_check_pose_error)):
                    self.pub_msg(pose, self.pub_setpoint, frame_id)

    def calculate_GPS2Vision_offset(self, GPS2Vision_offset, aruco_pose):

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
        pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, yaw,'rxyz'))

        print(euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]))

        return pose

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

        #Begin publish estimated aruco board pose with uav offsets
        self.publish_local_pose_with_offset = True
        #self.pub_uav_offset.publish(uav_pose)
        #self.pub_aruco_offset.publish(aruco_pose)

    def update_mission(self):

        param = ' '
        param_value = 0
        timeout = 0
        transform = 0
        """
        if not self.mission[0][0] == '-':
            param = self.mission[0][0]

            try:
                param_value = int(self.mission[0][1])
            except ValueError:
                param_value = float(self.mission[0][1])

            timeout = int(self.mission[0][2])

            #Update parameters
            self.flight_mode.set_param(param, param_value, timeout)
        """
        if not self.mission[0][3] == '-':

            self.next_waypoint.pose.position.x = float(self.mission[0][3])
            self.next_waypoint.pose.position.y = float(self.mission[0][4])
            self.next_waypoint.pose.position.z = float(self.mission[0][5])

            angle = Quaternion(*quaternion_from_euler(
                np.deg2rad(float(self.mission[0][6])),
                np.deg2rad(float(self.mission[0][7])),
                np.deg2rad(float(self.mission[0][8]))))

            self.next_waypoint.pose.orientation = angle

        if not self.mission[0][9] == '-':
            transform = int(self.mission[0][9])
            self.pub_aruco_board.publish(transform)
            print(transform)

        if not self.mission[0][10] == '-':
            self.waypoint_check_pose_error = float(self.mission[0][10])

        self.mission.pop(0)

    def read_mission(self, file_name):

        txtFile = open(file_name,'r')
        data = txtFile.readlines()
        mission = []

        for x in data:
            x = x.split(' ')
            x.pop(-1)
            mission.append(x)

        return mission

    def run(self):
        while not rospy.is_shutdown():
            
            if self.uav_state == 'takeoff':
                self.drone_takeoff()
            elif self.uav_state == 'home':
                self.drone_return_home()
            
            elif self.uav_state == 'move2GPS_locations_from_vision':
                self.move2GPS_locations_from_vision()
            elif self.uav_state == 'return_to_landing_station_one':
                self.return_to_landing_station_one()
            elif self.uav_state == 'return_to_landing_station_two':
                self.return_to_landing_station_two()
            elif self.uav_state == 'return_to_landing_station_three':
                self.return_to_landing_station_three()

            elif self.uav_state == 'GPS2Vision_aruco_pose_estimation_test':
                self.estimate_aruco_pose_front_test()
            elif self.uav_state == 'hold_aruco_pose_test': 
                self.hold_aruco_pose_test()
            elif self.uav_state == 'GPS2Vision_test': 
                self.GPS2Vision_test()
            elif self.uav_state == 'vision_navigation_test':
                self.vision_navigation_test()
            elif self.uav_state == 'landing_test':
                self.landing_test()
            
            self.rate.sleep()

if __name__ == "__main__":
    af = autonomous_flight()
    af.run() 
