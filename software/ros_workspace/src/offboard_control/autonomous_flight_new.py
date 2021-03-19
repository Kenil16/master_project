#!/usr/bin/env python2

##!/usr/bin/python 
from pathlib import Path
import cv2
import numpy as np
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from mavlink_msgs.msg import mavlink_lora_aruco
from uav_flight_modes import*
from geometry_msgs.msg import PoseStamped, Quaternion
from pid import*
from std_msgs.msg import (String, Int8, Float64, Float32, Bool)
from timeit import default_timer as timer
from os import sys, path

from log_data import* 

class autonomous_flight():

    def __init__(self):        
        
        #Arguments for which mission plan to load 
        self.args = sys.argv
        #print(self.args)
        rospy.init_node('autonomous_flight')
        self.rate = rospy.Rate(10)

        #Init flight modes 
        self.flight_mode = flight_modes()
        self.log_data = log_data()
        
        #Init PID controllers
        #self.pid_x = PID(0.1, 0.0, 1.7, 50, -50) #(0.8, 0.2, 1.3, 50, -50)
        #self.pid_y = PID(0.1, 0.0, 1.7, 50, -50) #(0.8, 0.2, 1.3, 50, -50)
        #self.pid_z = PID(1.0, 0.05, 1.0, 50, -50) # (0.5, 0.5, 0.5, 3, -3)
        #self.pid_yaw = PID(1.,1.,1.,1,-1)

        #Initialize objects for uav commands and status 
        self.uav_local_pose = PoseStamped()
        self.uav_local_setpoint = PoseStamped()
        self.uav_state = None
        self.uav_home_pose = PoseStamped()
        self.init_uav_home_pose = False
        self.aruco_pose = PoseStamped()
        self.next_board_found = False
        self.aruco_board_found = Bool()
        self.wind_offset = Float32()
        self.aruco_marker_pose_stable = False
        self.aruco_marker_board_center = PoseStamped()

        self.waypoint_check_pose_error = 0.25

        self.aruco_offset = PoseStamped()
        self.uav_offset = PoseStamped()

        self.delta_x = 0.0
        self.delta_y = 0.0
        self.delta_z = 0.0

        self.GPS2Vision_offset = [[0,0],
                                  [0,0],
                                  [0,0],
                                  [0,0]]
        
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

        #self.init_data_files()

        #Initialize uav flightmodes
        self.flight_mode.wait_for_topics(60)
        
    def on_position_change(self, msg):
        self.uav_local_pose = msg
        if not self.init_uav_home_pose:
            print(self.uav_local_pose)
            self.uav_home_pose = self.uav_local_pose
            self.init_uav_home_pose = True
        
        """
        angle = euler_from_quaternion([self.uav_local_pose.pose.orientation.x,
                                       self.uav_local_pose.pose.orientation.y,
                                       self.uav_local_pose.pose.orientation.z,
                                       self.uav_local_pose.pose.orientation.w])

        print(angle)
        """

    def on_setpoint_change(self, msg):
        self.uav_local_setpoint = msg
        pass

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

    def pub_msg(self, msg, topic):
        msg.header.frame_id = "att_pose"
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
                while(not self.waypoint_check(setpoint=waypoint)):
                    if not self.uav_state == 'home':
                        rospy.loginfo('Autonomous_flight: Return home disrupted!')
                        return
                    self.pub_msg(pre_pose, self.pub_setpoint)
                    
            rospy.loginfo('Autonomous_flight: UAV has returned home')
            self.set_state('idle')
        else:
            rospy.loginfo('Autonomous_flight: UAV already at home position')
            self.set_state('idle')

    def estimate_aruco_pose_front_test(self):

        #UAV valocity
        self.flight_mode.set_param('MPC_XY_VEL_MAX', 2.5, 5)
        self.flight_mode.set_param('MPC_Z_VEL_MAX_DN', 1.0, 5)
        self.flight_mode.set_param('MPC_Z_VEL_MAX_UP', 1.0, 5)

        alt_ = 2.5
        self.drone_takeoff(alt = alt_)

        self.set_state('estimate_aruco_pose_front_test')
        rospy.loginfo('Autonomous_flight: Estimate the aruco pose utilising the front camera test startet')

        x = self.uav_home_pose.pose.position.x
        y = self.uav_home_pose.pose.position.y

        #Initialize waypoints 
        waypoints = []
        dis_to_aruco_x = 2
        dx = 0
        dy = 4
        alt = alt_

        for row in range(7): #Resolution in y
            for col in range(9): #Resolution in x
                
                pose = PoseStamped()    
                pose.pose.position.x = x+dx
                pose.pose.position.y = y+dy
                pose.pose.position.z = alt

                #To be used so that the drone always facing the marker
                if (y+dy) < 0:
                    yaw = np.deg2rad(90) - np.arctan((dis_to_aruco_x/abs(y+dy)))
                elif (y+dy) > 0:
                    yaw = -1*(np.deg2rad(90) - np.arctan((dis_to_aruco_x/abs(y+dy))))
                else:
                    yaw = 0
                    
                pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, yaw))
                dy = dy - 1
                waypoints.append(pose)
            
            dy = 4
            dx = dx - 1
            dis_to_aruco_x = dis_to_aruco_x + 1

        dis_to_aruco_x = 2
        index = -1
        for waypoint in waypoints:
            print(waypoint)
            index = index + 1

            #wait until waypoint reached
            angle = euler_from_quaternion([waypoint.pose.orientation.x,
                                           waypoint.pose.orientation.y,
                                           waypoint.pose.orientation.z,
                                           waypoint.pose.orientation.w])

            while(not self.waypoint_check(setpoint = [waypoint.pose.position.x, waypoint.pose.position.y, waypoint.pose.position.z, np.rad2deg(angle[2])])):
                if not self.uav_state == 'estimate_aruco_pose_front_test':
                    rospy.loginfo('Autonomous_flight: Estimate the aruco pose utilising the front camera test disrupted!')
                    return
                self.pub_msg(waypoint, self.pub_setpoint)

            #Get ArUco position, mean and STD
            start_time = rospy.get_rostime()
            timeout = rospy.Duration(2) #Test each position for x seconds 
            
            #Pose
            aruco_x = []
            aruco_y = []
            aruco_z = []
            aruco_roll = []
            aruco_pitch = []
            aruco_yaw = []
            
            while (rospy.get_rostime() - start_time) < timeout:

                ground_truth = self.ground_truth
                #Rotation to align ground truth to aruco marker
                r_m = euler_matrix(0, 0, np.pi/2, 'rxyz')

                #Get current ground truth orientation
                t_g = quaternion_matrix(np.array([ground_truth.pose.pose.orientation.x,
                                                  ground_truth.pose.pose.orientation.y,
                                                  ground_truth.pose.pose.orientation.z,
                                                  ground_truth.pose.pose.orientation.w]))

                #Get current ground truth translation
                t_g[0][3] = ground_truth.pose.pose.position.x
                t_g[1][3] = ground_truth.pose.pose.position.y
                t_g[2][3] = ground_truth.pose.pose.position.z

                #Perform matrix multiplication for pose aligment
                T =  np.matmul(r_m, t_g)

                #Get angles and translation in degress
                g_euler = euler_from_matrix(T,'rxyz')

                #Because plugin is initiated in (0,0,0).
                g_x = T[0][3] + 7.4/2
                g_y = T[1][3] + 7.4/2
                g_z = T[2][3]

                g_roll = np.rad2deg(g_euler[0])
                g_pitch = np.rad2deg(-g_euler[1])
                g_yaw = np.rad2deg(g_euler[2])

                
                angle = euler_from_quaternion([self.aruco_pose.pose.orientation.x,
                                               self.aruco_pose.pose.orientation.y,
                                               self.aruco_pose.pose.orientation.z,
                                               self.aruco_pose.pose.orientation.w])

                error_x = abs(self.aruco_pose.pose.position.x-g_x)
                error_y = abs(self.aruco_pose.pose.position.y-g_y)
                error_z = abs(self.aruco_pose.pose.position.z-g_z)
                error_roll = abs(np.rad2deg(angle[0])-g_roll)
                error_pitch = abs(np.rad2deg(angle[1])-g_pitch)
                error_yaw = abs(np.rad2deg(angle[2])-g_yaw)

                aruco_x.append(error_x)
                aruco_y.append(error_y)
                aruco_z.append(error_z)
                aruco_roll.append(error_roll)
                aruco_pitch.append(error_pitch)
                aruco_yaw.append(error_yaw)
            
            #print(len(aruco_x))
            #print(waypoint)

            mean_x = sum(aruco_x)/len(aruco_x)
            mean_y = sum(aruco_y)/len(aruco_y)
            mean_z = sum(aruco_z)/len(aruco_z)

            mean_roll = sum(aruco_roll)/len(aruco_roll)
            mean_pitch = sum(aruco_pitch)/len(aruco_pitch)
            mean_yaw = sum(aruco_yaw)/len(aruco_yaw)

            x = waypoint.pose.position.x
            y = waypoint.pose.position.y

            self.log_data.write_GPS2Vision_marker_detection_data(x, y, mean_x, mean_y, mean_z, mean_roll, mean_pitch, mean_yaw)
        
        #Return home
        self.set_state('home')
        self.drone_return_home()

        rospy.loginfo('Autonomous_flight: Estimate the aruco pose utilising the front camera test complete')
        self.set_state('loiter')

    def GPS2Vision_test(self):

        self.mission = self.read_mission(self.args[1])

        #This test initiates the transition from using GPS to vision based navigation.
        initiate_high_speed = False
        keep_orientation2GPS2Vision_marker = True
        
        #Inialize parameters
        while not self.mission[0][0] == '-':
            self.update_mission()
        
        alt_ = 2.5
        self.drone_takeoff(alt = alt_)

        self.pub_wind_offset.publish(0)

        self.set_state('GPS2Vision_test')
        rospy.loginfo('Autonomous_flight: GPS to vision test startet')
        waypoint = []

        #Step 1 -> Move to final position using GPS 
        while not self.mission[0][3] == '-':
            
            print(self.mission[0])
            waypoint = [float(self.mission[0][3]), float(self.mission[0][4]), float(self.mission[0][5]), float(self.mission[0][8])]
            self.update_mission()

            while True:

                    self.pub_msg(self.next_waypoint, self.pub_local_pose)
                    
                    if self.waypoint_check(setpoint = [waypoint[0], waypoint[1], waypoint[2], waypoint[3]], threshold = self.waypoint_check_pose_error):
                        break
        
        #Step 2 -> Look for ArUco marker in the image from last waypoint if not found
        index = 0
        
        #Directions for initializing new places to search the GPS2Vision marker in meters if not found from last waypoint
        searching_waypoints = []
        shifts_x = [-1, -2, -3, -4, -5, -6, -7, -8]
        shifts_y = [0]
        shifts_yaw = [45, 0, -45] #This goes for each new waypoint

        for x in shifts_x:
            for y in shifts_y:
                for yaw in shifts_yaw:
                    pose = PoseStamped()
                    pose.pose.position.x = waypoint[0] + x
                    pose.pose.position.y = waypoint[1] + y
                    pose.pose.position.z = waypoint[2]
                    pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, np.deg2rad(yaw+waypoint[3]),'rxyz'))
                    searching_waypoints.append([pose, waypoint[0] + x, waypoint[1] + y, alt_, yaw + waypoint[3]])

        while not self.aruco_board_found:
            
            next_waypoint = searching_waypoints[index]
            print(next_waypoint)
            
            while(not self.waypoint_check(setpoint = [next_waypoint[1], next_waypoint[2], next_waypoint[3], next_waypoint[4]], threshold = 0.15)):
                self.pub_msg(next_waypoint[0], self.pub_setpoint)
                
            index += 1


        #Step 3 -> Now do final step by going from GPS2Vision navigation
        #Wait until aruco pose estimation has stabilized 
        print(self.aruco_marker_pose_stable)
        
        while not self.aruco_marker_pose_stable:

            board_center_x = self.aruco_marker_board_center.pose.position.x
            error = (360 - board_center_x)/360 #Because image width is 720 and normalize (0 to 1)
            angle_yaw = np.deg2rad(error*30) #Max 30 degress change in yaw angle 

            angle = euler_from_quaternion([self.uav_local_pose.pose.orientation.x,
                                           self.uav_local_pose.pose.orientation.y,
                                           self.uav_local_pose.pose.orientation.z,
                                           self.uav_local_pose.pose.orientation.w])
            


            new_x = self.uav_local_pose.pose.position.x + np.cos(angle[2])*2
            new_y = self.uav_local_pose.pose.position.y + np.sin(angle[2])*2
            new_z = self.uav_local_pose.pose.position.z

            pose = PoseStamped()
            pose.pose.position.x = new_x
            pose.pose.position.y = new_y
            pose.pose.position.z = new_z
            pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, angle[2]+angle_yaw,'rxyz'))

            while(not self.waypoint_check(setpoint = [new_x, new_y, new_z, np.rad2deg(angle[2]+angle_yaw)], threshold = 0.25)):
                self.pub_msg(pose, self.pub_setpoint)

            #wait x seconds to start high speeds
            start_time = rospy.get_rostime()
            timeout = rospy.Duration(3.0)

            while (rospy.get_rostime() - start_time) < timeout:
                if self.aruco_marker_pose_stable:
                    break


        
        rospy.loginfo('Autonomous_flight: Aruco board found and stable!')
        
        self.aruco_offset = self.aruco_pose
        self.uav_offset = self.uav_local_pose

        self.pub_uav_offset.publish(self.uav_offset)
        self.pub_aruco_offset.publish(self.aruco_offset)

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

        self.r = np.matmul(r_aruco.T, r_uav)

        #self.r = euler_matrix(0, 0, np.deg2rad(-90), 'rxyz')
        
        t = np.array([self.aruco_offset.pose.position.x,
                      self.aruco_offset.pose.position.y,
                      self.aruco_offset.pose.position.z, 1])
        print(t)
        t = np.dot(self.r,t)

        self.aruco_ofset_mapping = [[self.uav_offset.pose.position.x, t[0]],
                                    [self.uav_offset.pose.position.y, t[1]],
                                    [self.uav_offset.pose.position.z, t[2]],
                                    [angle_uav[2], angle_aruco[2]]]
        
        #self.aruco_ofset_mapping = [self.uav_offset, self.aruco_offset]
        
        #Inialize parameters
        while not self.mission[0][0] == '-':
            self.update_mission()
        
        waypoint = [float(self.mission[0][3]), float(self.mission[0][4]), float(self.mission[0][5]), float(self.mission[0][8])]

        #wait x seconds to start high speeds
        start_time = rospy.get_rostime()
        timeout = rospy.Duration(0.5) 
        
        new_pose_set = PoseStamped()
        while len(self.mission):
            print('sds')
            i = self.next_waypoint
            print(self.mission[0])
            self.update_mission()

            new_pose_set, waypoint = self.vision2local(self.aruco_ofset_mapping, i)

            while(not self.waypoint_check(setpoint = [waypoint[0], waypoint[1], waypoint[2], waypoint[3]], threshold = self.waypoint_check_pose_error)):
                
                if not initiate_high_speed and (rospy.get_rostime() - start_time) > timeout:
                    initiate_high_speed = True
                    while not self.mission[0][0] == '-':
                        self.update_mission()
                
                if keep_orientation2GPS2Vision_marker:
                    
                    delta_x = self.aruco_pose.pose.position.x - 3.2 - 0.5 #Known GPS2Visiom marker pos with 0.5 offset to center
                    delta_y = self.aruco_pose.pose.position.y - 3.0

                    theta = np.arctan2(delta_y, delta_x)
                    if theta < 0:
                        theta = theta + np.pi
                    
                    theta = (self.aruco_ofset_mapping[3][0] - (self.aruco_ofset_mapping[3][1] - theta))
                    
                    new_pose_set.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, theta,'rxyz'))

                self.pub_msg(new_pose_set, self.pub_setpoint)

            keep_orientation2GPS2Vision_marker = False


        angle = euler_from_quaternion([self.aruco_offset.pose.orientation.x,
                       self.aruco_offset.pose.orientation.y,
                       self.aruco_offset.pose.orientation.z,
                       self.aruco_offset.pose.orientation.w])

        theta = (self.aruco_ofset_mapping[3][0] - (self.aruco_ofset_mapping[3][1] - angle[2]))
        pose = PoseStamped()
        pose.pose.position.x = new_pose_set.pose.position.x
        pose.pose.position.y = new_pose_set.pose.position.y
        pose.pose.position.z = new_pose_set.pose.position.z
        pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, theta,'rxyz')) #self.aruco_offset.pose.orientation
 
        while(not self.waypoint_check(setpoint = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, np.rad2deg(theta)], threshold = 0.10)):
            self.pub_msg(pose, self.pub_setpoint)
        
        self.flight_mode.set_param('MC_ROLLRATE_MAX', 1.0, 5)
        self.flight_mode.set_param('MC_PITCHRATE_MAX', 1.0, 5)
        self.flight_mode.set_param('MC_YAWRATE_MAX', 1.0, 5)

        self.flight_mode.set_param('EKF2_AID_MASK', 1, 5)
        self.flight_mode.set_param('EKF2_HGT_MODE', 0, 5)

        #wait x seconds to start high speeds

        pose = PoseStamped()
        pose.pose.position.x = self.uav_offset.pose.position.x
        pose.pose.position.y = self.uav_offset.pose.position.y
        pose.pose.position.z = self.uav_offset.pose.position.z
        pose.pose.orientation = self.uav_offset.pose.orientation

        angle = euler_from_quaternion([pose.pose.orientation.x,
                               pose.pose.orientation.y,
                               pose.pose.orientation.z,
                               pose.pose.orientation.w])

        setpoint = [self.uav_offset.pose.position.x, self.uav_offset.pose.position.y, self.uav_offset.pose.position.z, np.rad2deg(angle[2])]
        start_time = rospy.get_rostime()
        timeout = rospy.Duration(1.0)

        while (rospy.get_rostime() - start_time) < timeout:
            self.pub_msg(pose, self.pub_setpoint)
            pass

        self.flight_mode.set_param('MC_ROLLRATE_MAX', 90.0, 5)
        self.flight_mode.set_param('MC_PITCHRATE_MAX', 90.0, 5)
        self.flight_mode.set_param('MC_YAWRATE_MAX', 90.0, 5)

        while(not self.waypoint_check(setpoint = [self.uav_offset.pose.position.x, self.uav_offset.pose.position.y, self.uav_offset.pose.position.z, np.rad2deg(angle[2])], threshold = 0.10)):
            self.pub_msg(pose, self.pub_setpoint)

        rospy.loginfo('Autonomous_flight: Follow aruco pose utilising the bottom camera test complete')
        self.set_state('loiter')


    def follow_aruco_pose_bottom_test(self):


        #self.mission = self.read_mission(self.args[1])

        self.drone_takeoff(alt = 2.5)
        self.set_state('follow_aruco_pose_bottom_test')
        rospy.loginfo('Autonomous_flight: Follow aruco pose utilising the bottom camera test startet')

        self.GPS_navigation(waypoints_xyzYaw=[[8, 0, 2.5, 0]])
        self.GPS2Vision_navigation()

        waypoints_xyzYaw = [[3.65, 1.10, 1.5, 90], [3.65, 4.25, 1.5, 90], [3.65, 4.25, 1.5, 90],
                            [0.40, 4.25, 1.5, 90], [0.40, 7.1, 1.5, 90]]

        self.vision_navigation(waypoints_xyzYaw=waypoints_xyzYaw)
        self.vision_landing_navigation(1)
        self.vision_takeoff_navigation(1)

        waypoints_xyzYaw = [[0.40, 7.10, 1.5, 90], [0.40, 4.25, 1.5, 90], [3.65, 4.25, 1.5, 90],
                            [3.65, 4.25, 1.5, 90], [3.65, 1.10, 1.5, 90]]

        self.vision_navigation(waypoints_xyzYaw=waypoints_xyzYaw)

        self.flight_mode.set_param('EKF2_AID_MASK', 1, 5)
        self.flight_mode.set_param('EKF2_HGT_MODE', 0, 5)
            
        rospy.loginfo('Autonomous_flight: Follow aruco pose utilising the bottom camera test complete')
        self.set_state('loiter')
    
    def hold_aruco_pose_test(self):

        #Enable aruco detection, cam use and start index board 
        #self.pub_enable_aruco_detection.publish(True)
        self.pub_aruco_board.publish(1)

        #Set UAV maximum linear and angular velocities in m/s and deg/s respectively
        
        #self.flight_mode.set_param('MPC_XY_VEL_MAX', 0.1, 5)
        #self.flight_mode.set_param('MPC_Z_VEL_MAX_DN', 0.1, 5)
        #self.flight_mode.set_param('MPC_Z_VEL_MAX_UP', 0.1, 5)

        self.flight_mode.set_param('MC_ROLLRATE_MAX', 1.0, 5)
        self.flight_mode.set_param('MC_PITCHRATE_MAX', 1.0, 5)
        self.flight_mode.set_param('MC_YAWRATE_MAX', 1.0, 5)

        alt_ = 2.5
        self.drone_takeoff(alt = alt_)

        self.set_state('hold_aruco_pose_test')
        rospy.loginfo('Autonomous_flight: Hold position by using the aruco marker test startet')

        self.flight_mode.set_param('EKF2_AID_MASK', 24, 5)
        self.flight_mode.set_param('EKF2_HGT_MODE', 3, 5)
        self.flight_mode.set_param('EKF2_EV_DELAY', 50., 5)
        
        while not self.aruco_board_found:
            pass

        start_time = rospy.get_rostime()
        timeout = rospy.Duration(1)

        new_pose = PoseStamped()
        new_pose.pose.position.x = 3.65 #-3.65
        new_pose.pose.position.y = 1.10 #-1.10
        new_pose.pose.position.z = 2.36 #1.5
        new_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, np.deg2rad(90),'rxyz'))

        while (rospy.get_rostime() - start_time) < timeout:
            self.pub_msg(new_pose, self.pub_setpoint)

        self.flight_mode.set_param('MPC_XY_VEL_MAX', 1.0, 5)
        self.flight_mode.set_param('MPC_Z_VEL_MAX_DN', 1.0, 5)
        self.flight_mode.set_param('MPC_Z_VEL_MAX_UP', 1.0, 5)

        self.flight_mode.set_param('MC_ROLLRATE_MAX', 90.0, 5)
        self.flight_mode.set_param('MC_PITCHRATE_MAX', 90.0, 5)
        self.flight_mode.set_param('MC_YAWRATE_MAX', 135.0, 5)

        #See elapsed time
        start = timer()
        time_sec = 0.0

        data = open('../../../../data/hold_aruco_pose/data.txt','a')

        while time_sec <180: #Run test for n seconds 

            new_pose = PoseStamped()
            new_pose.pose.position.x = 3.65 #self.pid_x.update_PID(self.uav_local_pose.pose.position.x)#+0.10
            new_pose.pose.position.y = 1.10 #self.pid_y.update_PID(self.uav_local_pose.pose.position.y)#+0.5
            new_pose.pose.position.z =  2.36 #1.5 #alt_ #self.pid_z.update_PID(self.uav_local_pose.pose.position.z-1.5)
            new_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, np.deg2rad(90),'rxyz'))

            self.pub_msg(new_pose, self.pub_setpoint)
            
            time_sec = timer()-start
            
            x = self.uav_local_pose.pose.position.x
            y = self.uav_local_pose.pose.position.y
            z = self.uav_local_pose.pose.position.z
        
            data.write(str(x) + " " + str(new_pose.pose.position.x) + " " + str(y) + " " + str(new_pose.pose.position.y) + " " + str(z) + " " + str(new_pose.pose.position.z) + " " + str(time_sec))
            data.write('\n')
        
        data.close()
        
        self.flight_mode.set_param('EKF2_AID_MASK', 1, 5)
        self.flight_mode.set_param('EKF2_HGT_MODE', 0, 5)
            
        rospy.loginfo('Autonomous_flight: Hold position by using the aruco marker test complete')
        self.set_state('loiter')
    
    """
    #Helper functions
    def generate_route(self, moves, alt):
    
        waypoints = []
        x = 0
        y = 0
        
        for move in moves:
            if move == 'f':
                x += 1
                yaw = 0
            elif move == 'd':
                x -= 1
                yaw = -180
            elif move == 'l':
                y += 1
                yaw = 90
            elif move == 'r':
                y -= 1
                yaw = -90

            if not move == '':
                waypoint = [x, y, alt, yaw]
                waypoints.append(waypoint)
        
        return waypoints
    """

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
        marker_board_found = self.locate_marker_board(self.uav_local_pose, -6, 0, 1, 0, 1, 1, -45, 90, 45)

        if not marker_board_found:
            rospy.loginfo('Autonomous_flight: Critical error! Marker board NOT found - GPS2Vision!')
            return

        #Navigate the drone to the marker board and switch only if the marker board pose estimation is good
        self.navigate_to_marker_board()

        #Now make transformation of the aruco pose to that of the GPS to insure a smooth GPS2Vision transition
        self.map_GPS_pose_to_vision(self.aruco_pose, self.uav_local_pose)

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
        
        #Giving start position of entrance in front of the GPS2Vision marker 
        pose = PoseStamped()
        pose.pose.position.x = 3.65
        pose.pose.position.y = 1.10
        pose.pose.position.z = 2.36
        pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, np.deg2rad(90),'rxyz'))
        
        pose = self.calculate_GPS2Vision_offset(self.GPS2Vision_offset, pose)

        #Move the drone to the entrance in front of the GPS2Vision marker board 
        while(not self.waypoint_check(setpoint_poseStamped = pose)):
            
            #Known GPS2Vision marker pos with 0.5 offset to center. This way the orientation 
            #of the drone always faces that of the GPS2Vision marker board
            delta_x = self.aruco_pose.pose.position.x - 3.2 - 0.5
            delta_y = self.aruco_pose.pose.position.y - 3.0
            
            theta = np.arctan2(delta_y, delta_x)
            if theta < 0:
                theta = theta + np.pi

            theta = (self.GPS2Vision_offset[3][0] - (self.GPS2Vision_offset[3][1] - theta))
            pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, theta,'rxyz'))
            self.pub_msg(pose, self.pub_setpoint)

        rospy.loginfo('Autonomous_flight: GPS2Vision complete!')

    def vision2GPS_navigation(self):
        
        #Using data from bottom camera
        self.pub_aruco_board.publish(2)


        pass

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
            new_x = self.uav_local_pose.pose.position.x + np.cos(angle[2])*2
            new_y = self.uav_local_pose.pose.position.y + np.sin(angle[2])*2
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
        
        self.fly_route(waypoints_poseStamped, waypoints_xyzYaw, self.GPS2Vision_offset)
        
        rospy.loginfo('Autonomous_flight: Vision navigation completed!')

    def vision_landing_navigation(self, landing_station):
        
        #This substate navigates the drone using vision
        rospy.loginfo('Autonomous_flight: Vision landing started!')
        
        #Using data from front camera to landing marker 
        self.pub_aruco_board.publish(landing_station)
    
        if landing_station == 1:
            waypoints_xyzYaw = [[0.4, 7.1, 1.5, 90], [0.4, 7.1, 0.2, 90]]
        elif landing_station == 2:
            waypoints_xyzYaw = [[3.75, 7.1, 1.5, 90], [3.75, 7.1, 0.2, 90]]
        elif landing_station == 3:
            waypoints_xyzYaw = [[7.1, 7.1, 1.5, 90], [7.1, 7.1, 0.2, 90]]

        self.fly_route(waypoints_poseStamped, waypoints_xyzYaw, self.GPS2Vision_offset)

        #Disarm drone when close tyo ground
        self.flight_mode.set_arm(False,5)

        rospy.loginfo('Autonomous_flight: Vision landing completed!')
    
    def vision_takeoff_navigation(self, landing_station):

        #This substate navigates the drone using vision
        rospy.loginfo('Autonomous_flight: Vision takeoff started!')

        #Using data from front camera to landing marker
        self.pub_aruco_board.publish(landing_station)
        
        #Disarm drone when close tyo ground
        self.flight_mode.set_arm(True,5)
        
        if landing_station == 1:
            waypoints_xyzYaw = [[0.4, 7.1, 1.5, 90]]
        elif landing_station == 2:
            waypoints_xyzYaw = [[3.75, 7.1, 1.5, 90]]
        elif landing_station == 3:
            waypoints_xyzYaw = [[7.1, 7.1, 1.5, 90]]
        self.fly_route(waypoints_poseStamped, waypoints_xyzYaw, self.GPS2Vision_offset)

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
    
    def fly_route(self, waypoints_poseStamped = None, waypoints_xyzYaw = None, GPS2Vision_offset = None):

        #List of waypoints giving as PoseStamped
        if not waypoints_poseStamped == None:

            for waypoint in waypoints_poseStamped:
                if not GPS2Vision_offset == None:
                    waypoint = self.calculate_GPS2Vision_offset(GPS2Vision_offset, waypoint)
                while(not self.waypoint_check(setpoint_poseStamped=waypoint, threshold = self.waypoint_check_pose_error)):
                    self.pub_msg(waypoint, self.pub_setpoint)
        
        #List of waypoints giving as [x,y,z,yaw]
        else:
            for waypoint in waypoints_xyzYaw:
                pose = PoseStamped()
                pose.pose.position.x = waypoint[0]
                pose.pose.position.y = waypoint[1]
                pose.pose.position.z = waypoint[2]
                pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, waypoint[3],'rxyz'))
                
                if not GPS2Vision_offset == None:
                    pose = self.calculate_GPS2Vision_offset(GPS2Vision_offset, pose)

                while(not self.waypoint_check(setpoint_poseStamped=pose, threshold = self.waypoint_check_pose_error)):
                    self.pub_msg(pose, self.pub_setpoint)

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
        print("test " + str(euler_from_matrix(self.r,'rxyz')))
        t = np.array([aruco_pose.pose.position.x,
                      aruco_pose.pose.position.y,
                      aruco_pose.pose.position.z, 1])
        t = np.dot(self.r,t)

        #Initialize offset between original drone pose and aruco board 
        self.GPS2Vision_offset = [[uav_pose.pose.position.x, t[0]],
                                  [uav_pose.pose.position.y, t[1]],
                                  [uav_pose.pose.position.z, t[2]],
                                  [angle_uav[2], angle_aruco[2]]]

        #Pubish aruco board and uav offsets
        self.pub_uav_offset.publish(uav_pose)
        self.pub_aruco_offset.publish(aruco_pose)

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
            elif self.uav_state == 'estimate_aruco_pose_front_test':
                self.estimate_aruco_pose_front_test()
            elif self.uav_state == 'hold_aruco_pose_test': 
                self.hold_aruco_pose_test()
            elif self.uav_state == 'follow_aruco_pose_bottom_test': 
                self.follow_aruco_pose_bottom_test()
            elif self.uav_state == 'GPS2Vision_test':
                self.GPS2Vision_test()
            self.rate.sleep()

    """
    def write_pose_error(self, time, setpoint, file_path):
        
        #Write data to file for analyzing 
        data = open(file_path,'a')
        
        x = self.uav_local_pose.pose.position.x
        y = self.uav_local_pose.pose.position.y
        z = self.uav_local_pose.pose.position.z
        
        data.write(str(x) + " " + str(setpoint[0]) + " " + str(y) + " " + str(setpoint[1]) + " " + str(z) + " " + str(setpoint[2]) + " " + str(time))
        data.write('\n')
        data.close()
    
    def init_data_files(self):
        
        #Init data textfile for aruco pose estimation for front camera 
        data = Path('../../../../data/estimate_aruco_pose_front/data.txt')
        if not data.is_file:
            data = open('../../../../data/estimate_aruco_pose_front/data.txt','r+')
            data.truncate(0)
            data.close
        else:
            data = open('../../../../data/estimate_aruco_pose_front/data.txt','w+')
            data.close()

        #Init data textfile for following aruco makers by utilising the bottom camera 
        data = Path('../../../../data/follow_aruco_pose_bottom/data.txt')
        if not data.is_file:
            data = open('../../../../data/follow_aruco_pose_bottom/data.txt','r+')
            data.truncate(0)
            data.close
        else:
            data = open('../../../../data/follow_aruco_pose_bottom/data.txt','w+')
            data.close()

        #Init data textfile for tracking the aruco for position hold 
        data = Path('../../../../data/hold_aruco_pose/data.txt')
        if not data.is_file:
            data = open('../../../../data/hold_aruco_pose/data.txt','r+')
            data.truncate(0)
            data.close
        else:
            data = open('../../../../data/hold_aruco_pose/data.txt','w+')
            data.close()

    """
if __name__ == "__main__":
    af = autonomous_flight()
    af.run() 
