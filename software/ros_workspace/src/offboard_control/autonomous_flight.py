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
from std_msgs.msg import (String, Int8, Float64, Bool)
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
        self.pid_x = PID(0.1, 0.0, 1.7, 50, -50) #(0.8, 0.2, 1.3, 50, -50)
        self.pid_y = PID(0.1, 0.0, 1.7, 50, -50) #(0.8, 0.2, 1.3, 50, -50)
        self.pid_z = PID(1.0, 0.05, 1.0, 50, -50) # (0.5, 0.5, 0.5, 3, -3)
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

        self.waypoint_check_pose_error = 0.25

        self.delta_x = 0.0
        self.delta_y = 0.0
        self.delta_z = 0.0
        
        self.mission = self.read_mission(self.args[1])
        #self.mission = self.read_mission('../../../../missions/mission3.txt')
        
        self.next_waypoint = PoseStamped()
        self.ground_truth = Odometry()

        #Publishers
        self.pub_local_pose = rospy.Publisher('/onboard/setpoint/autonomous_flight', PoseStamped, queue_size=1)
        self.pub_state = rospy.Publisher('/onboard/state', String, queue_size=10)
        #self.pub_enable_aruco_detection = rospy.Publisher('/onboard/enable_aruco_detection', Bool, queue_size=1)
        #self.pub_aruco_ids = rospy.Publisher('/onboard/aruco_ids', mavlink_lora_aruco, queue_size=1)
        #self.pub_change_aruco_board = rospy.Publisher('/onboard/change_aruco_board', Bool, queue_size=1)
        self.pub_aruco_board = rospy.Publisher('/onboard/aruco_board', Int8, queue_size=1)

        #Subscribers
        rospy.Subscriber('/onboard/state', String, self.on_uav_state)
        rospy.Subscriber('/onboard/aruco_marker_pose', PoseStamped, self.on_aruco_change)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.on_position_change)
        #rospy.Subscriber('/onboard/next_board_found', Bool, self.next_board_found_callback)
        rospy.Subscriber('/onboard/aruco_board_found', Bool, self.aruco_board_found_callback)
        rospy.Subscriber('/odom', Odometry, self.ground_truth_callback)
        self.new_uav_local_pose = PoseStamped()
        
        self.init_data_files()

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
        pass

    def ground_truth_callback(self, data):
        self.ground_truth = data

    def aruco_board_found_callback(self, msg):
        self.aruco_board_found = msg

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

    def waypoint_check(self, setpoint, threshold=0.25):
        
        self.delta_x = self.uav_local_pose.pose.position.x - setpoint[0]
        self.delta_y = self.uav_local_pose.pose.position.y - setpoint[1]
        self.delta_z = self.uav_local_pose.pose.position.z - setpoint[2]
        i = [self.uav_local_pose.pose.orientation.x, self.uav_local_pose.pose.orientation.y, self.uav_local_pose.pose.orientation.z, self.uav_local_pose.pose.orientation.w]
        ori =  euler_from_quaternion(i)
        self.delta_ori = ori[2]-np.deg2rad(setpoint[3])
        
        error = np.sqrt(np.power(self.delta_x,2) + np.power(self.delta_y,2) + np.power(self.delta_z,2) + np.power(self.delta_ori,2))
        
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
        
        #self.flight_mode.set_mode('AUTO.TAKEOFF',10)
        self.flight_mode.set_mode('OFFBOARD',5)
    
        rospy.loginfo('Autonomous_flight: UAV takeoff')
        
        #Wait until takeoff has occurred
        waypoint = [self.uav_local_pose.pose.position.x, self.uav_local_pose.pose.position.y, alt, 0]
        print(pre_pose)
        while(not self.waypoint_check(setpoint = waypoint, threshold=0.20)): #Set t0 0.05 in loiter to avoid ascilations
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
                self.pub_msg(waypoint, self.pub_local_pose)

            #Get ArUco position, mean and STD
            start_time = rospy.get_rostime()
            timeout = rospy.Duration(5) #Test each position for x seconds 
            
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
                error_roll = abs(angle[0]-g_roll)
                error_pitch = abs(angle[1]-g_pitch)
                error_yaw = abs(angle[2]-g_yaw)

                aruco_x.append(error_x)
                aruco_y.append(error_y)
                aruco_z.append(error_z)
                aruco_roll.append(error_roll)
                aruco_pitch.append(error_pitch)
                aruco_yaw.append(error_yaw)

            mean_x = sum(aruco_x)/len(aruco_x)
            mean_y = sum(aruco_y)/len(aruco_y)
            mean_z = sum(aruco_z)/len(aruco_z)

            mean_roll = sum(aruco_roll)/len(aruco_roll)
            mean_pitch = sum(aruco_pitch)/len(aruco_pitch)
            mean_yaw = sum(aruco_yaw)/len(aruco_yaw)

            x = waypoint.pose.position.x
            y = waypoint.pose.position.x

            self.log_data.write_GPS2Vision_marker_detection_data(x, y, mean_x, mean_y, mean_z, mean_roll, mean_pitch, mean_yaw)
        
        #Return home
        self.set_state('home')
        self.drone_return_home()

        rospy.loginfo('Autonomous_flight: Estimate the aruco pose utilising the front camera test complete')
        self.set_state('loiter')

    def follow_aruco_pose_bottom_test(self):

        #Inialize parameters 
        while not self.mission[0][0] == '-':
            self.update_mission()
        self.mission.pop(0)
        
        self.drone_takeoff(alt = 2.5)

        self.set_state('follow_aruco_pose_bottom_test')
        rospy.loginfo('Autonomous_flight: Follow aruco pose utilising the bottom camera test startet')

        #Now turn to vision navigation
        while not self.mission[0][0] == '-':
            self.update_mission()

        """
        while not self.aruco_board_found:
            pass
        """
        #See elapsed time
        start = timer()
        time_sec = 0.0

        data = open('../../../../data/follow_aruco_pose_bottom/data.txt','a')
        
        while len(self.mission):
            print(self.mission[0])
            
            start_time = rospy.get_rostime()
            timeout = rospy.Duration(1)
           
            #Inialize now set parameters to enable higher speeds after GPS to vision has settled
            while not self.mission[0][0] == '-':
                self.update_mission()

            waypoint = [float(self.mission[0][3]), float(self.mission[0][4]), float(self.mission[0][5]), float(self.mission[0][8])] 
            self.update_mission()
            
            while True:
                
                    self.pub_msg(self.next_waypoint, self.pub_local_pose)
                    
                    x = self.uav_local_pose.pose.position.x
                    y = self.uav_local_pose.pose.position.y
                    z = self.uav_local_pose.pose.position.z

                    if self.waypoint_check(setpoint = [waypoint[0], waypoint[1], waypoint[2], waypoint[3]], threshold = self.waypoint_check_pose_error):
                        break

                    time_sec = timer()-start
                    data.write(str(x) + " " + str(waypoint[0]) + " " + str(y) + " " + str(waypoint[1]) + " " + str(z) + " " + str(waypoint[2]) + " " + str(time_sec))
                    data.write('\n')
            

        data.close()
        self.flight_mode.set_param('EKF2_AID_MASK', 1, 5)
        self.flight_mode.set_param('EKF2_HGT_MODE', 0, 5)
            
        rospy.loginfo('Autonomous_flight: Follow aruco pose utilising the bottom camera test complete')
        self.set_state('loiter')
    
    def hold_aruco_pose_test(self):

        #Enable aruco detection, cam use and start index board 
        #self.pub_enable_aruco_detection.publish(True)
        self.pub_aruco_board.publish(1)

        #Set UAV maximum linear and angular velocities in m/s and deg/s respectively
        
        self.flight_mode.set_param('MPC_XY_VEL_MAX', 0.1, 5)
        self.flight_mode.set_param('MPC_Z_VEL_MAX_DN', 0.1, 5)
        self.flight_mode.set_param('MPC_Z_VEL_MAX_UP', 0.1, 5)

        
        self.flight_mode.set_param('MC_ROLLRATE_MAX', 0.1, 5)
        self.flight_mode.set_param('MC_PITCHRATE_MAX', 0.1, 5)
        self.flight_mode.set_param('MC_YAWRATE_MAX', 0.1, 5)

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
            self.pub_msg(new_pose, self.pub_local_pose)

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

            self.pub_msg(new_pose, self.pub_local_pose)
            
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

    def update_mission(self):

        param = ' '
        param_value = 0
        timeout = 0
        transform = 0

        if not self.mission[0][0] == '-':
            param = self.mission[0][0]

            try:
                param_value = int(self.mission[0][1])
            except ValueError:
                param_value = float(self.mission[0][1])

            timeout = int(self.mission[0][2])

            #Update parameters
            self.flight_mode.set_param(param, param_value, timeout)

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
        #print(param + ' ' + str(param_value) + ' ' + str(timeout) + ' ' + str(self.next_waypoint.pose.position.x) + ' ' + str(transform))

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
            self.rate.sleep()

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
if __name__ == "__main__":
    af = autonomous_flight()
    af.run() 
