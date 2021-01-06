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

class autonomous_flight():

    def __init__(self):        
        
        rospy.init_node('autonomous_flight')
        self.rate = rospy.Rate(30)

        #Init flight modes 
        self.flight_mode = flight_modes()
        
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

        self.delta_x = 0.0
        self.delta_y = 0.0
        self.delta_z = 0.0


        #Publishers
        self.pub_local_pose = rospy.Publisher('/onboard/setpoint/autonomous_flight', PoseStamped, queue_size=1)
        self.pub_state = rospy.Publisher('/onboard/state', String, queue_size=10)
        self.pub_enable_aruco_detection = rospy.Publisher('/onboard/enable_aruco_detection', Bool, queue_size=1)
        self.pub_aruco_ids = rospy.Publisher('/onboard/aruco_ids', mavlink_lora_aruco, queue_size=1)
        self.pub_change_aruco_board = rospy.Publisher('/onboard/change_aruco_board', Bool, queue_size=1)
        self.pub_use_bottom_cam= rospy.Publisher('/onboard/use_bottom_cam', Bool, queue_size=1)

        #Subscribers
        rospy.Subscriber('/onboard/state', String, self.on_uav_state)
        rospy.Subscriber('/onboard/aruco_marker_pose', PoseStamped, self.on_aruco_change)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.on_position_change)
        rospy.Subscriber('/onboard/next_board_found', Bool, self.next_board_found_callback)
        rospy.Subscriber('/onboard/aruco_board_found', Bool, self.aruco_board_found_callback)
        self.new_uav_local_pose = PoseStamped()
        
        self.init_data_files()

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

    def next_board_found_callback(self, msg):
        self.next_board_found = msg

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
        error = np.sqrt(np.power(self.delta_x,2) + np.power(self.delta_y,2) + np.power(self.delta_z,2))

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

        #Enable aruco detection
        self.pub_enable_aruco_detection.publish(True)
        aruco_ids = mavlink_lora_aruco()
        aruco_ids.elements.append(101)
        aruco_ids.elements.append(100)
        self.pub_aruco_ids.publish(aruco_ids)

        #UAV valocity
        self.flight_mode.set_param('MPC_XY_VEL_MAX', 1.5, 5)
        self.flight_mode.set_param('MPC_Z_VEL_MAX_DN', 1.0, 5)
        self.flight_mode.set_param('MPC_Z_VEL_MAX_UP', 1.0, 5)

        alt_ = 1.7
        self.drone_takeoff(alt = alt_)

        self.set_state('aruco_pose_estimation_test')
        rospy.loginfo('Autonomous_flight: UAV aruco pose estimation test startet')

        x = self.uav_home_pose.pose.position.x
        y = self.uav_home_pose.pose.position.y

        """
        #Init data textfile
        data = Path('../../../../data/aruco_pose_estimation_test/data.txt')
        if not data.is_file:
            data = open('../../../../data/aruco_pose_estimation_test/data.txt','r+')
            data.truncate(0)
            data.close
        else:
            data = open('../../../../data/aruco_pose_estimation_test/data.txt','w+')
            data.close()

        """

        #Initialize waypoints 
        waypoints = []
        dis_to_aruco_x = 2
        dx = 2
        dy = 4
        alt = 1.7

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
            while(not self.waypoint_check(setpoint = [waypoint.pose.position.x, waypoint.pose.position.y, waypoint.pose.position.z])):
                if not self.uav_state == 'aruco_pose_estimation_test':
                    rospy.loginfo('Autonomous_flight: Aruco pose estimation test disrupted!')
                    return
                self.pub_msg(waypoint, self.pub_local_pose)

            #Get ArUco position, mean and STD
            start_time = rospy.get_rostime()
            timeout = rospy.Duration(5) #Test each position for x seconds 
            aruco_pos_x = []
            aruco_pos_y = []
            aruco_pos_z = []
            
            while (rospy.get_rostime() - start_time) < timeout:
                aruco_pos_x.append(self.aruco_pose.pose.position.x)
                aruco_pos_y.append(self.aruco_pose.pose.position.y)
                aruco_pos_z.append(self.aruco_pose.pose.position.z)

            #Find mean, STD and error from real ArUco pos
            if index and index % 7 == 0:
                dis_to_aruco_x = dis_to_aruco_x + 1
            mean_x = sum(aruco_pos_x)/len(aruco_pos_x)
            diff = 0.0
            for pos in aruco_pos_x:
                diff += (pos-mean_x)*(pos-mean_x)
            std_x = np.sqrt(diff/len(aruco_pos_x))
            error_x = abs(dis_to_aruco_x+mean_x)

            mean_y = sum(aruco_pos_y)/len(aruco_pos_y)
            diff = 0.0
            for pos in aruco_pos_y:
                diff += (pos-mean_y)*(pos-mean_y)
            std_y = np.sqrt(diff/len(aruco_pos_y))
            error_y = abs(mean_y - self.uav_local_pose.pose.position.y)
                
            mean_z = sum(aruco_pos_z)/len(aruco_pos_z)
            diff = 0.0
            for pos in aruco_pos_z:
                diff += (pos-mean_z)*(pos-mean_z)

            std_z = np.sqrt(diff/len(aruco_pos_z))
            error_z = mean_z

            #Write data to file for analyzing 
            data = open('../../../../data/aruco_pose_estimation_test/data.txt','a')
            x = waypoint.pose.position.x
            y = waypoint.pose.position.y
            z = waypoint.pose.position.z
            data.write(str(x) + " " + str(std_x) + " " + str(error_x) + " " + str(y) + " " + str(std_y) + " " + str(error_y) + " "+ str(z) + " " + str(std_z) + " " + str(error_z))
            data.write('\n')
            data.close()

        #Return home
        self.set_state('home')
        self.drone_return_home()

        rospy.loginfo('Autonomous_flight: Aruco pose estimation test complete')
        self.set_state('loiter')

    def gps_to_vision_test(self):

        #Enable aruco detection, cam use and start index board 
        self.pub_enable_aruco_detection.publish(True)
        self.pub_use_bottom_cam.publish(True)
        self.pub_change_aruco_board.publish(True)

        #Route to charging station (Here three different routes have been specified)
        to_charging_station = mavlink_lora_aruco()
        
        """
        to_charging_station.id = [1,7,13,19,25,31,37,43]
        to_charging_station.moves = ['','f','f','f','f','f','f','f']
        to_charging_station.yaw = [0, 0, 0, 0, 0, 0, 0, 0,]   
        
        """
        to_charging_station.id = [1,7,13,19,25,49,55,61,67,73,79,85]
        to_charging_station.moves = ['','f','f','f','f','l','l','l','l','f','f','f']
        to_charging_station.yaw = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        
        
        """
        to_charging_station.id = [1,7,13,19,25,91,97,103,109,115,121,127]
        to_charging_station.moves = ['','f','f','f','f','r','r','r','r','f','f','f']
        to_charging_station.yaw = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        """

        #Route back to the world
        back_to_world = mavlink_lora_aruco()
        back_to_world.id = to_charging_station.id[::-1]
        back_to_world.moves = to_charging_station.moves[::-1]
        back_to_world.moves.insert(0,back_to_world.moves.pop()) #Get '' to start index
        for i in range(len(back_to_world.moves)):
            if back_to_world.moves[i] == 'f':
                back_to_world.moves[i] = 'd'
            elif back_to_world.moves[i] == 'd':
                back_to_world.moves[i] = 'f'
            elif back_to_world.moves[i] == 'l':
                back_to_world.moves[i] = 'r'
            elif back_to_world.moves[i] == 'r':
                back_to_world.moves[i] = 'l'
        
        print(back_to_world.moves)
        print(back_to_world.id)
        
         
        self.pub_aruco_ids.publish(to_charging_station)
        waypoints = self.generate_route(to_charging_station.moves, alt=1.5)
        waypoints_to_complete = len(waypoints)

        """
        self.pub_aruco_ids.publish(back_to_world)
        waypoints = self.generate_route(back_to_world.moves, alt=1.5)
        waypoints_to_complete = len(waypoints)
        """

        """
        #Set UAV maximum linear and angular velocities in m/s and deg/s respectively
        self.flight_mode.set_param('MPC_XY_VEL_MAX', 0.2, 5)
        self.flight_mode.set_param('MPC_Z_VEL_MAX_DN', 0.2, 5)
        self.flight_mode.set_param('MPC_Z_VEL_MAX_UP', 0.2, 5)
        """

        self.flight_mode.set_param('MC_ROLLRATE_MAX', 45.0, 5)
        self.flight_mode.set_param('MC_PITCHRATE_MAX', 45.0, 5)
        self.flight_mode.set_param('MC_YAWRATE_MAX', 90.0, 5)

        self.drone_takeoff(alt = 1.5)

        self.set_state('gps_to_vision_test')
        rospy.loginfo('Autonomous_flight: Gps to vision test startet')

        self.flight_mode.set_param('EKF2_AID_MASK', 24, 5)
        self.flight_mode.set_param('EKF2_HGT_MODE', 3, 5)
        self.flight_mode.set_param('EKF2_EV_DELAY', 50., 5)
        
        while not self.aruco_board_found:
            pass

        #See elapsed time
        start = timer()
        time_sec = 0.0

        #wait until waypoint reached
        while True:

            waypoint = waypoints[0]
            new_pose = PoseStamped()
            #new_pose.pose.position.x = waypoint[0]
            #new_pose.pose.position.y = waypoint[1]
            new_pose.pose.position.z = waypoint[2]
            
            new_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, np.deg2rad(0),'rxyz'))       
            
            while True:

                new_pose.pose.position.x = self.pid_x.update_PID(self.uav_local_pose.pose.position.x-waypoint[0])
                new_pose.pose.position.y = self.pid_y.update_PID(self.uav_local_pose.pose.position.y-waypoint[1])
                #start = timer()
                
                #Write pose error
                #self.write_pose_error(start,waypoint)

                #if not len(waypoints):
                #self.waypoint_check(setpoint=[waypoint[0], waypoint[1], waypoint[2]], threshold = 0.25)
                #        break
                #else:
                
                if self.next_board_found:
                    waypoints.pop(0)
                    self.next_board_found = False
                    break

                self.pub_msg(new_pose, self.pub_local_pose)

                time_sec = timer()-start

                self.write_pose_error(time_sec, waypoint)
            
            
            self.pub_change_aruco_board.publish(True)
            
            if not len(waypoints):
                break
        
        self.flight_mode.set_param('EKF2_AID_MASK', 1, 5)
        self.flight_mode.set_param('EKF2_HGT_MODE', 0, 5)
            
        rospy.loginfo('Autonomous_flight: Gps to vision test complete')
        self.set_state('loiter')

    def hold_vision_pose_test(self):

        #Enable aruco detection, cam use and start index board 
        self.pub_enable_aruco_detection.publish(True)
        self.pub_use_bottom_cam.publish(True)
        #self.pub_change_aruco_board.publish(True)

        """
        #Set UAV maximum linear and angular velocities in m/s and deg/s respectively
        self.flight_mode.set_param('MPC_XY_VEL_MAX', 0.2, 5)
        self.flight_mode.set_param('MPC_Z_VEL_MAX_DN', 0.2, 5)
        self.flight_mode.set_param('MPC_Z_VEL_MAX_UP', 0.2, 5)

        self.flight_mode.set_param('MC_ROLLRATE_MAX', 45.0, 5)
        self.flight_mode.set_param('MC_PITCHRATE_MAX', 45.0, 5)
        self.flight_mode.set_param('MC_YAWRATE_MAX', 90.0, 5)

        """
        
        alt_ = 1.5
        self.drone_takeoff(alt = alt_)

        self.set_state('hold_vision_pose_test')
        rospy.loginfo('Autonomous_flight: Hold vision pose test startet')

        self.flight_mode.set_param('EKF2_AID_MASK', 24, 5)
        self.flight_mode.set_param('EKF2_HGT_MODE', 3, 5)
        self.flight_mode.set_param('EKF2_EV_DELAY', 50., 5)
        
        while not self.aruco_board_found:
            pass

        #See elapsed time
        start = timer()
        time_sec = 0.0

        #wait until waypoint reached
        while time_sec <80:

            new_pose = PoseStamped()
            new_pose.pose.position.x = self.pid_x.update_PID(self.uav_local_pose.pose.position.x)#+0.10
            new_pose.pose.position.y = self.pid_y.update_PID(self.uav_local_pose.pose.position.y)#+0.5
            new_pose.pose.position.z = 1.5 #self.pid_z.update_PID(self.uav_local_pose.pose.position.z-1.5)
            new_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, np.deg2rad(-90),'rxyz'))

            self.pub_msg(new_pose, self.pub_local_pose)
            time_sec = timer()-start
            self.write_pose_error(time_sec, [new_pose.pose.position.x, new_pose.pose.position.y, new_pose.pose.position.z])
        
        self.flight_mode.set_param('EKF2_AID_MASK', 1, 5)
        self.flight_mode.set_param('EKF2_HGT_MODE', 0, 5)
            
        rospy.loginfo('Autonomous_flight: Hold vision pose test complete')
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
            elif self.uav_state == 'hold_vision_pose_test': 
                self.hold_vision_pose_test()
            self.rate.sleep()

    def write_pose_error(self, time, setpoint):
        
        #Write data to file for analyzing 
        data = open('../../../../data/gps_vision_pose_error/data.txt','a')
        
        x = self.uav_local_pose.pose.position.x
        y = self.uav_local_pose.pose.position.y
        z = self.uav_local_pose.pose.position.z
        
        data.write(str(x) + " " + str(setpoint[0]) + " " + str(y) + " " + str(setpoint[1]) + " " + str(z) + " " + str(setpoint[2]) + " " + str(time))
        data.write('\n')
        data.close()

    def init_data_files(self):
        
        #Init data textfile for aruco pose etsimation error
        data = Path('../../../../data/aruco_pose_estimation_test/data.txt')
        if not data.is_file:
            data = open('../../../../data/aruco_pose_estimation_test/data.txt','r+')
            data.truncate(0)
            data.close
        else:
            data = open('../../../../data/aruco_pose_estimation_test/data.txt','w+')
            data.close()

        #Init data textfile for gps to vision navigation pose error
        data = Path('../../../../data/gps_vision_pose_error/data.txt')
        if not data.is_file:
            data = open('../../../../data/gps_vision_pose_error/data.txt','r+')
            data.truncate(0)
            data.close
        else:
            data = open('../../../../data/gps_vision_pose_error/data.txt','w+')
            data.close()

if __name__ == "__main__":
    af = autonomous_flight()
    af.run() 
