#!/usr/bin/env python

import numpy as np
import rospy
import mavros as mav
import mavros.utils
import mavros.command as mavCMD
import mavros.setpoint as mavSP
import mavros_msgs.msg
import mavros_msgs.srv

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import (String, Int8, Float64, Bool)
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion

mavros.set_namespace('mavros')

onB_StateSub     = '/onboard/state'
mh_enableSub     = '/onboard/enableMH'
commandSub       = '/gcs/command'
wpCompleteTopic  = '/onboard/check/WPSuccess'
marker_detection_mission_pub = '/onboard/setpoint/marker_position_mission'

class droneCore():
    def __init__(self):
        
        #Init ROS
        rospy.init_node('DroneCoreNode')
        self.rate = rospy.Rate(20)
        
        #Variables
        self.MH_enabled = False
        self.sysState = 'idle' 
        self.MAVROS_State = mavros_msgs.msg.State()
        self.isAirbourne = False
        
        self.uavGPSPos = None
        self.uavLocalPos = mavSP.PoseStamped()
        self.uavLocalSetpoint = mavSP.PoseStamped()
        self.uavHdg = None  

        #Subscribers
        rospy.Subscriber(mavros.get_topic('state'), mavros_msgs.msg.State, self._cb_uavState) 
        rospy.Subscriber(mavros.get_topic('global_position','global'), NavSatFix, self._cb_SatFix)
        rospy.Subscriber(mavros.get_topic('setpoint_position','local'), mavSP.PoseStamped, self._cb_onSetpointChange)
        rospy.Subscriber(mavros.get_topic('local_position', 'pose'), mavSP.PoseStamped, self._cb_onPositionChange)
        rospy.Subscriber(mavros.get_topic('global_position', 'compass_hdg'), Float64, self._cb_headingUpdate)
        rospy.Subscriber(commandSub, Int8, self._cb_onCommand)
        #rospy.Subscriber(isolatedSub, Bool, self._cb_onKillSwitch)

        #Publishers
        self.statePub = rospy.Publisher(onB_StateSub, String, queue_size=1)
        self.enableMHPub = rospy.Publisher(mh_enableSub, Bool, queue_size=1)
        self.wpCompletePub = rospy.Publisher(wpCompleteTopic, Bool, queue_size=1)
        self.spLocalPub = mavSP.get_pub_position_local(queue_size=5)
        self.pub_local_pose = rospy.Publisher('/mavros/setpoint_position/local',PoseStamped,queue_size=10)
        self.marker_detection_mission_pub = rospy.Publisher(marker_detection_mission_pub, PoseStamped, queue_size=1)

        #Services
        self.setMode = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
        self.enableTakeoff = rospy.ServiceProxy('/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
        self.set_home_pos = rospy.ServiceProxy('/mavros/cmd/set_home', mavros_msgs.srv.CommandHome)

        #Perform MAVROS handshake   
        self._mavrosHandshake()

    
    def _mavrosHandshake(self): 
        rospy.loginfo('DroneCore: Waiting for MAVROS Connection.')
        i=0
        time = rospy.Time.now()
        for i in range(0,3):
            print'.',
            if self.MAVROS_State.connected:
                rospy.loginfo("DroneCore: MAVROS Connected!")
                break
            rospy.sleep(1)
        if not self.MAVROS_State.connected:
            errorMsg = "DroneCore: MAVROS not connected!"
            rospy.logfatal(errorMsg)
            rospy.signal_shutdown(errorMsg)
    
    #Generic function to publish a message
    def _pubMsg(self, msg, topic):
        msg.header = mavSP.Header(
            frame_id="att_pose",
            stamp=rospy.Time.now())

        topic.publish(msg)
        self.rate.sleep()

    #Fuction for updating onboard state
    def _setState(self, state):
        self.sysState = state
        if state == 'idle':
            self.enableMHPub.publish(False)
        elif not self.MH_enabled:
            self.enableMHPub.publish(True)
            self.MH_enabled = True
        self.statePub.publish(state)
        rospy.loginfo('DroneCore: state = {}'.format(state))

    def _cb_onPositionChange(self, msg):
        self.uavLocalPos = msg
        pass

    def _cb_onSetpointChange(self, msg):
        self.uavLocalSetpoint = msg
        pass

    def _cb_onCommand(self, msg):
        command = str(chr(msg.data))
        command.lower()
    
        if command == 't': #Takeoff
            self.droneTakeoff()
            #if self.sysState == 'takeoff':
            self._setState('loiter')
        
        if command == 'h': #Returns the drone to home
            pass
        
        if command == 'm': #Execute mission
            self.drone_marker_detection_mission()

        if command == 'k': # Kill drone
            self.enableMHPub.publish(False)
            self._setState('idle')            
            #TODO: Implement PX4 kill switch

    def _cb_SatFix(self, msg):
        self.uavGPSPos = msg
    
    def _cb_headingUpdate(self,msg):
        self.uavHdg = msg
    
    def _cb_uavState(self, msg):
        self.MAVROS_State = msg
        if self.sysState != 'idle' and self.MAVROS_State.mode != 'OFFBOARD':
             rospy.logwarn("DroneCore: System enabled, but drone is in manual control. Disabling Message Handler")
             self._setState('idle')
        pass

    
    def waypointCheck(self, threshold=0.25):
        pos = np.array((self.uavLocalPos.pose.position.x,
                        self.uavLocalPos.pose.position.y,
                        self.uavLocalPos.pose.position.z))

        setpoint = np.array((self.uavLocalSetpoint.pose.position.x,
                        self.uavLocalSetpoint.pose.position.y,
                        self.uavLocalSetpoint.pose.position.z))        

        return np.linalg.norm(setpoint - pos) < threshold

    def droneTakeoff(self, alt=1.5):
        if self.isAirbourne == False or self.sysState == 'idle':
            if not self.MAVROS_State.armed:
                mavCMD.arming(True)
                rospy.loginfo('DroneCore: Arming')

            preArmMsgs = self.uavLocalPos
            preArmMsgs.pose.position.z = alt
            rospy.loginfo('DroneCore: Takeoff altitude = {} m'.format(preArmMsgs.pose.position.z))

            for i in range(0,50):
                self._pubMsg(preArmMsgs, self.spLocalPub)
                #self.rate.sleep()

            self.setMode(0, 'OFFBOARD')
            rospy.loginfo('DroneCore: PX4 mode = OFFBOARD')
            
            #self._setState('takeoff')
            self.isAirbourne = True

            #wait until takeoff has occurred
            while(not self.waypointCheck()):
                self._pubMsg(preArmMsgs, self.spLocalPub)

            rospy.loginfo('DroneCore: UAV is airbourne')
            self.enableMHPub.publish(self.isAirbourne)
            rospy.loginfo('DroneCore: Takeoff complete')
    
    def drone_marker_detection_mission(self):
        
        #To change velocity of the drone set the MPC_XY_VEL_MAX, MPC_Z_VEL_MAX_DN and MPC_Z_VEL_MAX_UP parameters
        alt_ = 1
        self.droneTakeoff(alt = alt_)
        
        waypoints = [[-5, 0, alt_], [5, 0, alt_], [0, 0, alt_]]
        angle = Quaternion(*quaternion_from_euler(0, 0, np.deg2rad(90)))

        self._setState('marker_detection_mission')
        for waypoint in waypoints:
            
            preArmMsgs = self.uavLocalPos
            preArmMsgs.pose.position.x = waypoint[0]
            preArmMsgs.pose.position.y = waypoint[1]
            preArmMsgs.pose.position.z = waypoint[2]
            preArmMsgs.pose.orientation = angle
            
            #wait until waypoint reached
            while(not self.waypointCheck()):
                self.marker_detection_mission_pub.publish(preArmMsgs)
                self._pubMsg(preArmMsgs, self.spLocalPub)
            
        self._setState('idle')
        rospy.loginfo('DroneCore: Mission fence detection complete')

    def run(self):
        while not rospy.is_shutdown():
            self.wpCompletePub.publish(self.waypointCheck())
            self.rate.sleep()
        pass

if __name__ == "__main__":
    dc = droneCore()
    dc.run()
