#!/usr/bin/env python

import math
import utm
import rospy
import sys
import termios
import tty

import mavros.utils
import mavros.setpoint as mavSP
import mavros.command as mavCMD
import mavros_msgs.msg
import mavros_msgs.srv

from std_msgs.msg import (Bool, String, Int8)
from geometry_msgs.msg import (Quaternion)
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import NavSatFix

mavros.set_namespace('mavros')

mavlink_lora_keypress_pub_topic = '/gcs/command'
onboard_state_sub = '/onboard/state'
onboard_substate_sub = '/onboard/substate'
loiter_sub = '/onboard/setpoint/loiter'
marker_detection_mission_sub = '/onboard/setpoint/marker_position_mission'
onboard_message_sub = '/onboard/message'

class msgControl():
    def __init__(self):

        #Init ROS
        rospy.init_node('msgControl', disable_signals = True)
        rospy.Timer(rospy.Duration(1./20.), self.timer_callback)
        
        #Variables
        self.enable = False
        self.sysState = None
        self.sysSubState = None 
        self.MAVROS_State = mavros_msgs.msg.State()
        self.homePos = None
        self.gotMission = False
        self.marker_detection_mission_msg = None
        self.loiterMsg = None
        self.setpointLocal = mavSP.PoseStamped()
        self.setpointATTI = mavros_msgs.msg.AttitudeTarget()
        self.curLocalPos = mavSP.PoseStamped()

        #Publishers
        self.setpointLocalPub = mavSP.get_pub_position_local(queue_size=1)
        self.setpointATTIPub = mavSP.get_pub_attitude_pose(queue_size=1)
        self.keypress_pub = rospy.Publisher(mavlink_lora_keypress_pub_topic, Int8, queue_size=0)

        #Subscribers
        rospy.Subscriber(onboard_state_sub, String, self.onStateChange)
        rospy.Subscriber(onboard_substate_sub, String, self.onSubStateChange)
        rospy.Subscriber(mavros.get_topic('state'), mavros_msgs.msg.State, self.uavState)
        rospy.Subscriber('/onboard/enableMH', Bool, self.handlerEnable)
        rospy.Subscriber(mavros.get_topic('local_position', 'pose'), mavSP.PoseStamped, self.localPosUpdate)
        rospy.Subscriber(mavros.get_topic('home_position', 'home'), mavros_msgs.msg.HomePosition, self.onHomeUpdate)
        rospy.Subscriber(loiter_sub, mavSP.PoseStamped, self.pilot_loiterMsg)
        rospy.Subscriber(marker_detection_mission_sub, mavSP.PoseStamped, self.pilot_marker_detection_mission_sub)
        rospy.Subscriber(onboard_message_sub, String, self.onMessage)
        
        #Services 
        self.setMode = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)

        self._mavrosHandshake()
        rospy.spin()

    def _mavrosHandshake(self):
        rospy.loginfo('MessageHandler: Waiting for MAVROS Connection.')
        i = 0
        time = rospy.Time.now()
        for i in range(0, 3):
            if self.MAVROS_State.connected:
                rospy.loginfo("MessageHandler: MAVROS Connected!")
                break
            rospy.sleep(1)

        if not self.MAVROS_State.connected:
            errorMsg = "MAVROS not connected, will try again in 30 Seconds."
            rospy.logfatal(errorMsg)
            for i in range(1, 30):
                rospy.sleep(1)
                if self.MAVROS_State.connected:
                    rospy.loginfo("MessageHandler: MAVROS Connected!")
                    break
            # rospy.signal_shutdown(errorMsg)

    def handlerEnable(self, msg):
        if msg.data == True:
            rospy.loginfo("MessageHandler: Enabled")
            self.enable = True
        if msg.data == False:
            if self.enable:
                rospy.loginfo("MessageHandler: Disabled")
            self.enable = False

    def _pubMsg(self, msg, topic):
        msg.header.stamp = rospy.Time.now()
        topic.publish(msg)
        self.rate.sleep()
        
    def onStateChange(self, msg):
        if msg.data == 'idle':
            self.enable = False
            print("messageHandler Disabled")
        else:
            self.sysState = msg.data

    def onSubStateChange(self,msg):
        self.sysSubState = msg

    def onMessage(self,msg):
        self.on_message = msg

    def localPosUpdate(self, msg):
        self.curLocalPos = msg

    def onHomeUpdate(self, msg):
        if self.homePos == None:
            # print("home updated")
            pass
        self.homePos = msg

    def uavState(self, msg):
        self.MAVROS_State = msg
        pass

    def onPositionChange(self, msg):
        self.curLocalPos = msg

    def pilot_loiterMsg(self, msg):
        self.loiterMsg = msg

    def pilot_marker_detection_mission_sub(self, msg):
        self.marker_detection_mission_msg = msg

    def get_pilotMsg(self):
        
        outwardMsg = None

        
        #if not self.on_message == None:

        if self.sysState == 'loiter':
            outwardMsg = self.loiterMsg
        
        if self.sysState == 'gps_navigation':
            outwardMsg = self.gps_navigation_msg

        if self.sysState == 'gps_to_vision':
            outwardMsg = self.gps_to_vision_msg

        if self.sysState == 'vision_to_gps':
            outwardMsg = self.vision_to_gps_msg

        if self.sysState == 'vision_navigation':
            outwardMsg = self.vision_navigation_msg

        if self.sysState == 'vision_landing':
            outwardMsg = self.vision_landing_msg
            
        if self.sysState == 'marker_detection_mission':
            outwardMsg = self.marker_detection_mission_msg

        if outwardMsg == None:
            rospy.logfatal_once("MessageHandler received no message: has a pilot crashed?")
            self.setMode(0, "AUTO.LOITER")
            rospy.loginfo('MessageHandler: PX4 mode = AUTO.LOITER')
        
        return outwardMsg

    def publishPilotMsg(self):
        outMsg = None
        try:
            outMsg = self.get_pilotMsg()
            if (outMsg._type == "geometry_msgs/PoseStamped"):
                self._pubMsg(outMsg, self.setpointLocalPub)

            if self.homePos == None:
                rospy.loginfo_once("unable: no home position")
        
        except AttributeError:
            if outMsg == None:
                rospy.logwarn_once("MessageHandler: No ")
                self.setMode(0, 'AUTO.LOITER')
    
    def calcDist(self, utmPosA, utmPosB):
        dist = -1
        deltaEast = utmPosA[0]-utmPosB[0]
        deltaNorth = utmPosA[1]-utmPosB[1]
        if deltaNorth != 0:
            dist = math.sqrt(deltaNorth**2 + deltaEast**2)
        return deltaNorth, deltaEast, dist

    def gpsToLocal(self, gpsPos):
        utmPos = utm.from_latlon(gpsPos.latitude, gpsPos.longitude)

        utmHome = utm.from_latlon(self.homePos.geo.latitude, self.homePos.geo.longitude)

        deltaNorth, deltaEast, _ = self.calcDist(utmPos, utmHome)
        deltaAlt = gpsPos.altitude
        # print ("Local Point: %.4f, %.4f, %.2f" % (deltaNorth, deltaEast, gpsPos.altitude))
        return deltaNorth, deltaEast, deltaAlt

    def timer_callback(self,event):
        
        #Enable messagehandler
        if self.enable:
            self.publishPilotMsg()

if __name__ == "__main__":
    node = msgControl()
