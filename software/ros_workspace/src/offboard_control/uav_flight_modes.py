#!/usr/bin/python 

##!/usr/bin/env python2
from __future__ import division

import unittest
import rospy
import math

from mavros_msgs.srv import CommandBool, CommandTOL, ParamGet, SetMode, WaypointClear, WaypointPush, ParamSet, CommandLong
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, WaypointList, ParamValue
from geometry_msgs.msg import PoseStamped, Quaternion
from pymavlink import mavutil
from sensor_msgs.msg import NavSatFix, Imu
from six.moves import xrange


# Inspiration from https://github.com/PX4/Firmware/blob/master/integrationtests/python_src/px4_it/mavros/mavros_test_common.py
class flight_modes:

    def __init__(self):
        
        #Initialize parameters
        self.altitude = Altitude()
        self.global_position = NavSatFix()
        self.imu_data = Imu()
        self.home_position = HomePosition()
        self.local_position = PoseStamped()
        self.state = State()
        self.param_value = ParamValue()
        #self.force_disarm_msg = CommandLong()
        
        #Initialize list of subscribed topics ready to be false
        self.sub_topics_ready = {
            key: False
            for key in [
                'alt', 'global_pos', 'home_pos', 'local_pos','state', 'imu'
                ]
        }
        
        #ROS services
        service_timeout = 30
        rospy.loginfo("uav_flight_modes: Waiting for ROS services")
        try:
            rospy.wait_for_service('/mavros/param/set',service_timeout)
            rospy.wait_for_service('/mavros/param/get', service_timeout)
            rospy.wait_for_service('/mavros/cmd/arming', service_timeout)
            rospy.wait_for_service('/mavros/set_mode', service_timeout)
            rospy.loginfo("uav_flight_modes: ROS services are up")
        except rospy.ROSException:
            rospy.logerr("uav_flight_modes: ROS services are NOT up!")

        #Initialize services
        self.set_arming_uav = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_uav = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.set_param_uav = rospy.ServiceProxy('/mavros/param/set', ParamSet)
        self.get_param_uav = rospy.ServiceProxy('/mavros/param/get', ParamGet)
        self.set_force_disarm_uav = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
        
        #Initialize subscribers
        self.alt_sub = rospy.Subscriber('/mavros/altitude', Altitude, self.altitude_callback)
        self.global_pos_sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.global_position_callback)
        self.imu_data_sub = rospy.Subscriber('/mavros/imu/data', Imu, self.imu_data_callback)
        self.home_pos_sub = rospy.Subscriber('/mavros/home_position/home', HomePosition, self.home_position_callback)
        self.local_pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_position_callback)
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_callback)

    def altitude_callback(self, data):
        self.altitude = data
        
        # amsl has been observed to be nan while other fields are valid
        if not self.sub_topics_ready['alt'] and not math.isnan(data.amsl):
            self.sub_topics_ready['alt'] = True
            
    def global_position_callback(self, data):
        
        self.global_position = data
        
        if not self.sub_topics_ready['global_pos']:
            self.sub_topics_ready['global_pos'] = True
 
    def imu_data_callback(self, data):
        self.imu_data = data
        
        if not self.sub_topics_ready['imu']:
            self.sub_topics_ready['imu'] = True
            
    def home_position_callback(self, data):
        self.home_position = data
        
        if not self.sub_topics_ready['home_pos']:
            self.sub_topics_ready['home_pos'] = True
            
    def local_position_callback(self, data):
        self.local_position = data
        
        if not self.sub_topics_ready['local_pos']:
            self.sub_topics_ready['local_pos'] = True
            
    def state_callback(self, data):
        if self.state.armed != data.armed:
            rospy.loginfo("uav_flight_modes: Armed state changed from {0} to {1}".format(self.state.armed, data.armed))
            
        if self.state.connected != data.connected:
            rospy.loginfo("uav_flight_modes: Connected changed from {0} to {1}".format(self.state.connected, data.connected))
            
        if self.state.mode != data.mode:
            rospy.loginfo("uav_flight_modes: Mode changed from {0} to {1}".format(self.state.mode, data.mode))
            
        if self.state.system_status != data.system_status:
            rospy.loginfo("uav_flight_modes: System_status changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_STATE'][
                    self.state.system_status].name, mavutil.mavlink.enums[
                        'MAV_STATE'][data.system_status].name))
                    
        self.state = data
        
        # Mavros publishes a disconnected state message on init
        if not self.sub_topics_ready['state'] and data.connected:
            self.sub_topics_ready['state'] = True

    def force_disarm(self):
        self.set_force_disarm_uav(broadcast=False, command=400, confirmation=0, param1=0.0, param2=21196.0, param3=0.0,
                param4=0.0, param5=0.0, param6=0.0, param7=0.0)

    def set_arm(self, arm, timeout):
        
        #arm: True to arm or False to disarm, timeout(int): seconds
        rospy.loginfo("uav_flight_modes: Setting uav arm: {0}".format(arm))
        old_arm = self.state.armed
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        arm_set = False
        
        for i in xrange(timeout * loop_freq):
            if self.state.armed == arm:
                arm_set = True
                rospy.loginfo("uav_flight_modes: Set arm success | seconds: {0} of {1}".format(i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.set_arming_uav(arm)
                    if not res.success:
                        rospy.logerr("uav_flight_modes: Failed to send arm command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)
                    
            try:
                rate.sleep()
            except rospy.ROSException as e:
                rospy.logerr(e)
     
    def set_mode(self, mode, timeout):
        
        # mode: PX4 mode string, timeout(int): seconds
        rospy.loginfo("uav_flight_modes: Setting FCU mode: {0}".format(mode))
        old_mode = self.state.mode
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        mode_set = False
        
        for i in xrange(timeout * loop_freq):
            if self.state.mode == mode:
                mode_set = True
                rospy.loginfo("uav_flight_modes: Set mode success | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.set_mode_uav(0, mode)  # 0 is custom mode
                    if not res.mode_sent:
                        rospy.logerr("uav_flight_modes: Failed to send mode command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)
                    
            try:
                rate.sleep()
            except rospy.ROSException as e:
                rospy.logerr(e)
        """     
        self.assertTrue(mode_set, (
            "failed to set mode | new mode: {0}, old mode: {1} | timeout(seconds): {2}".
            format(mode, old_mode, timeout)))
        """
    
    def set_param(self, param_id, value, timeout):
        
        rospy.loginfo("uav_flight_modes: Setting uav parameters: {0}".format(value))
        
        if isinstance(value, float):
            val = ParamValue(integer=0, real=value)
        else:
            val = ParamValue(integer=value, real=0.0)

        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        param_set = False
        
        for i in xrange(timeout * loop_freq):
            
            new_value = self.get_param_uav(param_id)
            
            if not new_value == None:
                
                if not new_value.value.integer == 0:
                    new_value = new_value.value.integer
                else:
                    new_value = new_value.value.real

                if value == new_value:
                    param_set = True
                    rospy.loginfo("uav_flight_modes: Set parameters success | seconds: {0} of {1}".format(
                        i / loop_freq, timeout))
                    break
                else:
                    res = self.set_param_uav(param_id, val) 
            try:
                rate.sleep()
            except rospy.ROSException as e:
                rospy.logerr(e)
                
    def get_param(self, param_id):

        try:
            ret = self.get_param_uav(param_id=param_id)
            return ret
        except rospy.ServiceException as ex:
            raise IOError(str(ex))

        """
        try:
            ret = self.get_param_uav(param_id=param_id)
        except rospy.ServiceException as ex:
            raise IOError(str(ex))
    
        if not ret.success:
            raise IOError("uav_flight_modes: Request failed.")
        """   
    def wait_for_topics(self, timeout):
        
        """wait for simulation to be ready, make sure we're getting topic info
        from all topics by checking dictionary of flag values set in callbacks,
        timeout(int): seconds"""
        rospy.loginfo("uav_flight_modes: waiting for subscribed topics to be ready")
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        simulation_ready = False
        
        for i in xrange(timeout * loop_freq):
            if all(value for value in self.sub_topics_ready.values()):
                simulation_ready = True
                rospy.loginfo("uav_flight_modes: Simulation topics ready | seconds: {0} of {1}".
                        format(i / loop_freq, timeout))
                break
            try:
                rate.sleep()
            except rospy.ROSException as e:
                rospy.logerr(e)
        """        
        self.assertTrue(simulation_ready, (
            "failed to hear from all subscribed simulation topics | topic ready flags: {0} | timeout(seconds): {1}".
            format(self.sub_topics_ready, timeout)))
        """
