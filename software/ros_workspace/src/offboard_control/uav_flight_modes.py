#!/usr/bin/python 

##!/usr/bin/env python2
from __future__ import division

import unittest
import rospy
import math

from mavros_msgs.srv import CommandBool, CommandTOL, ParamGet, SetMode, WaypointClear, WaypointPush, ParamSet
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
        
        #Initialize list of subscribed topics ready to be false
        self.sub_topics_ready = {
            key: False
            for key in [
                'alt', 'global_pos', 'home_pos', 'local_pos','state', 'imu', 'param_value'
                ]
        }
        
        #ROS services
        service_timeout = 30
        rospy.loginfo("waiting for ROS services")
        try:
            rospy.wait_for_service('/mavros/param/set',service_timeout)
            rospy.wait_for_service('/mavros/param/get', service_timeout)
            rospy.wait_for_service('/mavros/cmd/arming', service_timeout)
            rospy.wait_for_service('/mavros/set_mode', service_timeout)
            rospy.loginfo("ROS services are up")
        except rospy.ROSException:
            rospy.logerr("ROS services are NOT up!")

        #Initialize services
        self.set_arming_uav = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_uav = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.set_param = rospy.ServiceProxy('/mavros/param/set', ParamSet)
        self.get_param = rospy.ServiceProxy('/mavros/param/get', ParamGet)
        
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
            rospy.loginfo("armed state changed from {0} to {1}".format(self.state.armed, data.armed))
            
        if self.state.connected != data.connected:
            rospy.loginfo("connected changed from {0} to {1}".format(self.state.connected, data.connected))
            
        if self.state.mode != data.mode:
            rospy.loginfo("mode changed from {0} to {1}".format(self.state.mode, data.mode))
            
        if self.state.system_status != data.system_status:
            rospy.loginfo("system_status changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_STATE'][
                    self.state.system_status].name, mavutil.mavlink.enums[
                        'MAV_STATE'][data.system_status].name))
                    
        self.state = data
        
        # Mavros publishes a disconnected state message on init
        if not self.sub_topics_ready['state'] and data.connected:
            self.sub_topics_ready['state'] = True

    """
    def set_arm(self, arm, timeout):
        
        #arm: True to arm or False to disarm, timeout(int): seconds
        rospy.loginfo("setting FCU arm: {0}".format(arm))
        old_arm = self.state.armed
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        arm_set = False
        
        for i in xrange(timeout * loop_freq):
            if self.state.armed == arm:
                arm_set = True
                rospy.loginfo("set arm success | seconds: {0} of {1}".format(i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.set_arming_uav(arm)
                    if not res.success:
                        rospy.logerr("failed to send arm command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)
                    
            try:
                rate.sleep()
            except rospy.ROSException as e:
                rospy.logerr(e)
    """ 
    def set_mode(self, mode, timeout):
        
        """mode: PX4 mode string, timeout(int): seconds"""
        rospy.loginfo("setting FCU mode: {0}".format(mode))
        old_mode = self.state.mode
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        mode_set = False
        
        for i in xrange(timeout * loop_freq):
            if self.state.mode == mode:
                mode_set = True
                rospy.loginfo("set mode success | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.set_mode_uav(0, mode)  # 0 is custom mode
                    if not res.mode_sent:
                        rospy.logerr("failed to send mode command")
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

    def set_param(self, param_id, param, timeout):
        
        """mode: PX4 mode string, timeout(int): seconds"""
        rospy.loginfo("setting FCU mode: {0}".format(param))
        old_value = self.get_param(param_id)
        old_value = old_value.value.integer
        loop_freq = 1  # Hz
        rate = rospy.Rate(loopi_freq)
        mode_set = False
        
        for i in xrange(timeout * loop_freq):
            if old_value == param_id:
                param_set = True
                rospy.loginfo("set mode success | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.set_param('param', param_id)  # 0 is custom mode
                    if not res.mode_sent:
                        rospy.logerr("failed to send parameter command")
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
    def wait_for_topics(self, timeout):
        
        """wait for simulation to be ready, make sure we're getting topic info
        from all topics by checking dictionary of flag values set in callbacks,
        timeout(int): seconds"""
        rospy.loginfo("waiting for subscribed topics to be ready")
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        simulation_ready = False
        
        for i in xrange(timeout * loop_freq):
            if all(value for value in self.sub_topics_ready.values()):
                simulation_ready = True
                rospy.loginfo("simulation topics ready | seconds: {0} of {1}".
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
