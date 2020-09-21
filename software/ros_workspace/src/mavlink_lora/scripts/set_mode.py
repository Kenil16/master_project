#!/usr/bin/env python
#/***************************************************************************
# MavLink LoRa node (ROS) param list example script
# Copyright (c) 2018, Kjeld Jensen <kjen@mmmi.sdu.dk> <kj@kjen.dk>
# SDU UAS Center, http://sdu.dk/uas 
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#****************************************************************************
'''
This example script shows how to set mode on a drone.

It has been tested using a Pixhawk 2.1 flight controller running PX4 and on
an AutoQuad flight controller. Please remember to set the correct
target_system value below.

Revision
2018-06-13 FMA First published version
2019-03-14 FMA Redid the script to properly utilize custom modes for px4 as example
'''

# parameters
mavlink_lora_cmd_ack_topic = '/mavlink_interface/command/ack'
mavlink_lora_set_mode_pub_topic = '/mavlink_interface/command/set_mode'

# imports
import rospy
from mavlink_lora.msg import mavlink_lora_command_set_mode, mavlink_lora_command_ack

# variables
target_sys = 0 # reset by first message
target_comp = 0


def on_mavlink_msg (msg):
    # print cmd ack
    print(msg)


def send_mavlink_set_mode(mode, custom_mode, custom_sub_mode):
    # make msg and publish it
    msg = mavlink_lora_command_set_mode()
    msg.mode = mode
    msg.custom_mode = custom_mode
    msg.custom_sub_mode = custom_sub_mode

    mavlink_set_mode_pub.publish(msg)


# launch node
rospy.init_node('mavlink_lora_set_mode')

# ack sub
rospy.Subscriber(mavlink_lora_cmd_ack_topic, mavlink_lora_command_ack, on_mavlink_msg)

# pubs
mavlink_set_mode_pub = rospy.Publisher(mavlink_lora_set_mode_pub_topic, mavlink_lora_command_set_mode, queue_size=0)

# wait until everything is running
rospy.sleep(1)

# ##### Custom Modes ######
# base modes: https://mavlink.io/en/messages/common.html#MAV_MODE_FLAG
# PX4 specific modes: https://github.com/PX4/Firmware/blob/master/src/modules/commander/px4_custom_mode.h

# enum PX4_CUSTOM_MAIN_MODE {
# 	PX4_CUSTOM_MAIN_MODE_MANUAL = 1,
# 	PX4_CUSTOM_MAIN_MODE_ALTCTL,
# 	PX4_CUSTOM_MAIN_MODE_POSCTL,
# 	PX4_CUSTOM_MAIN_MODE_AUTO,
# 	PX4_CUSTOM_MAIN_MODE_ACRO,
# 	PX4_CUSTOM_MAIN_MODE_OFFBOARD,
# 	PX4_CUSTOM_MAIN_MODE_STABILIZED,
# 	PX4_CUSTOM_MAIN_MODE_RATTITUDE,
# 	PX4_CUSTOM_MAIN_MODE_SIMPLE /* unused, but reserved for future use */
# };
#
# enum PX4_CUSTOM_SUB_MODE_AUTO {
# 	PX4_CUSTOM_SUB_MODE_AUTO_READY = 1,
# 	PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF,
# 	PX4_CUSTOM_SUB_MODE_AUTO_LOITER,
# 	PX4_CUSTOM_SUB_MODE_AUTO_MISSION,
# 	PX4_CUSTOM_SUB_MODE_AUTO_RTL,
# 	PX4_CUSTOM_SUB_MODE_AUTO_LAND,
# 	PX4_CUSTOM_SUB_MODE_AUTO_RTGS,
# 	PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET,
# 	PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND
# };

# Using custom mode requires to send 1 as basemode, as that means: Enabled_custom_modes
# Some flightmodes could also be done though basemodes, but custom modes means we utilize the exact flightmodes PX4 has

print("set_mode: Auto Hold (Loiter)")
send_mavlink_set_mode(1, 4, 3)

rospy.sleep(5)

print("set_mode: STABILIZED")
send_mavlink_set_mode(1, 7, 0)

rospy.sleep(5)

print("set_mode: Altitude Control")
send_mavlink_set_mode(1, 2, 0)

rospy.sleep(5)
