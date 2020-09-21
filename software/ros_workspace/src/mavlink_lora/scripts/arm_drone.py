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
This example script shows how to arm a drone.

It has been tested using a Pixhawk 2.1 flight controller running PX4 and on
an AutoQuad flight controller. Please remember to set the correct
target_system value below.

Revision
2018-06-13 FMA First published version
2018-03-14 FMA Cleaned scripts and made them better as examples
'''

# parameters
mavlink_lora_cmd_ack_topic = '/mavlink_interface/command/ack'
mavlink_lora_arm_pub_topic = '/mavlink_interface/command/arm_disarm'
mavlink_lora_start_mission_pub_topic = '/mavlink_interface/command/start_mission'
update_interval = 0.2


# imports
import rospy
from mavlink_lora.msg import mavlink_lora_command_start_mission, mavlink_lora_command_ack
from std_msgs.msg import Bool

# variables
target_sys = 0 # reset by first message
target_comp = 0
home_lat = 55.057883
home_lon = 10.569678


def on_mavlink_msg (msg):
    # print ack message
    print(msg)


def send_mavlink_arm():
    # make msg and publish it
    msg = Bool()
    msg.data = True

    mavlink_arm_pub.publish(msg)


def send_mavlink_start_mission():
    # make msg and publish it
    cmd = mavlink_lora_command_start_mission()

    # last item isn't needed as PX4 doesn't use last_item for anything.
    cmd.first_item = 0
    cmd.last_item = 0

    mavlink_start_mission_pub.publish(cmd)


# launch node
rospy.init_node('mavlink_lora_arm_disarm')

# ack sub
rospy.Subscriber(mavlink_lora_cmd_ack_topic, mavlink_lora_command_ack, on_mavlink_msg) # mavlink_msg subscriber

mavlink_arm_pub = rospy.Publisher(mavlink_lora_arm_pub_topic, Bool, queue_size=0)
mavlink_start_mission_pub = rospy.Publisher(mavlink_lora_start_mission_pub_topic, mavlink_lora_command_start_mission, queue_size=0)

rospy.sleep(1) # wait until everything is running

# sends arm command. Start mission can also be sent instead.
# If it got valid mission on the flight controller and is in mission mode, arming will start the mission
# If it got valid mission on flight controller and send "start mission" it will arm the drone and start flying mission
print("Sending arm command")
send_mavlink_arm()

# print("Sending start mission command")
# send_mavlink_start_mission()

# sleep for 10sec to ensure we get the cmd ack back and print it before we exits
rospy.sleep(10)
