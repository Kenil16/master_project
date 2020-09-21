#!/usr/bin/env python
# /***************************************************************************
# MavLink LoRa node (ROS) upload mission example script
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
# ****************************************************************************
'''
This example script shows how to upload a mission with more than 20 items to a drone.
This is mainly used to test upload time of large missions while flying.

It has been tested using a Pixhawk 2.1 flight controller running PX4 and on
an AutoQuad flight controller. Please remember to set the correct
target_system value below.

Revision
2018-06-13 FMA First published version
2018-03-14 FMA Cleaned scripts and made them better as examples
'''
# parameters
mavlink_lora_pub_topic = 'mavlink_interface/mission/mavlink_upload_mission'

# imports
import rospy
from mavlink_lora.msg import mavlink_lora_mission_list
from mavlink_lora.msg import mavlink_lora_mission_item_int, mavlink_lora_mission_ack

time_start = 0
time_end = 0


def on_ack_received_callback(msg):
    global time_start, time_end

    # Calculate the time used
    time_end = rospy.get_time()
    diff_time = time_end - time_start
    print("Time taken for transmission:" + str(diff_time))

    rospy.signal_shutdown("user quit")


# variables
target_sys = 0  # reset by first message
target_comp = 0
home_lat = 55.4720390
home_lon = 10.4147298

# launch node
rospy.init_node('mavlink_lora_mission_upload')

# pubs
mavlink_msg_pub = rospy.Publisher(mavlink_lora_pub_topic, mavlink_lora_mission_list, queue_size=0)

# subs
mavlink_mission_ack_sub = rospy.Subscriber("mavlink_interface/mission/ack", mavlink_lora_mission_ack,
                                           on_ack_received_callback)

# wait until everything is running
rospy.sleep(1)

# make 20+ waypoints and save in list.
missionlist = mavlink_lora_mission_list()

# CHANGE SPEED
speed = mavlink_lora_mission_item_int()
speed.target_system = 0
speed.target_component = 0
speed.seq = 0
speed.frame = 2  # mission command frame
speed.command = 178
speed.param1 = 5  # air_speed
speed.param2 = 5  # m/s
speed.param3 = -1  # no change
speed.param4 = 0  # abosulte or relative. relative = 1
speed.autocontinue = 1

speed2 = mavlink_lora_mission_item_int()
speed2.target_system = 0
speed2.target_component = 0
speed2.seq = 1
speed2.frame = 2  # mission command frame
speed2.command = 178
speed2.param1 = 1  # air_speed
speed2.param2 = 5  # m/s
speed2.param3 = -1  # no change
speed2.param4 = 0  # abosulte or relative. relative = 1
speed2.autocontinue = 1

# TAKEOFF waypoint
way1 = mavlink_lora_mission_item_int()
way1.target_system = 0
way1.target_component = 0
way1.seq = 2
way1.frame = 6  # global pos, relative alt_int
way1.command = 22
way1.x = home_lat * 10000000
way1.y = home_lon * 10000000
way1.z = 20
way1.param1 = 5
way1.current = 1
way1.autocontinue = 1

# WAYPOINT 1
way2 = mavlink_lora_mission_item_int()
way2.target_system = 0
way2.target_component = 0
way2.seq = 3
way2.frame = 6  # global pos, relative alt_int
way2.command = 16
way2.param1 = 0  # hold time
way2.param2 = 5  # acceptance radius in m
way2.param3 = 0  # pass though waypoint, no trajectory control
way2.x = 55.4720010 * 10000000
way2.y = 10.4164463 * 10000000
way2.z = 20
way2.autocontinue = 1

# WAYPOINT 2
way3 = mavlink_lora_mission_item_int()
way3.target_system = 0
way3.target_component = 0
way3.seq = 4
way3.frame = 6  # global pos, relative alt_int
way3.command = 16
way3.param1 = 0  # hold time
way3.param2 = 5  # acceptance radius in m
way3.param3 = 0  # pass though waypoint, no trajectory control
way3.x = 55.4720615 * 10000000
way3.y = 10.4161885 * 10000000
way3.z = 40
way3.autocontinue = 1

# WAYPOINT 3
way4 = mavlink_lora_mission_item_int()
way4.target_system = 0
way4.target_component = 0
way4.seq = 5
way4.frame = 6  # global pos, relative alt_int
way4.command = 16
way4.param1 = 0  # hold time
way4.param2 = 5  # acceptance radius in m
way4.param3 = 0  # pass though waypoint, no trajectory control
way4.x = 55.4721370 * 10000000
way4.y = 10.4164410 * 10000000
way4.z = 30
way4.autocontinue = 1

# WAYPOINT 4
way5 = mavlink_lora_mission_item_int()
way5.target_system = 0
way5.target_component = 0
way5.seq = 6
way5.frame = 6  # global pos, relative alt_int
way5.command = 16
way5.param1 = 0  # hold time
way5.param2 = 5  # acceptance radius in m
way5.param3 = 0  # pass though waypoint, no trajectory control
way5.x = 55.4720680 * 10000000
way5.y = 10.4161800 * 10000000
way5.z = 20
way5.autocontinue = 1

way6 = mavlink_lora_mission_item_int()
way6.target_system = 0
way6.target_component = 0
way6.seq = 7
way6.frame = 6  # global pos, relative alt_int
way6.command = 16
way6.param1 = 0  # hold time
way6.param2 = 5  # acceptance radius in m
way6.param3 = 0  # pass though waypoint, no trajectory control
way6.x = 55.4720650 * 10000000
way6.y = 10.4161785 * 10000000
way6.z = 20
way6.autocontinue = 1

way7 = mavlink_lora_mission_item_int()
way7.target_system = 0
way7.target_component = 0
way7.seq = 8
way7.frame = 6  # global pos, relative alt_int
way7.command = 16
way7.param1 = 0  # hold time
way7.param2 = 5  # acceptance radius in m
way7.param3 = 0  # pass though waypoint, no trajectory control
way7.x = 55.4720600 * 10000000
way7.y = 10.4161862 * 10000000
way7.z = 20
way7.autocontinue = 1

way8 = mavlink_lora_mission_item_int()
way8.target_system = 0
way8.target_component = 0
way8.seq = 9
way8.frame = 6  # global pos, relative alt_int
way8.command = 16
way8.param1 = 0  # hold time
way8.param2 = 5  # acceptance radius in m
way8.param3 = 0  # pass though waypoint, no trajectory control
way8.x = 55.4720667 * 10000000
way8.y = 10.4161899 * 10000000
way8.z = 20
way8.autocontinue = 1

way9 = mavlink_lora_mission_item_int()
way9.target_system = 0
way9.target_component = 0
way9.seq = 10
way9.frame = 6  # global pos, relative alt_int
way9.command = 16
way9.param1 = 0  # hold time
way9.param2 = 5  # acceptance radius in m
way9.param3 = 0  # pass though waypoint, no trajectory control
way9.x = 55.4720636 * 10000000
way9.y = 10.4161800 * 10000000
way9.z = 20
way9.autocontinue = 1

way10 = mavlink_lora_mission_item_int()
way10.target_system = 0
way10.target_component = 0
way10.seq = 11
way10.frame = 6  # global pos, relative alt_int
way10.command = 16
way10.param1 = 0  # hold time
way10.param2 = 5  # acceptance radius in m
way10.param3 = 0  # pass though waypoint, no trajectory control
way10.x = 55.4720699 * 10000000
way10.y = 10.4161885 * 10000000
way10.z = 20
way10.autocontinue = 1

way11 = mavlink_lora_mission_item_int()
way11.target_system = 0
way11.target_component = 0
way11.seq = 12
way11.frame = 6  # global pos, relative alt_int
way11.command = 16
way11.param1 = 0  # hold time
way11.param2 = 5  # acceptance radius in m
way11.param3 = 0  # pass though waypoint, no trajectory control
way11.x = 55.4720615 * 10000000
way11.y = 10.4161890 * 10000000
way11.z = 20
way11.autocontinue = 1

way12 = mavlink_lora_mission_item_int()
way12.target_system = 0
way12.target_component = 0
way12.seq = 13
way12.frame = 6  # global pos, relative alt_int
way12.command = 16
way12.param1 = 0  # hold time
way12.param2 = 5  # acceptance radius in m
way12.param3 = 0  # pass though waypoint, no trajectory control
way12.x = 55.4720680 * 10000000
way12.y = 10.4161822 * 10000000
way12.z = 20
way12.autocontinue = 1

way13 = mavlink_lora_mission_item_int()
way13.target_system = 0
way13.target_component = 0
way13.seq = 14
way13.frame = 6  # global pos, relative alt_int
way13.command = 16
way13.param1 = 0  # hold time
way13.param2 = 5  # acceptance radius in m
way13.param3 = 0  # pass though waypoint, no trajectory control
way13.x = 55.4720622 * 10000000
way13.y = 10.4161885 * 10000000
way13.z = 20
way13.autocontinue = 1

way14 = mavlink_lora_mission_item_int()
way14.target_system = 0
way14.target_component = 0
way14.seq = 15
way14.frame = 6  # global pos, relative alt_int
way14.command = 16
way14.param1 = 0  # hold time
way14.param2 = 5  # acceptance radius in m
way14.param3 = 0  # pass though waypoint, no trajectory control
way14.x = 55.4720695 * 10000000
way14.y = 10.4161800 * 10000000
way14.z = 20
way14.autocontinue = 1

way15 = mavlink_lora_mission_item_int()
way15.target_system = 0
way15.target_component = 0
way15.seq = 16
way15.frame = 6  # global pos, relative alt_int
way15.command = 16
way15.param1 = 0  # hold time
way15.param2 = 5  # acceptance radius in m
way15.param3 = 0  # pass though waypoint, no trajectory control
way15.x = 55.4720620 * 10000000
way15.y = 10.4161876 * 10000000
way15.z = 20
way15.autocontinue = 1

way16 = mavlink_lora_mission_item_int()
way16.target_system = 0
way16.target_component = 0
way16.seq = 17
way16.frame = 6  # global pos, relative alt_int
way16.command = 16
way16.param1 = 0  # hold time
way16.param2 = 5  # acceptance radius in m
way16.param3 = 0  # pass though waypoint, no trajectory control
way16.x = 55.4720600 * 10000000
way16.y = 10.4161885 * 10000000
way16.z = 20
way16.autocontinue = 1

way17 = mavlink_lora_mission_item_int()
way17.target_system = 0
way17.target_component = 0
way17.seq = 18
way17.frame = 6  # global pos, relative alt_int
way17.command = 16
way17.param1 = 0  # hold time
way17.param2 = 5  # acceptance radius in m
way17.param3 = 0  # pass though waypoint, no trajectory control
way17.x = 55.4720699 * 10000000
way17.y = 10.4161823 * 10000000
way17.z = 20
way17.autocontinue = 1

# WAYPOINT 4 LANDING
land = mavlink_lora_mission_item_int()
land.target_system = 0
land.target_component = 0
land.seq = 19
land.frame = 6  # global pos, relative alt_int
land.command = 21
land.param1 = 5  # abort alt
land.param2 = 0  # precision landing. 0 = normal landing
land.x = home_lat * 10000000
land.y = home_lon * 10000000
land.z = 20
land.autocontinue = 1

# add waypoints to list
missionlist.waypoints.append(speed)
missionlist.waypoints.append(speed2)
missionlist.waypoints.append(way1)
missionlist.waypoints.append(way2)
missionlist.waypoints.append(way3)
missionlist.waypoints.append(way4)
missionlist.waypoints.append(way5)
missionlist.waypoints.append(way6)
missionlist.waypoints.append(way7)
missionlist.waypoints.append(way8)
missionlist.waypoints.append(way9)
missionlist.waypoints.append(way10)
missionlist.waypoints.append(way11)
missionlist.waypoints.append(way12)
missionlist.waypoints.append(way13)
missionlist.waypoints.append(way14)
missionlist.waypoints.append(way15)
missionlist.waypoints.append(way16)
missionlist.waypoints.append(way17)
missionlist.waypoints.append(land)

time_start = rospy.get_time()  # in seconds
mavlink_msg_pub.publish(missionlist)

# loop until shutdown
while not (rospy.is_shutdown()):
    # do stuff
    rospy.sleep(5)
