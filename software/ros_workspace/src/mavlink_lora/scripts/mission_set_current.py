#!/usr/bin/env python
#/***************************************************************************
# MavLink LoRa node (ROS) mission set current id script
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
# (INCLUDING NEGIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#****************************************************************************
'''
Revision
2018-06-12 KJ First published version
2019-03-14 FMA Changed to use cmd long interface
'''

# other defines
ros_node_name = 'mission_set_current'
ros_node_update_interval = 10
mavlink_lora_rx_sub = '/mavlink_rx'
mavlink_lora_tx_pub = '/mavlink_tx'

# imports
from sys import argv
import rospy
from mission_lib import *


class ros_node():
	def __init__(self, mission_id_set):
		# save variables
		self.mission_id_set = mission_id_set

		# initiate variables
		self.stop = False
		self.first_msg_ok = False
		self.request_sent = False
		self.mi = mission_lib()
		self.sys_id = 0 # reset when receiving first msg
		self.comp_id = 0	

		# initiate node
		rospy.init_node(ros_node_name)
		self.mavlink_msg_pub = rospy.Publisher(mavlink_lora_tx_pub, mavlink_lora_msg, queue_size=0)
		rospy.Subscriber(mavlink_lora_rx_sub, mavlink_lora_msg, self.on_mavlink_msg)
		self.rate = rospy.Rate(ros_node_update_interval)

		# wait until everything is running (important)
		rospy.sleep (1)

	def on_mavlink_msg (self, msg):
		if self.first_msg_ok == False:
			self.first_msg_ok = True
			self.sys_id = msg.sys_id
			self.mi.set_target(self.sys_id, self.comp_id)
		if msg.msg_id == MAVLINK_MSG_ID_MISSION_CURRENT:
			print ("Received: Current mission ID is %d" % self.mi.unpack_mission_current(msg.payload))
			self.stop = True

	def send_mavlink_mission_set_current(self, mission_id):
		self.mi.msg.header.stamp = rospy.Time.now()
		self.mi.pack_mission_set_current(mission_id)
		self.mavlink_msg_pub.publish(self.mi.msg)

	def loop(self):
		while not (rospy.is_shutdown() or self.stop):
			# do stuff
			if self.request_sent == False and self.first_msg_ok == True:
				self.send_mavlink_mission_set_current(self.mission_id_set)
				self.request_sent = True	

			# sleep the defined interval
			self.rate.sleep()


if __name__ == "__main__":
	if len(argv) == 1:
		print ("Usage: mission_set_current.py [id]")
	else:
		mission_id = int(argv[1])
		print("Setting current mission ID to: %d" % mission_id)
		rn = ros_node(mission_id)
		rn.loop()

