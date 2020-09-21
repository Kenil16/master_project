#!/usr/bin/env python
#/***************************************************************************
# MavLink LoRa node (ROS) example script
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
This example script shows how to obtain RSSI of the SIK radios.

NOT YET: It has been tested using a Pixhawk 2.1 flight controller running PX4 and on
an AutoQuad flight controller. Please remember to set the correct
target_system value below.

Revision
2019-03-12 FMA First version
2018-03-14 FMA Cleaned scripts and made them better as examples
'''

# parameters
mavlink_lora_radio_status_topic = '/mavlink_radio_status'

# imports
import rospy
from mavlink_lora.msg import mavlink_lora_radio_status

'''
NOTE: This only works if a heartbeat is sent. Otherwise the SIK radios isn't injecting radio_status packages.
Either enable so mavlink_lora sends heartbeats with "heartbeats:=true", or have another node that publish 
heartbeats to /mavlink_heartbeat_tx
'''


class rssi_node:
	def __init__(self):

		# launch node
		rospy.init_node('mavlink_lora_radio_status')

		# subs
		rospy.Subscriber(mavlink_lora_radio_status_topic, mavlink_lora_radio_status, self.on_mavlink_msg)

		# wait until everything is running
		rospy.sleep(1)
	
	def on_mavlink_msg(self, msg):
		# print rssi and remote rssi
		print("RSSI: {0} dbm, Remote RSSI: {1} dbm".format(msg.rssi, msg.remrssi))

	def run(self):
		# loop until shutdown
		while not (rospy.is_shutdown()):
			# spin until shutdown
			rospy.spin()


if __name__ == '__main__':
	gcs = rssi_node()
	gcs.run()
	

