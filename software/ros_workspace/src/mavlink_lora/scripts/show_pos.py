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
This example script shows how to obtain position of a UAV.

NOT YET: It has been tested using a Pixhawk 2.1 flight controller running PX4 and on
an AutoQuad flight controller. Please remember to set the correct
target_system value below.

Revision
2019-03-12 FMA First version
2018-03-14 FMA Cleaned scripts and made them better as examples
'''
# parameters
mavlink_lora_sub_topic = '/mavlink_rx'
mavlink_lora_pos_sub_topic = '/mavlink_pos'
update_interval = 1

# imports
import rospy
from mavlink_lora.msg import mavlink_lora_msg, mavlink_lora_pos


class pos_node:
	def __init__(self):

		# status variables
		self.last_heard = 0
		self.lat = 0.0
		self.lon = 0.0
		self.alt = 0.0

		# launch node
		rospy.init_node('mavlink_lora_pos_simple', disable_signals = True)

		# subs
		rospy.Subscriber(mavlink_lora_sub_topic, mavlink_lora_msg, self.on_mavlink_msg)
		rospy.Subscriber(mavlink_lora_pos_sub_topic, mavlink_lora_pos, self.on_mavlink_lora_pos)

		# rate to run the loop
		self.rate = rospy.Rate(update_interval)

		# wait until everything is running
		rospy.sleep(1)

	def printPos(self):
		now = rospy.get_time()

		# update last_heard
		t = now - self.last_heard
		if t < 86400:
			last_heard_text = '%ds' % t
		else:
			last_heard_text = 'Never'

		# update pos status
		if self.lat == 0 and self.lon == 0:
			pos_text = 'Unknown'
		else:
			pos_text = '%02.5f %03.5f' % (self.lat, self.lon)

		# update altitude status
		if self.alt == 0:
			alt_text = 'Unknown'
		else:
			alt_text = '%.1fm ' % (self.alt)

		print('Last heard:         {0}'.format(last_heard_text))
		print('Position:           {0}'.format(pos_text))
		print('Altitude:           {0}'.format(alt_text))
		print('\n')
	
	def on_mavlink_msg(self, msg):
		# save timestamp of last package of anything received from the drone
		self.last_heard = rospy.get_time()

	def on_mavlink_lora_pos(self, msg):
		self.lat = msg.lat
		self.lon = msg.lon
		self.alt = msg.alt

	def run(self):
		# loop until shutdown
		while not (rospy.is_shutdown()):
			# do stuff
			self.printPos()

			# sleep the defined interval
			self.rate.sleep()


if __name__ == '__main__':
	gcs = pos_node()
	gcs.run()
	

