#!/usr/bin/env python
#/***************************************************************************
# MavLink LoRa node (ROS) example script
# Copyright (c) 2018, Kjeld Jensen <kjen@mmmi.sdu.dk> <kj@kjen.dk>
# SDU UAS Center, http://sdu.dk/uas 
# All rights reserved.
#
# This software is based on Python keypress software written by 
# Steven D'Aprano released under the MIT license.
# https://opensource.org/licenses/MIT
# http://code.activestate.com/recipes/577977-get-single-keypress/
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

# parameters
mavlink_lora_keypress_pub_topic = '/keypress'
update_interval = 10

# imports
import rospy
from std_msgs.msg import Int8
import sys
import termios
import tty

class keypress_node:
	def __init__(self):

		# launch node
		rospy.init_node('mavlink_lora_gcs_keypress', disable_signals = True)
		self.rate = rospy.Rate(update_interval)
		self.keypress_pub = rospy.Publisher(mavlink_lora_keypress_pub_topic, Int8, queue_size=0)

	def getch(self):
		"""
		getch() -> key character

		Read a single keypress from stdin and return the resulting character. 
		Nothing is echoed to the console. This call will block if a keypress 
		is not already available, but will not wait for Enter to be pressed. 

		If the pressed key was a modifier key, nothing will be detected; if
		it were a special function key, it may return the first character of
		of an escape sequence, leaving additional characters in the buffer.
		"""
		fd = sys.stdin.fileno()
		old_settings = termios.tcgetattr(fd)
		try:
		    tty.setraw(fd)
		    ch = sys.stdin.read(1)
		finally:
		    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
		return ch

	def run(self):
		# loop until shutdown
		while not (rospy.is_shutdown()):
			# do stuff
			msg = Int8()
			ch = self.getch()
			msg.data = ord(ch)
			self.keypress_pub.publish(msg)

			if ch == 'q':
				rospy.signal_shutdown('User quit')

			# sleep the defined interval
			self.rate.sleep()

if __name__ == '__main__':
	kbd = keypress_node()
	kbd.run()



