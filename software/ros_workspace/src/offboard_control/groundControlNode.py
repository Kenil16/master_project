#!/usr/bin/env python

'''
| Keypress | Action |
|:-----:|:--------|
|t | DroneCore: takeoff + offboard + loiter mode |
|o | DroneCore: offboard + loiter (useful if something stops during operation) |
|v | DroneCore: vision guided landing | (not implemented)
|m | DroneCore: mission | (not implemented)
|h | DroneCore: move the drone to home | (not implemented)
|k | DroneCore: kill switch | (not implemented)
|r | DroneCore: reset ROS framework | (not implemented)

|l | Unassigned: logging | (not implemented)

|wasd | Loiterpilot: forwards, left, back, right |
|qe   | Loiterpilot: yaw ccw/cw (respectively) |
|zx   | Loiterpilot: decrease/increase altitude |
'''

# parameters
mavlink_lora_keypress_pub_topic = '/gcs/command'
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
		rospy.init_node('keypressNode', disable_signals = True)
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

			
			if msg.data == 27: # esc
				print("esc pressed!")
				rospy.signal_shutdown('User quit')
			else:
				self.keypress_pub.publish(msg)
			# sleep the defined interval
			self.rate.sleep()

if __name__ == '__main__':
	kbd = keypress_node()
	kbd.run()
