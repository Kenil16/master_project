#!/usr/bin/env python

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

		#Init ROS
		rospy.init_node('keypressNode', disable_signals = True)
		self.rate = rospy.Rate(update_interval)

                #Publishers 
		self.keypress_pub = rospy.Publisher(mavlink_lora_keypress_pub_topic, Int8, queue_size=0)

        #Inspiration from https://stackoverflow.com/questions/510357/how-to-read-a-single-character-from-the-user
	def getch(self):
		fd = sys.stdin.fileno()
		old_settings = termios.tcgetattr(fd)
		try:
		    tty.setraw(fd)
		    ch = sys.stdin.read(1)
		finally:
		    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
		return ch
	
        def run(self):
		while not (rospy.is_shutdown()):
			msg = Int8()
			ch = self.getch()
			msg.data = ord(ch)
			
			if msg.data == 27: # esc
				print("Esc pressed!")
				rospy.signal_shutdown('User quit')
			else:
				self.keypress_pub.publish(msg)
			
                        self.rate.sleep()

if __name__ == '__main__':
	kbd = keypress_node()
	kbd.run()
