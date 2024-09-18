#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class Drawer:

	def __init__(self):
		self.pub = rospy.Publisher('/turtlesim/turtle1/cmd_vel', Twist, queue_size=10)

	def draw_square(self):
		for i in range(4):
			straight_cmd = Twist()
			straight_cmd.linear.x = 3.0
			straight_cmd.angular.z = 0
			rospy.loginfo("Moving straight:\n%s", straight_cmd)
			self.pub.publish(straight_cmd)
			rospy.sleep(1)

			turn_cmd = Twist()
			turn_cmd.angular.z = 1.57
			turn_cmd.linear.x = 0
			rospy.loginfo("Turning: \n%s", turn_cmd)
			self.pub.publish(turn_cmd)
			rospy.sleep(1)

if __name__ == '__main__':
	try:
		rospy.init_node('drawer', anonymous=True)
		d = Drawer()
		d.draw_square()
	except rospy.ROSInterruptException:
		pass
