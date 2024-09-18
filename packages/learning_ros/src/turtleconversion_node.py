#!/usr/bin/env python3

import rospy
from turtlesim_helper.msg import UnitsLabelled

class DistanceConverter:

	def __init__(self):   
		rospy.Subscriber('/turtlesim/turtle1/distance', UnitsLabelled, self.callback)

		self.pub = rospy.Publisher('/turtlesim/turtle1/distance_converted', UnitsLabelled, queue_size=10)

		self.units = 'smoots'


	def callback(self, msg):
		self.units = rospy.get_param('units')
		value = msg.value
		# No need to handle conversion for meters
		if self.units == 'feet':
			value *= 3.28084
		elif self.units == 'smoots':
			value /= 1.7018

		converted_msg = UnitsLabelled()
		converted_msg.units = self.units
		converted_msg.value = value
		
		rospy.loginfo(f"Converted {msg.value:.2f} {msg.units} to {value:.2f} {self.units}")
		self.pub.publish(converted_msg)


if __name__ == '__main__':
	try:
		rospy.init_node('distance_converter', anonymous=True)
		DistanceConverter()

		rospy.spin()
	except rospy.ROSInterruptException:
		pass
