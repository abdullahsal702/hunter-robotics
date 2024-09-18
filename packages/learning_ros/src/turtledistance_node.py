#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from turtlesim_helper.msg import UnitsLabelled
import math

class DistanceCalculator:

    def __init__(self):   
        rospy.Subscriber('/turtlesim/turtle1/pose', Pose, self.callback)
        
        # New topic
        self.pub = rospy.Publisher('/turtlesim/turtle1/distance', UnitsLabelled, queue_size=10)
        
        # Initial position
        self.prev_x = None
        self.prev_y = None

    def callback(self, msg):
    	# Don't publish anything if turtle hasn't moved
        if self.prev_x is not None and self.prev_y is not None:
            distance = math.sqrt((msg.x - self.prev_x)**2 + (msg.y - self.prev_y)**2)
            rospy.loginfo(f"Distance traveled: {distance:.2f} meters")

            distance_msg = UnitsLabelled()
            distance_msg.value = distance
            distance_msg.units = "meters"
            self.pub.publish(distance_msg)

        self.prev_x = msg.x
        self.prev_y = msg.y

if __name__ == '__main__':
    try:
        rospy.init_node('distance_calculator', anonymous=True)
        DistanceCalculator()
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
