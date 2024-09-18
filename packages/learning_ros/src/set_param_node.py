#!/usr/bin/env python3

import rospy

class ParamSetter:

    def __init__(self):
        self.units_param = 'units'
        self.units = ['meters', 'feet', 'smoots']
        self.index = 0
        self.rate = rospy.Rate(0.2) # 0.2Hz = 5 seconds 

    def set_param(self):
        while not rospy.is_shutdown():
            current_unit = self.units[self.index % len(self.units)] # Loops through units array
            rospy.set_param(self.units_param, current_unit)
            rospy.loginfo(f"Set param {self.units_param} to {current_unit}")
            self.index += 1
            self.rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('param_setter', anonymous=True)
        p = ParamSetter()
        p.set_param()
    except rospy.ROSInterruptException:
        pass
