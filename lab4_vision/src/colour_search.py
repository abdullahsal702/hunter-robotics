#!/usr/bin/env python3

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image

# Import some other modules from within this package
from tb3 import Tb3Move

class ColourSearch():

    def __init__(self):
        node_name = "turn_and_face"
        rospy.init_node(node_name)

        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        self.robot_controller = Tb3Move()
        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)

        self.move_rate = "" # fast, slow or stop
        self.stop_counter = 0

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.rate = rospy.Rate(5)
        
        self.m00 = 0
        self.m00_min = 10000

    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True
    
    def camera_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        height, width, _ = cv_img.shape
        crop_width = width - 800
        crop_height = 400
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))

        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        blue_lower = (115, 224, 100)
        blue_upper = (130, 255, 255)
        blue_mask = cv2.inRange(hsv_img, blue_lower, blue_upper)
        
        red_lower = (0, 225, 100)
        red_upper = (5, 255, 255)
        red_mask = cv2.inRange(hsv_img, red_lower, red_upper)
        
        green_lower = (55, 235, 100)
        green_upper = (62, 255, 255)
        green_mask = cv2.inRange(hsv_img, green_lower, green_upper)
        
        lightblue_lower = (86, 185, 100)
        lightblue_upper = (92, 255, 255)
        lightblue_mask = cv2.inRange(hsv_img, lightblue_lower, lightblue_upper)
        
        # res = cv2.bitwise_and(crop_img, crop_img, mask = red_mask)
        blue_m = cv2.moments(blue_mask)
        red_m = cv2.moments(red_mask)
        green_m = cv2.moments(green_mask)
        lightblue_m = cv2.moments(lightblue_mask)
        moments = [blue_m, red_m, green_m, lightblue_m]
        
        # The moment that has the highest value for non-zero pixels should be used
        m = max(moments, key=lambda moment: moment['m00'])
        self.m00 = m['m00']
        self.cy = m['m10'] / (m['m00'] + 1e-5)

        if self.m00 > self.m00_min:
          cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
        
        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)

    def main(self):
        while not self.ctrl_c:
            if self.stop_counter > 0:
                self.stop_counter -= 1

            if self.m00 > self.m00_min:
                # blob detected
                if self.cy >= 560-100 and self.cy <= 560+100:
                    if self.move_rate == 'slow':
                        self.move_rate = 'stop'
                        self.stop_counter = 30
                else:
                    self.move_rate = 'slow'
            else:
                self.move_rate = 'fast'
                
            if self.move_rate == 'fast':
                print("MOVING FAST: I can't see anything at the moment, scanning the area...")
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)
            elif self.move_rate == 'slow':
                print(f"MOVING SLOW: A blob of colour of size {self.m00:.0f} pixels is in view at y-position: {self.cy:.0f} pixels.")
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
            elif self.move_rate == 'stop' and self.stop_counter > 0:
                print(f"STOPPED: The blob of colour is now dead-ahead at y-position {self.cy:.0f} pixels... Counting down: {self.stop_counter}")
                self.robot_controller.set_move_cmd(0.0, 0.0)
            else:
                print(f"MOVING SLOW: A blob of colour of size {self.m00:.0f} pixels is in view at y-position: {self.cy:.0f} pixels.")
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
            
            self.robot_controller.publish()
            self.rate.sleep()
            
if __name__ == '__main__':
    node = ColourSearch()
    try:
        node.main()
    except rospy.ROSInterruptException:
        pass