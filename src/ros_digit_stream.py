#!/usr/bin/env python3

import rospy
import time
import numpy as np
import cv2
from cv_bridge import CvBridge
from digit_interface.digit import Digit
from sensor_msgs.msg import Image, CompressedImage

class Collect(object):
    def __init__(self):                 

        # Parameter setting
        color_intensity = rospy.get_param("color_intensity")               
        r, g, b = color_intensity[0],color_intensity[1],color_intensity[2] 

        self.bridge = CvBridge()

        self.digit_right = Digit("D20356", "Right Gripper") 
        self.digit_left = Digit("D20365", "Left Gripper")   

        self.connect(self.digit_left)  
        self.connect(self.digit_right) 
        self.set_intensity(r,g,b) 

        # Publisher
        self.pub_left = rospy.Publisher("/finger_left", Image, queue_size=10)  
        self.pub_right = rospy.Publisher("/finger_right", Image, queue_size=10)
        # self.pub_test = rospy.Publisher("/test", Image, queue_size=1) 

        
        # Time callback funtion
        self.timer = rospy.Timer(rospy.Duration(1/60), self.stream_callback)
        self.counter = 0

        # self.cv_2_rosmsg()             


    def connect(self, camera):
        connected = False
        while not connected:
            print('Try connecting....')
            try:
                camera.connect()
                print('Succeed')
                connected = True
            except:
                print('Failed')
                pass
            time.sleep(1)
    
    def set_intensity(self, r, g, b):

        for camera in {self.digit_right, self.digit_left}:
            if camera is not None:
                camera.set_intensity_rgb(r, g, b)

    def disconnect(self):
        self.digit_left.disconnect()  
        self.digit_right.disconnect()

    def cv_2_rosmsg(self):
        self.counter += 1
        print(rospy.get_time())
        cv_left_image = self.digit_left.get_frame()  
        cv_right_image = self.digit_right.get_frame()
        
        ros_left_image = self.bridge.cv2_to_imgmsg(cv_left_image, encoding='passthrough')
        ros_right_image = self.bridge.cv2_to_imgmsg(cv_right_image, encoding='passthrough')

        self.pub_left.publish(ros_left_image)   
        self.pub_right.publish(ros_right_image) 

        # if (self.counter == 20):
        #     cv2.imwrite(f"img_{time.ctime()}.png",cv_left_image)
        #     self.pub_test.publish(ros_left_image)
    
    def stream_callback(self,timer):
        self.cv_2_rosmsg()
        # print(self.cv_2_rosmsg())
        
        


if __name__ == "__main__":
    rospy.init_node("collect",anonymous=False)
    rospy.set_param("color_intensity",[5,5,5])
    collect = Collect()
    try:
        collect.cv_2_rosmsg()
        rospy.spin()
    except rospy.ROSInterruptException:
        collect.disconnect()
        print("exception thrown")
        pass
    

