#!/usr/bin/env python3

import rospy
import time
from cv_bridge import CvBridge
from digit_interface.digit import Digit
from std_msgs.msg import String
import pickle
import base64
import cv2

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
        self.pub_left = rospy.Publisher("finger_left_byte", String, queue_size=10)  
        self.pub_right = rospy.Publisher("finger_right_byte", String, queue_size=10) 
        # Time callback funtion
        self.timer = rospy.Timer(rospy.Duration(1), self.stream_callback)
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

    def img_2_byte(self, img):
        return pickle.dumps(img).encode()

    
    def img_2_base64(self, img):
        retval, buffer = cv2.imencode('.jpg', img)
        text = base64.b64encode(buffer).decode('utf-8')
        return text

    def cv_2_rosmsg(self):
        self.counter += 1
        cv_left_image = self.digit_left.get_frame()  
        cv_right_image = self.digit_right.get_frame()


        self.pub_left.publish(self.img_2_base64(cv_left_image))   
        self.pub_right.publish(self.img_2_base64(cv_right_image))  
        return self.counter
    
    def stream_callback(self,timer):
        self.cv_2_rosmsg()
        
        


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
    

