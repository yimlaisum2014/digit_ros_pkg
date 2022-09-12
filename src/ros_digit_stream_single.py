#!/usr/bin/env python3

from itertools import count
import rospy
import time
import math
import numpy as np
import cv2
from cv_bridge import CvBridge
from digit_interface.digit import Digit
from sensor_msgs.msg import Image, CompressedImage

class Stream(object):
    def __init__(self):                 

        self.count = 0
        self.start_time = rospy.get_time()
        self.ros_left_image = Image()
        # Parameter setting
        color_intensity = rospy.get_param("color_intensity")               
        r, g, b = color_intensity[0],color_intensity[1],color_intensity[2] 

        self.bridge = CvBridge()

        # Digit set up
        self.digit_left = Digit("D20365", "Left Gripper")   
        self.connect(self.digit_left)  
        self.set_intensity(r,g,b)      

        # Publisher
        self.pub_left = rospy.Publisher("/finger_left", Image, queue_size=1)  


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

        for camera in {self.digit_left}:
            if camera is not None:
                camera.set_intensity_rgb(r,g,b)

    def cv_2_rosmsg(self, cv_left_image):
        ros_left_image = self.bridge.cv2_to_imgmsg(cv_left_image, encoding='passthrough')
        
        return ros_left_image

    def read_digit_data(self,timer_info):
        
        cv_left_image = self.digit_left.get_frame()
        # cv_left_image = None
        # while cv_left_image is None:
        #     try:
        #         # Get data from digit
        #         cv_left_image = self.digit_left.get_frame()
        #     except:
        #         print("Failed to get frame......")
        #         pass
        # gray = cv2.cvtColor(cv_left_image, cv2.COLOR_RGB2GRAY)
        # cv2.imwrite(f"before_{self.count}.png",cv_left_image)

        # convent to rosmsg (Image)
        self.ros_left_image = self.cv_2_rosmsg(cv_left_image)
        self.ros_left_image.header.stamp = rospy.Time.now()
        # self.ros_left_image.header.seq = self.count
        # self.ros_left_image.header.frame_id = "left_digit"
        self.ros_left_image.header.frame_id = str(self.count)

        self.count += 1
        # count valid image count
        # if (cv_left_image.all() != None):
        #     self.count += 1
        #     self.timer(self.count)

        # check convent sucess
        # assert type(cv_left_image) == Image 
        # return self.count, self.ros_left_image

    def timer(self,count):

        current_time =rospy.get_time()
        diff_time = current_time - self.start_time
        # print("diff_time",diff_time)
        if (math.modf(diff_time)[1] == 1.0):
            print("stop count" ,count)
            self.count = 0
            self.start_time = rospy.get_time()

    def publish_digit_data(self,timer_info):
        print(self.ros_left_image.header.frame_id)
        self.pub_left.publish(self.ros_left_image)   

if __name__ == "__main__":
    rospy.init_node("stream_digit",anonymous=False)
    rospy.set_param("color_intensity",[5,5,5])

    stream = Stream()
    # For reading # digit - 60FPS 1s get 60 images // 0.0167s = 16.7ms for 1 images // read timer should > 16.7ms & set 0.03s = 30ms
    rospy.Timer(rospy.Duration(1.0),stream.read_digit_data)

    # For pulblishing 
    rospy.Timer(rospy.Duration(1.0),stream.publish_digit_data)
    rospy.spin()

    

