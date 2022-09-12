#!/usr/bin/env python3

from itertools import count
import rospy
import time
import math
import numpy as np
import cv2
import os
from cv_bridge import CvBridge
from digit_interface.digit import Digit
from sensor_msgs.msg import Image, CompressedImage


# os.path.join(os.path.abspath(__file__), '../')

class Stream(object):
    def __init__(self,save=False,gray=False):                 

        self.count = 0
        self.start_time = rospy.get_time()
        self.ros_left_image = Image()
        self.ros_left_diff_image = Image()
        self.save = save
        self.gray_conv = gray

        self.savepath = "/home/sis/WFH-locobot/Go_2_ROS/"
        if not os.path.exists(self.savepath):
            os.makedirs(self.savepath)

        # Parameter setting
        color_intensity = rospy.get_param("color_intensity")               
        r, g, b = color_intensity[0],color_intensity[1],color_intensity[2] 

        self.bridge = CvBridge()

        # Digit set up
        self.digit_left = Digit("D20365", "Left Gripper")   
        # self.digit_left = Digit("D20356", "Right Gripper")  
        self.connect(self.digit_left)  
        self.set_intensity(r,g,b)      
        # Get init-frame
        self.ori_frame = self.digit_left.get_frame()

        # Publisher
        self.pub_left = rospy.Publisher("/finger_left", Image, queue_size=1)  
        self.pub_left_diff = rospy.Publisher("/finger_left_diff", Image, queue_size=1) 

    def save_stream(self,cv_image,gray_conv):
        # cv_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding='passthrough')
        if gray_conv == True:
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
        path = self.savepath
        name = f'img_{self.count}.jpg'
        print(os.path.join(path, name))
        cv2.imwrite(os.path.join(path, name), cv_image)

        
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
        left_diff_frame = self.digit_left.get_diff(self.ori_frame)
        if self.save == True:
            self.save_stream(cv_left_image,self.gray_conv)
        # convent to rosmsg (Image - raw)
        self.ros_left_image = self.cv_2_rosmsg(cv_left_image)
        self.ros_left_image.header.stamp = rospy.Time.now()
        self.ros_left_image.header.frame_id = str(self.count)
        
        # convent to rosmsg (Image - different)
        self.ros_left_diff_image = self.cv_2_rosmsg(left_diff_frame)
        self.ros_left_diff_image.header.stamp = rospy.Time.now()
        self.ros_left_diff_image.header.frame_id = str(self.count)
        
        self.count += 1


    def publish_digit_data(self,timer_info):
        print(self.ros_left_image.header.frame_id)
        self.pub_left.publish(self.ros_left_image)  
        self.pub_left_diff.publish(self.ros_left_diff_image)

if __name__ == "__main__":
    rospy.init_node("stream_digit",anonymous=False)
    rospy.set_param("color_intensity",[5,5,5])

    stream = Stream(save=False,gray=False)

    data_collect = False

    if data_collect:
        # For reading # digit - 60FPS 1s get 60 images // 0.0167s = 16.7ms for 1 images // read timer should > 16.7ms & set 0.03s = 30ms
        rospy.Timer(rospy.Duration(1/60.0),stream.read_digit_data)
        # For pulblishing 
        rospy.Timer(rospy.Duration(1/60.0),stream.publish_digit_data)
    else:
        rospy.Timer(rospy.Duration(1/60.0),stream.read_digit_data)
        rospy.Timer(rospy.Duration(1/6),stream.publish_digit_data)
    
    rospy.spin()

    

