#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float64

class Stream:

    def __init__(self):

        self.count = 0
        self.start_time = rospy.get_time()
        

    def read(self, event=None):

        self.count += 1
        self.timer(self.count)
        
        
        

    def timer(self,count):

        current_time =rospy.get_time()
        diff_time = current_time - self.start_time
        # print("diff_time",diff_time)
        if (math.modf(diff_time)[1] == 1.0):
            print("stop count" ,count)
            self.count = 0
            self.start_time = rospy.get_time()


    def publish_temperature(self, event=None):
        msg = Float64()
        msg.data = self.temperature
        self.temperature_publisher.publish(msg)

if __name__ == '__main__':
    rospy.init_node("your_sensor_node")

    # Create an instance of Temperature sensor
    stream = Stream()

    # Create a ROS Timer for reading data
    rospy.Timer(rospy.Duration(1.0/10.0),stream.read)

    # Don't forget this or else the program will exit
    rospy.spin()

