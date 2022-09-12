from example.data_collect import collect
import rospy
import time
from cv_bridge import CvBridge
from digit_interface.digit import Digit
from sensor_msgs.msg import Image

class Collect(object):
    def __init__(self):                 # -> None: Return type for __init__ function is unnecessary
                                        # __init__() function stands for doing some initialization work, e.g., declare attributes, passing parameters.
                                        # So, here the better code order should be: 1) declare attributes, 2) passing parameters if needed, 3) call some initializing functions.

        # Parameter setting
        rospy.set_param("color_intensity",[5,5,5])                         # You could pass this parameters as the arguments of __init__ method, e.g., __init__(self, color_intensity=[5,5,5])
        color_intensity = rospy.get_param("color_intensity")               # Here, color_intensity exists twice, so you can create a variable to store this value.
        r, g, b = color_intensity[0],color_intensity[1],color_intensity[2] # Here is right, but a more tight way to express this is `r,g,b = color_intensity`

        self.bridge = CvBridge         # Here, in my understanding, it should be a object, not a class, because the function (cv2_to_imgmsg) you wanna call is not a static or class method.
                                       # That's the reason why you should write in this way `self.bridge = CvBridge()`.

        self.digit_right = Digit("D20356", "Right Gripper") # Here is good.
        self.digit_left = Digit("D20365", "Left Gripper")   # Here is good.

        self.connect(self.digit_left)  # These three lines are fine, but it would better not to call them here because they do something useful.
        self.connect(self.digit_right) # I mean, suppose you declare a class A, and define some functions a(), b(), it'd better to use them like this,
        self.set_intensity(r,g,b)      # obj = A(), obj.a(), obj.b(). However, currently, you push all functions inside the constructor, i.e., __init__() function.
                                       # That's not wrong, but it's not following the best practice.               

        self.cv_2_rosmsg()             # Here is wrong because you call `self.pub_left` and `self.pub_right` within this function,
                                       # while they are undefined right now.

        # Publisher
        self.pub_left = rospy.Publisher("finger_left", Image, queue_size=10)  # As aforementioned, you should declare this before you call `self.cv_2_rosmsg()`
        self.pub_right = rospy.Publisher("finger_left", Image, queue_size=10) # The same as above. I am not sure whether it works. We'll know when you test it.

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
        self.digit_left.disconnect()  # These two lines are right, but you can make it more robust like the previous function, `set_intensity()`.
        self.digit_right.disconnect()

    def cv_2_rosmsg(self):
        cv_left_image = self.digit_left.get_frame()  # Here should be working.
        cv_right_image = self.digit_right.get_frame()
        
        ros_left_image = self.bridge.cv2_to_imgmsg(cv_left_image, desired_encoding='passthrough')   # This needs testing. I am not sure until it runs.
        ros_right_image = self.bridge.cv2_to_imgmsg(cv_right_image, desired_encoding='passthrough') # The same as above.

        self.pub_left.publish(ros_left_image)   # This needs testing. I am not sure until it runs.
        self.pub_right.publish(ros_right_image) # The same as above.
        


if __name__ == "__main__":
    rospy.init_node("collect",anonymous=False)
    collect = Collect()
    if rospy.spin() : # Well, I see this functions returns nothing(None), so, the next line will never be executed since None is regarded as False.
        collect.disconnect()


# Overall, this is not gonna work...