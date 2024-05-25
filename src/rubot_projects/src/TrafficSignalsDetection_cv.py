#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class TakeAShot(object):

    def __init__(self):
        self.image_sub = rospy.Subscriber("/rubot/camera1/image_raw", Image, self.camera_callback)
        self.bridge_object = CvBridge()

    def camera_callback(self,data):
        try:
            # We select bgr8 because its the OpenCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
            cv2.imwrite("image.png", cv_image)
        except CvBridgeError as e:
            print(e)               

    def main():

        rospy.init_node('take_a_shot', anonymous=True)
        take_a_shot = TakeAShot()
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")

    if __name__ == '__main__':
        main()