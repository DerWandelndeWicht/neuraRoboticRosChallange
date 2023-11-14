#!/usr/bin/env python3

import cv2 as cv
import numpy as np
import os
from sensor_msgs.msg import Image
from edge_detection.srv import edgeDetector, edgeDetectorResponse
import sys
import rospy
from std_msgs.msg import UInt32, UInt8
from cv_bridge import CvBridge


def callback(data):
    pub = rospy.Publisher('edge_image_publisher', Image, queue_size=100)
    rospy.wait_for_service('canny_edge_detector')
    try:
        edge_img_detector = rospy.ServiceProxy('canny_edge_detector', edgeDetector)
        ros_edge_img = edge_img_detector(data)
        
        pub.publish(ros_edge_img.edgeImage)
            

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def detect_rosbag_edges():
    
    rospy.init_node('bag_edge_detector', anonymous=True)

    rospy.Subscriber("/camera/color/image_raw", Image, callback)

    rospy.spin()

if __name__ == "__main__":
    detect_rosbag_edges()