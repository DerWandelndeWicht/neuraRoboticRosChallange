#!/usr/bin/env python3

from __future__ import print_function
import cv2 as cv
import numpy as np
import os
from sensor_msgs.msg import Image
from edge_detection.srv import edgeDetector, edgeDetectorResponse
import sys
import rospy
from std_msgs.msg import UInt32, UInt8
from cv_bridge import CvBridge



def get_dir_imgs():
    bridge = CvBridge()
    dir = "/home/ros/catkin_ws/src/edge_detection_ros_challenge/edge_detection_ros/neuraRoboticRosChallange/edge_detection/data"
    dst_dir = os.path.join(dir, "edge_imgs")
    os.makedirs(dst_dir, exist_ok=True)
    
    for i in range(0,len(os.listdir(dir))):
        file = os.listdir(dir)[i]
        if not(file.endswith("png") or file.endswith("jpg")):
            continue
        img = cv.imread(os.path.join(dir, file))
        ros_img = bridge.cv2_to_imgmsg(img, encoding="passthrough")
        rospy.wait_for_service('canny_edge_detector')
        try:
            edge_img_detector = rospy.ServiceProxy('canny_edge_detector', edgeDetector)
            ros_edge_img = edge_img_detector(ros_img)
            edge_img = bridge.imgmsg_to_cv2(ros_edge_img.edgeImage)
            cv.imwrite(os.path.join(dst_dir, file), edge_img)
            print(f"Image saved at: {os.path.join(dst_dir, file)}")
   
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

if __name__ == "__main__":
    
    get_dir_imgs()