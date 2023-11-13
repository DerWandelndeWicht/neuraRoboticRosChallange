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
from data_converter import dataConverter

def get_dir_imgs():
    dc = dataConverter
    dir = "/home/ros/catkin_ws/src/edge_detection_ros_challenge/edge_detection_ros/edge_detection/data"
    dst_dir = os.path.join(dir, "edge_imgs")
    os.makedirs(dst_dir, exist_ok=True)
    
    for i in range(0,len(os.listdir(dir))):
        file = os.listdir(dir)[i]
        if not(file.endswith("png") or file.endswith("jpg")):
            continue
        img = cv.imread(os.path.join(dir, file))
        # ros_img = Image
        # ros_img.height = np.uint32(img.shape[0])
        # ros_img.width = np.uint32(img.shape[1])
        # ros_img.step = np.uint32(0)
        # ros_img.data = list(np.array(img.flatten(), dtype=np.uint8))
        ros_img = dc.img2rosImage(img)
        rospy.wait_for_service('canny_edge_detector')
        try:
            edge_img_detector = rospy.ServiceProxy('canny_edge_detector', edgeDetector)
            ros_edge_img = edge_img_detector(ros_img.height, ros_img.width, ros_img.step, ros_img.data)

            edge_img = np.reshape(ros_edge_img, (img.shape))
            cv.imwrite(os.path.join(dst_dir, edge_img))
            
   
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

if __name__ == "__main__":
    
    get_dir_imgs()