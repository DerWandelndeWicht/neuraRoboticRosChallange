#!/usr/bin/env python3

import cv2 as cv
import numpy as np
import os
from sensor_msgs.msg import Image, PointCloud
from edge_detection.srv import edgeDetector, edgeDetectorResponse
import sys
import rospy
from std_msgs.msg import UInt32, UInt8
from cv_bridge import CvBridge
import message_filters

def callback(rgb_img, depth_img):
    bridge = CvBridge()
    pub = rospy.Publisher('depth_img_pub', PointCloud, queue_size=100)
    rospy.wait_for_service('canny_edge_detector')
    try:
        edge_img_detector = rospy.ServiceProxy('canny_edge_detector', edgeDetector)
        ros_edge_img = edge_img_detector(rgb_img)
        
        cv_edge_img = bridge.imgmsg_to_cv2(ros_edge_img.edgeImage)
        edge_mask = cv.cvtColor(cv_edge_img, cv.COLOR_BGR2GRAY)
        edge_pxl  = np.array(np.where(edge_mask>=100))
        edge_pxl = np.transpose(edge_pxl)

        cv_depth_img = bridge.imgmsg_to_cv2(depth_img)
        depth_vals = cv_depth_img[edge_mask>=100]
        
        print(edge_pxl.shape, np.expand_dims(depth_vals,1).shape)

        depth_pointcload = PointCloud()
        depth_pointcload.points = np.concatenate(
            [edge_pxl, np.expand_dims(depth_vals,1)], axis=1)
        pub.publish(bridge.cv2_to_imgmsg(depth_pointcload))

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def detect_rosbag_edges():
        
    rospy.init_node('bag_edge_detector', anonymous=True)
    img_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
    deph_sub = message_filters.Subscriber("/camera/depth/image_rect_raw", Image)

    ts = message_filters.TimeSynchronizer([img_sub, deph_sub], queue_size=100)
    ts.registerCallback(callback)

    rospy.spin()

if __name__ == "__main__":
    detect_rosbag_edges()