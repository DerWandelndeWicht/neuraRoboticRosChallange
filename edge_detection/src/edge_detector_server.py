#!/usr/bin/env python3

import rospy
import cv2 as cv
import numpy as np
import os
from sensor_msgs.msg import Image

from edge_detection.srv import edgeDetector, edgeDetectorResponse
from cv_bridge import CvBridge




def detect_edge(req):
    bridge = CvBridge()

    img = bridge.imgmsg_to_cv2(req.img)
    mask = cv.Canny(img, 200,200)
    mask = cv.cvtColor(mask, cv.COLOR_GRAY2BGR)
    mask[:,:,0] *= 0
    mask[:,:,2] *= 0
    edge_image = bridge.cv2_to_imgmsg(mask, encoding="passthrough")
    return edgeDetectorResponse(edge_image)


if __name__=="__main__":
        
    try:
        rospy.init_node("edge_detector_server")
        s = rospy.Service("canny_edge_detector", edgeDetector, detect_edge)
        print("ready")
        rospy.spin()
    except:
        rospy.logerr("ERROR")
