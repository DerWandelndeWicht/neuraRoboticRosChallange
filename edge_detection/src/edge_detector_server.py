#!/usr/bin/env python3

import rospy
import cv2 as cv
import numpy as np
import os
from sensor_msgs.msg import Image

from edge_detection.srv import edgeDetector, edgeDetectorResponse
from data_converter import dataConverter

def detect_edge(req):
    dc = dataConverter
    #img_path = "/home/ros/Downloads/check.jpg"
    #img = cv.imread(os.path.join(req.srcDir, req.fname))
    data = req.data
    img = dc.rosImage2img(data)
    #img = np.reshape(data, (req.width, req.height, 3))
    canny_mask = cv.Canny(img, 100,100)
    
    pos_pix = canny_mask[canny_mask > 100]
    edge_image = np.zeros((req.width, req.height,3))
    edge_image[:,:,1] = 255*canny_mask
    
    edge_image = list(np.array(edge_image.flatten(),dtype=np.uin8))
    return edgeDetectorResponse(edge_image)


if __name__=="__main__":
        
    try:
        rospy.init_node("edge_detector_server")
        s = rospy.Service("canny_edge_detector", edgeDetector, detect_edge)
        print("ready")
        rospy.spin()
    except:
        rospy.logerr("ERROR")
