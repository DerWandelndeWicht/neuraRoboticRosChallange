#!/usr/bin/env python3
import cv2 as cv
import rospy

from edge_detection.srv import edgeDetector, edgeDetectorResponse
from cv_bridge import CvBridge

###
# Service for edge detection on incoming sensor_msgs/Image
# The Service converts an incoming sensor Image, converts it to a cvImage,
# then uses the cv canny edge detector for finding the edges.
# The resulting edge Image is transfered back into an sensor Image and send back
###

def detect_edge(req):
    bridge = CvBridge()
    # convert incoming sensor img -> cv image
    img = bridge.imgmsg_to_cv2(req.img)
    # detect edges
    mask = cv.Canny(img, 200,200)
    # convert color and "make" a green edge image
    mask = cv.cvtColor(mask, cv.COLOR_GRAY2BGR)
    mask[:,:,0] *= 0
    mask[:,:,2] *= 0
    # convert back to sensor image
    edge_image = bridge.cv2_to_imgmsg(mask, encoding="passthrough")
    # send back edge image
    return edgeDetectorResponse(edge_image)

if __name__=="__main__":
    try:
        # create and init service node
        rospy.init_node("edge_detector_server")
        s = rospy.Service("canny_edge_detector", edgeDetector, detect_edge)
        print("Edge Detector Service Ready")
        rospy.spin()
    except:
        rospy.logerr("ERROR")
