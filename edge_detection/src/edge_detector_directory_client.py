#!/usr/bin/env python3
import cv2 as cv
import rospy
import os

from edge_detection.srv import edgeDetector, edgeDetectorResponse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

###
# Service Client for reading Image from a directory
# detect the edges with the service client
# save Images into subdir </data/edge_imgs>
###

def get_dir_imgs():
    bridge = CvBridge()
    # init source directory and desination directory
    dir = "./src/neuraRoboticRosChallange/edge_detection/data"
    dst_dir = os.path.join(dir, "edge_imgs")
    os.makedirs(dst_dir, exist_ok=True)
    print(f"Detect edges for Images in: {dir}")
    
    # iterate through every file in directory
    for i in range(0,len(os.listdir(dir))):
        file = os.listdir(dir)[i]
        # skip if its not an image
        if not(file.endswith("png") or file.endswith("jpg")):
            continue
        # laod and convert to sensor image
        img = cv.imread(os.path.join(dir, file))
        ros_img = bridge.cv2_to_imgmsg(img, encoding="passthrough")
        # wait for edge detctor to be ready
        rospy.wait_for_service('canny_edge_detector')
        try:
            # init service proxy and send image to edge detector
            edge_img_detector = rospy.ServiceProxy('canny_edge_detector', edgeDetector)
            ros_edge_img = edge_img_detector(ros_img)
            # convert image back and save it into subdir
            edge_img = bridge.imgmsg_to_cv2(ros_edge_img.edgeImage)
            cv.imwrite(os.path.join(dst_dir, file), edge_img)
            print(f"Image saved at: {os.path.join(dst_dir, file)}")
   
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

if __name__ == "__main__":
    get_dir_imgs()
