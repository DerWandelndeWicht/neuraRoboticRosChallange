#!/usr/bin/env python3
import rospy

from edge_detection.srv import edgeDetector
from sensor_msgs.msg import Image

###
# Service for recieving sensor Images from a rosbag
# detect the edges with the service and publish them 
# edge_image_publisher
###

def callback(data):
    # init publisher for publish the edge images
    pub = rospy.Publisher('edge_image_publisher', Image, queue_size=100)
    # wait for edge detection service to be ready
    rospy.wait_for_service('canny_edge_detector')
    try:
        # init service proxy for service and detect edges on rosbag image
        edge_img_detector = rospy.ServiceProxy('canny_edge_detector', edgeDetector)
        ros_edge_img = edge_img_detector(data)
        # publish edge image on new topic
        pub.publish(ros_edge_img.edgeImage)
            
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def detect_rosbag_edges():
    # init subsciber node to subscribe to topic from rosbag
    rospy.init_node('bag_edge_detector', anonymous=True)
    rospy.Subscriber("/camera/color/image_raw", Image, callback)
    rospy.spin()

if __name__ == "__main__":
    detect_rosbag_edges()