#!/usr/bin/env python3
import numpy as np
import message_filters
import rospy

from geometry_msgs.msg import Point32, Point, Pose, Quaternion, Vector3
from sensor_msgs.msg import Image, PointCloud, ChannelFloat32
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from edge_detection.srv import edgeDetector
from cv_bridge import CvBridge

###
# Rosnode for converting edge images from rosbag
# into edge images and publish them with the depth
# values to the topic 'edge_points' amd 'edge_marker'
###

def callback(rgb_img, depth_img):
    custom_pc = True
    # rgb_image[sensor_msg/Image] and depth_image[sensor_msg/Image] obtained by subsciber
    bridge = CvBridge()
    # create publisher for point cloud and edge Marker
    pub = rospy.Publisher('edge_points', PointCloud, queue_size=100)
    marker_pub = rospy.Publisher("/edge_marker", Marker, queue_size=100)
    # wait for edge detection service 
    rospy.wait_for_service('canny_edge_detector')
    try:
        # obtain image with detected edges from serive
        edge_img_detector = rospy.ServiceProxy('canny_edge_detector', edgeDetector)
        ros_edge_img = edge_img_detector(rgb_img)
        # convert edge image into 
        cv_edge_img = bridge.imgmsg_to_cv2(ros_edge_img.edgeImage)
        # edge_mask = cv.cvtColor(cv_edge_img, cv.COLOR_BGR2GRAY)
        # get pixels where green value of edge img > 100 -> edges
        edge_pxl  = np.transpose(np.array(np.where(cv_edge_img[:,:,1]>=100)))
        # convert depth image into cv2 image
        cv_depth_img = bridge.imgmsg_to_cv2(depth_img, desired_encoding="8UC1")
        # get corresponding depth values of edges
        depth_edges = np.array(cv_depth_img[cv_edge_img[:,:,1]>=100])
        # create 3d array with dims = [row, col, depth]
        np_depth_array = np.concatenate(
            [edge_pxl, np.expand_dims(depth_edges,1)], axis=1)
        # define values for publishing pc and markers
        # converting the xyz numpy array into point32 list
        # create rgba list for pointer colors
        # norm x,y,z in 0 to 1
        p32_lst = np.empty(shape=np_depth_array.shape[0], dtype=Point32)
        rgba_lst = np.empty(shape=np_depth_array.shape[0], dtype=ColorRGBA)
        for i in range(np_depth_array.shape[0]):
            p32_lst[i] = Point32(np_depth_array[i,0]/(cv_edge_img.shape[0]),
                                 np_depth_array[i,1]/(cv_edge_img.shape[1]),
                                 np_depth_array[i,2]/(255))
            rgba_lst[i] = ColorRGBA(1.,0.,0.,0.75)
        # create pointcloud and publish it 
        pc1 = PointCloud()
        pc1.header = rgb_img.header
        pc1.header.frame_id = "root_link"
        pc1.points = p32_lst
        pc1.channels = [ChannelFloat32("x", np.float32(np_depth_array[:,0])),
                        ChannelFloat32("y", np.float32(np_depth_array[:,1])),
                        ChannelFloat32("2", np.float32(np_depth_array[:,2]))]
        pub.publish(pc1)
  
        # create Marker and publish it
        marker = Marker()
        marker.header.stamp = rgb_img.header.stamp
        marker.header.frame_id = "root_link"
        marker.type = 8 # 4 line strip #2 sphere
        marker.id = 0
        marker.points = p32_lst
        marker.colors = rgba_lst
        marker.pose = Pose(Point(0,0,0), Quaternion(0,0,0,1))
        marker.scale = Vector3(0.005,0.005,0.005)
        marker.mesh_use_embedded_materials = False
        marker_pub.publish(marker)

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def detect_depth_edges():
    # initialize subsciber notes for rbg image and depth image
    rospy.init_node('bag_edge_detector', anonymous=True)
    img_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
    deph_sub = message_filters.Subscriber("/camera/depth/image_rect_raw", Image)
    # use message filter to synchrinize callback
    ts = message_filters.TimeSynchronizer([img_sub, deph_sub], queue_size=100)
    ts.registerCallback(callback)
    rospy.spin()

if __name__ == "__main__":
    detect_depth_edges()