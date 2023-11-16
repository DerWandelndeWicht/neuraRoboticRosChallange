#!/usr/bin/env python3
import cv2 as cv
import numpy as np
import ros_numpy
import os
import message_filters
import sys
import rospy

from std_msgs.msg import UInt32, UInt8, ColorRGBA
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2, ChannelFloat32, PointField
from edge_detection.srv import edgeDetector, edgeDetectorResponse
from geometry_msgs.msg import Point32, Point, Pose, Quaternion, Vector3
from visualization_msgs.msg import Marker, MarkerArray

def callback(rgb_img, depth_img):
    custom_pc = False
    # rgb_image[sensor_msg/Image] and depth_image[sensor_msg/Image] obtained by subsciber
    bridge = CvBridge()
    # create publisher for point cloud and edge Marker
    pub = rospy.Publisher('edge_points', PointCloud2, queue_size=100)
    marker_pub = rospy.Publisher("/edge_marker", Marker, queue_size=100)
    # wait for edge detection service 
    rospy.wait_for_service('canny_edge_detector')
    try:
        # obtain image with detected edges from serive
        edge_img_detector = rospy.ServiceProxy('canny_edge_detector', edgeDetector)
        ros_edge_img = edge_img_detector(rgb_img)
        # convert edge image into 
        # print(depth_img.encoding)
        cv_edge_img = bridge.imgmsg_to_cv2(ros_edge_img.edgeImage)
        # edge_mask = cv.cvtColor(cv_edge_img, cv.COLOR_BGR2GRAY)
        # get pixels where green value of edge img > 100 -> edges
        edge_pxl  = np.transpose(np.array(np.where(cv_edge_img[:,:,1]>=100)))
        # convert depth image into cv2 image
        cv_depth_img = bridge.imgmsg_to_cv2(depth_img, desired_encoding="8UC1")
        # print(cv_depth_img, np.max(cv_depth_img))
        # get corresponding depth values of edges
        depth_edges = np.array(cv_depth_img[cv_edge_img[:,:,1]>=100])
        # create 3d array with dims = [row, col, depth]
        np_depth_array = np.concatenate(
            [edge_pxl, np.expand_dims(depth_edges,1)], axis=1)

        # cv.imshow("", cv_edge_img)
        # cv.waitKey(0)
        # define values for publishing pc and markers
        rgb_arr = np.ones((np_depth_array.shape[0],3))*255
        lst = np.empty(shape=np_depth_array.shape[0], dtype=tuple)
        p32_lst = np.empty(shape=np_depth_array.shape[0], dtype=Point32)
        rgba_lst = np.empty(shape=np_depth_array.shape[0], dtype=ColorRGBA)
        #markerArray = np.empty(shape=np_depth_array.shape[0], dtype=Marker)
        #print(cv_edge_img.shape[0]*cv_edge_img.shape[1], np_depth_array.shape)
        for i in range(np_depth_array.shape[0]):
            lst[i]     = (np_depth_array[i,0],np_depth_array[i,1],np_depth_array[i,2])
            p32_lst[i] = Point32(np_depth_array[i,0]/cv_edge_img.shape[0],
                                 np_depth_array[i,1]/cv_edge_img.shape[1],
                                 np_depth_array[i,2]/255.)
            rgba_lst[i] = ColorRGBA(1.,0.,0.,0.75)

        # pc = get_pointCloud2(header=rgb_img.header,
        #                      img_shape=cv_edge_img.shape[:2],
        #                      data = np.reshape(np_depth_array, 
        #                             (np_depth_array.shape[0]*np_depth_array.shape[1])),
        #                      custom_pc = True)
        ############################################
        if custom_pc:
            pc = PointCloud2()
            pc.header = rgb_img.header
            pc.header.frame_id = "base_link"
            pc.height, pc.width = cv_edge_img.shape[:2]
            pc.fields = [PointField("u",0,7,1),
                        PointField("v",4,7,1),
                        PointField("z",8,7,1),
                        PointField("rgb",16,7,1)]
            pc.is_bigendian = False
            pc.point_step = 12
            pc.row_step   = len(depth_edges)
            np_depth_array = np.reshape(np_depth_array, 
                                        (np_depth_array.shape[0]*np_depth_array.shape[1]))
            pc.data = np.array(np_depth_array, dtype=np.uint8).tobytes()
            pub.publish(pc)
        else:
            np_depth_array = np.array(lst,
                                    dtype=[('u', np.int32), ('v', np.int32), ('z', np.float32)])
            depth_pc = ros_numpy.point_cloud2.array_to_pointcloud2(np.transpose(np_depth_array))
            depth_pc.header.frame_id = "base_link"
            depth_pc.header.seq = rgb_img.header.seq
            depth_pc.header.stamp = rgb_img.header.stamp
            #depth_pc.is_dense = False
            #print(depth_pc.data[2], depth_pc.data[4],depth_pc.data[6],depth_pc.data[8],depth_pc.data[10],depth_pc.data[12])
            #print(depth_pc.fields)
            pub.publish(depth_pc)
        #####################################
    
        ####
        marker = Marker()
        #marker.header.frame_id = rgb_img.header.frame_id
        marker.header.stamp = rgb_img.header.stamp
        marker.header.frame_id = "root_link"
        marker.type = 8 # 4 line strip #2 sphere
        marker.id = 0
        marker.points = p32_lst
        marker.colors = rgba_lst
        #marker.color = ColorRGBA(244,1,0,1)
        marker.pose = Pose(Point(1,1,1), Quaternion(0,0,0,1))
        marker.scale = Vector3(0.01,0.01,0.01)
        marker.mesh_use_embedded_materials = False
        marker_pub.publish(marker)



    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def get_pointCloud2():
    pass


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