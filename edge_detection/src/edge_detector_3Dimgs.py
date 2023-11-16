#!/usr/bin/env python3

import cv2 as cv
import numpy as np
import ros_numpy
import os
from sensor_msgs.msg import Image, PointCloud2, ChannelFloat32, PointField
from edge_detection.srv import edgeDetector, edgeDetectorResponse
import sys
import rospy
from std_msgs.msg import UInt32, UInt8, ColorRGBA
from cv_bridge import CvBridge
import message_filters
from geometry_msgs.msg import Point32, Point, Pose, Quaternion, Vector3
from visualization_msgs.msg import Marker




def callback(rgb_img, depth_img):
    bridge = CvBridge()
    pub = rospy.Publisher('edge_points', PointCloud2, queue_size=100)
    marker_pub = rospy.Publisher("/edge_marker", Marker, queue_size=20)
    rospy.wait_for_service('canny_edge_detector')
    try:
        edge_img_detector = rospy.ServiceProxy('canny_edge_detector', edgeDetector)
        ros_edge_img = edge_img_detector(rgb_img)
        
        cv_edge_img = bridge.imgmsg_to_cv2(ros_edge_img.edgeImage)
        edge_mask = cv.cvtColor(cv_edge_img, cv.COLOR_BGR2GRAY)
        edge_pxl  = np.transpose(np.array(np.where(edge_mask>=100)))
        cv_depth_img = bridge.imgmsg_to_cv2(depth_img)
        depth_vals = np.array(cv_depth_img[edge_mask>=100])
        np_depth_array = np.concatenate(
            [edge_pxl, np.expand_dims(depth_vals,1)], axis=1)
        #print(cv_edge_img.shape)
        rgb_arr = np.ones(np_depth_array.shape[0],3)*255
        lst = []
        p32_lst = []
        rgba_lst = []

        for i in range(np_depth_array.shape[0]):
            lst.append((np_depth_array[i,0]/480,np_depth_array[i,1]/640,np_depth_array[i,2]/np.max(depth_vals)))
            p = Point32(np_depth_array[i,0]/480,np_depth_array[i,1]/640,np_depth_array[i,2]/np.max(depth_vals))
            p32_lst.append(p)
            rgba_lst.append((ColorRGBA(1.,1.,0.,0.9)))




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
        pc.row_step   = len(depth_vals)
        np_depth_array = np.reshape(np_depth_array, 
                                    (np_depth_array.shape[0]*np_depth_array.shape[1]))
        pc.data = np.array(np_depth_array, dtype=np.uint8).tobytes()
        
        
        pub.publish(pc)

        
        
        
        
        
       
        # np_depth_array = np.array(lst,
        #                           dtype=[('u', np.int32), ('v', np.int32), ('z', np.float32)])

        # depth_pc = ros_numpy.point_cloud2.array_to_pointcloud2(np.transpose(np_depth_array))
        # depth_pc.header.frame_id = "base_link"
        # depth_pc.header.seq = rgb_img.header.seq
        # depth_pc.header.stamp = rgb_img.header.stamp
        # #depth_pc.is_dense = False
        # #print(depth_pc.data[2], depth_pc.data[4],depth_pc.data[6],depth_pc.data[8],depth_pc.data[10],depth_pc.data[12])
        # print(depth_pc.fields)

        # pub.publish(depth_pc)

        ####
        marker = Marker()
        marker.header.frame_id = rgb_img.header.frame_id
        marker.header.stamp = rgb_img.header.stamp
        marker.type = 8 # 4 line strip #2 sphere
        marker.id = 0
        marker.points = p32_lst
        marker.colors = rgba_lst
        #marker.color = ColorRGBA(244,1,0,1)
        #marker.pose = Pose(Point(1,1,1), Quaternion(0,0,0,0))
        marker.scale = Vector3(0.1,0.1,0.1)
        marker.mesh_use_embedded_materials = False
        marker_pub.publish(marker)



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