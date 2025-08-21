#!/usr/bin/env python

import numpy as np
import pcl
from sensor_msgs.msg import PointCloud2
import struct

def ros_to_pcl(ros_cloud):
    """Convert ROS PointCloud2 to PCL PointCloud"""
    points_list = []
    
    for data in ros_cloud.data:
        points_list.append([struct.unpack('f', data[0:4])[0],
                           struct.unpack('f', data[4:8])[0],
                           struct.unpack('f', data[8:12])[0]])
    
    pcl_cloud = pcl.PointCloud()
    pcl_cloud.from_list(points_list)
    return pcl_cloud

def pcl_to_ros(pcl_cloud):
    """Convert PCL PointCloud to ROS PointCloud2"""
    ros_cloud = PointCloud2()
    ros_cloud.header.stamp = rospy.Time.now()
    ros_cloud.header.frame_id = "map"
    
    ros_cloud.height = 1
    ros_cloud.width = len(pcl_cloud)
    
    ros_cloud.fields.append(PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1))
    ros_cloud.fields.append(PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1))
    ros_cloud.fields.append(PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1))
    
    ros_cloud.point_step = 12
    ros_cloud.row_step = ros_cloud.point_step * ros_cloud.width
    
    for point in pcl_cloud:
        ros_cloud.data += struct.pack('fff', point[0], point[1], point[2])
    
    return ros_cloud

def array_to_pointcloud2(points_array, header):
    """Convert numpy array to ROS PointCloud2"""
    from sensor_msgs.msg import PointField
    
    ros_cloud = PointCloud2()
    ros_cloud.header = header
    
    ros_cloud.height = 1
    ros_cloud.width = len(points_array)
    
    if points_array.shape[1] == 3:  # XYZ only
        ros_cloud.fields.append(PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1))
        ros_cloud.fields.append(PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1))
        ros_cloud.fields.append(PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1))
        ros_cloud.point_step = 12
    elif points_array.shape[1] == 6:  # XYZRGB
        ros_cloud.fields.append(PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1))
        ros_cloud.fields.append(PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1))
        ros_cloud.fields.append(PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1))
        ros_cloud.fields.append(PointField(name="r", offset=12, datatype=PointField.FLOAT32, count=1))
        ros_cloud.fields.append(PointField(name="g", offset=16, datatype=PointField.FLOAT32, count=1))
        ros_cloud.fields.append(PointField(name="b", offset=20, datatype=PointField.FLOAT32, count=1))
        ros_cloud.point_step = 24
    
    ros_cloud.row_step = ros_cloud.point_step * ros_cloud.width
    
    for point in points_array:
        if len(point) == 3:
            ros_cloud.data += struct.pack('fff', point[0], point[1], point[2])
        elif len(point) == 6:
            ros_cloud.data += struct.pack('ffffff', point[0], point[1], point[2], 
                                        point[3], point[4], point[5])
    
    return ros_cloud
