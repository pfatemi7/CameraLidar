#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
import struct

class LidarOnlyNode:
    def __init__(self):
        rospy.init_node('lidar_only_node', anonymous=True)
        
        # Publishers
        self.filtered_cloud_pub = rospy.Publisher('/fused/cloud', PointCloud2, queue_size=1)
        self.status_pub = rospy.Publisher('/lidar/status', String, queue_size=1)
        
        # Subscribers
        self.cloud_sub = rospy.Subscriber('/unilidar/cloud', PointCloud2, self.cloud_callback)
        
        # Processing parameters
        self.z_min = rospy.get_param('~z_min', -0.2)
        self.z_max = rospy.get_param('~z_max', 2.5)
        
        rospy.loginfo("LiDAR-only processing node initialized")
        rospy.loginfo("Processing LiDAR data with Z range: %.2f to %.2f", self.z_min, self.z_max)
    
    def filter_point_cloud(self, cloud_msg):
        """Simple filtering of point cloud"""
        # For now, just republish the cloud with filtering info
        # In a real implementation, you would filter the points here
        
        # Create a simple filtered message (just copying for now)
        filtered_msg = PointCloud2()
        filtered_msg.header = cloud_msg.header
        filtered_msg.height = cloud_msg.height
        filtered_msg.width = cloud_msg.width
        filtered_msg.fields = cloud_msg.fields
        filtered_msg.is_bigendian = cloud_msg.is_bigendian
        filtered_msg.point_step = cloud_msg.point_step
        filtered_msg.row_step = cloud_msg.row_step
        filtered_msg.data = cloud_msg.data
        filtered_msg.is_dense = cloud_msg.is_dense
        
        return filtered_msg
    
    def cloud_callback(self, cloud_msg):
        """Callback for point cloud messages"""
        try:
            # Process the point cloud
            filtered_cloud = self.filter_point_cloud(cloud_msg)
            
            # Publish filtered cloud
            self.filtered_cloud_pub.publish(filtered_cloud)
            
            # Publish status
            status_msg = String()
            status_msg.data = "LiDAR processing: %d points, Z range: %.2f to %.2f" % (
                cloud_msg.width * cloud_msg.height, self.z_min, self.z_max
            )
            self.status_pub.publish(status_msg)
            
            rospy.loginfo("Processed LiDAR cloud with %d points", cloud_msg.width * cloud_msg.height)
            
        except Exception as e:
            rospy.logerr("Error processing point cloud: %s", str(e))

if __name__ == '__main__':
    try:
        node = LidarOnlyNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
