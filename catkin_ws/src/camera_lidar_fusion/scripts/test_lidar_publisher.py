#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2, PointField
import struct
import numpy as np

class TestLidarPublisher:
    def __init__(self):
        rospy.init_node('test_lidar_publisher', anonymous=True)
        
        # Publisher
        self.cloud_pub = rospy.Publisher('/unilidar/cloud', PointCloud2, queue_size=1)
        
        # Timer for publishing
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_cloud)  # 10 Hz
        
        rospy.loginfo("Test LiDAR publisher initialized")
    
    def create_test_point_cloud(self):
        """Create a simple test point cloud"""
        # Create a simple point cloud with some test points
        num_points = 2000
        
        # Generate random points in a cube
        x = np.random.uniform(-5, 5, num_points)
        y = np.random.uniform(-5, 5, num_points)
        z = np.random.uniform(0, 3, num_points)
        
        # Create point cloud message
        cloud_msg = PointCloud2()
        cloud_msg.header.stamp = rospy.Time.now()
        cloud_msg.header.frame_id = "unilidar_link"
        cloud_msg.height = 1
        cloud_msg.width = num_points
        
        # Define point fields
        cloud_msg.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('intensity', 12, PointField.FLOAT32, 1)
        ]
        cloud_msg.point_step = 16
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.is_dense = True
        
        # Pack points into binary data
        cloud_msg.data = b''
        for i in range(num_points):
            cloud_msg.data += struct.pack('ffff', x[i], y[i], z[i], 1.0)
        
        return cloud_msg
    
    def publish_cloud(self, event):
        """Publish test point cloud"""
        try:
            cloud_msg = self.create_test_point_cloud()
            self.cloud_pub.publish(cloud_msg)
            rospy.loginfo("Published test point cloud with %d points", cloud_msg.width)
        except Exception as e:
            rospy.logerr("Error publishing test cloud: %s", str(e))

if __name__ == '__main__':
    try:
        publisher = TestLidarPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
