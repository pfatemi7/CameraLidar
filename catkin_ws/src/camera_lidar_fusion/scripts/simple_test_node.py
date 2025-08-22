#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
import struct

class SimpleTestNode:
    def __init__(self):
        rospy.init_node('simple_test_node', anonymous=True)
        
        # Publishers
        self.test_pub = rospy.Publisher('/test/topic', String, queue_size=10)
        
        # Subscribers
        self.cloud_sub = rospy.Subscriber('/unilidar/cloud', PointCloud2, self.cloud_callback)
        
        # Timer
        self.timer = rospy.Timer(rospy.Duration(1.0), self.timer_callback)
        
        rospy.loginfo("Simple test node initialized")
    
    def cloud_callback(self, msg):
        """Callback for point cloud messages"""
        rospy.loginfo("Received point cloud with %d points", msg.width * msg.height)
    
    def timer_callback(self, event):
        """Timer callback to publish test message"""
        test_msg = String()
        test_msg.data = "Camera-LiDAR Fusion Test - " + str(rospy.Time.now())
        self.test_pub.publish(test_msg)
        rospy.loginfo("Published test message")

if __name__ == '__main__':
    try:
        node = SimpleTestNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
