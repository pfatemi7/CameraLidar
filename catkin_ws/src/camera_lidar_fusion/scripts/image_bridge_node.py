#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageBridgeNode:
    def __init__(self):
        rospy.init_node('image_bridge_node', anonymous=True)
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Publishers
        self.debug_image_pub = rospy.Publisher('/fused/debug_image', Image, queue_size=1)
        
        # Subscribers
        self.left_image_sub = rospy.Subscriber('/zed/left/image_raw', Image, self.left_image_callback)
        
        rospy.loginfo("Image Bridge Node initialized")
    
    def left_image_callback(self, image_msg):
        """Callback for left camera image"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            
            # Create a simple debug image (you can add processing here)
            debug_image = cv_image.copy()
            
            # Add some text to show it's working
            cv2.putText(debug_image, "ZED Camera Feed", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(debug_image, "LiDAR Fusion System", (10, 70), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            
            # Convert back to ROS image
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            debug_msg.header = image_msg.header
            
            # Publish debug image
            self.debug_image_pub.publish(debug_msg)
            
        except Exception as e:
            rospy.logerr("Error processing image: %s", str(e))

if __name__ == '__main__':
    try:
        node = ImageBridgeNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
