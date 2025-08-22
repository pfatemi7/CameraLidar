#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraTestNode:
    def __init__(self):
        rospy.init_node('camera_test_node', anonymous=True)
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Publishers
        self.debug_image_pub = rospy.Publisher('/fused/debug_image', Image, queue_size=1)
        
        # Subscribers
        self.image_sub = rospy.Subscriber('/zed/left/image_raw', Image, self.image_callback)
        
        # Frame counter
        self.frame_count = 0
        
        rospy.loginfo("Camera Test Node initialized")
    
    def image_callback(self, image_msg):
        """Callback for camera images"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            
            # Increment frame counter
            self.frame_count += 1
            
            # Create debug image with frame counter
            debug_image = cv_image.copy()
            
            # Add frame counter and timestamp
            cv2.putText(debug_image, "Camera Test - Frame: %d" % self.frame_count, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(debug_image, "Camera Working!", (10, 70), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            
            # Add a moving indicator to show it's updating
            import time
            current_time = int(time.time() * 10) % 100
            cv2.putText(debug_image, "Time: %02d" % current_time, (10, 110), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # Convert back to ROS image
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            debug_msg.header = image_msg.header
            
            # Publish debug image
            self.debug_image_pub.publish(debug_msg)
            
            # Log every 30 frames
            if self.frame_count % 30 == 0:
                rospy.loginfo("Camera test: Processed %d frames", self.frame_count)
            
        except Exception as e:
            rospy.logerr("Error processing image: %s", str(e))

if __name__ == '__main__':
    try:
        node = CameraTestNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
