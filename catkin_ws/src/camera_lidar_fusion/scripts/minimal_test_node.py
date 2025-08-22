#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

class MinimalTestNode:
    def __init__(self):
        rospy.init_node('minimal_test_node', anonymous=True)
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Publishers
        self.debug_image_pub = rospy.Publisher('/fused/debug_image', Image, queue_size=1)
        
        # Subscribers
        self.image_sub = rospy.Subscriber('/zed/left/image_raw', Image, self.image_callback)
        
        # Frame counter
        self.frame_count = 0
        
        rospy.loginfo("Minimal Test Node initialized")
    
    def image_callback(self, image_msg):
        """Callback for camera images"""
        try:
            rospy.loginfo("Received image: %dx%d", image_msg.width, image_msg.height)
            
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            
            # Increment frame counter
            self.frame_count += 1
            
            # Just republish the image with a simple overlay
            import cv2
            
            # Add simple text
            cv2.putText(cv_image, "MINIMAL TEST", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(cv_image, "Frame: %d" % self.frame_count, (10, 70), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Convert back to ROS image
            debug_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            debug_msg.header = image_msg.header
            
            # Publish debug image
            self.debug_image_pub.publish(debug_msg)
            
            rospy.loginfo("Published frame %d", self.frame_count)
            
        except Exception as e:
            rospy.logerr("Error in minimal test: %s", str(e))
            import traceback
            traceback.print_exc()

if __name__ == '__main__':
    try:
        node = MinimalTestNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
