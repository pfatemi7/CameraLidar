#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import threading
import time

class ZEDCameraNode:
    def __init__(self):
        rospy.init_node('zed_camera_node', anonymous=True)
        
        # Publishers
        self.left_image_pub = rospy.Publisher('/zed/left/image_raw', Image, queue_size=1)
        self.right_image_pub = rospy.Publisher('/zed/right/image_raw', Image, queue_size=1)
        self.left_camera_info_pub = rospy.Publisher('/zed/left/camera_info', CameraInfo, queue_size=1)
        self.right_camera_info_pub = rospy.Publisher('/zed/right/camera_info', CameraInfo, queue_size=1)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Camera parameters (typical ZED camera parameters)
        self.camera_info = CameraInfo()
        self.camera_info.header.frame_id = "zed_camera_optical_frame"
        self.camera_info.width = 1280
        self.camera_info.height = 720
        self.camera_info.distortion_model = "plumb_bob"
        
        # ZED camera intrinsic parameters (approximate)
        self.camera_info.K = [1000.0, 0.0, 640.0, 0.0, 1000.0, 360.0, 0.0, 0.0, 1.0]
        self.camera_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.camera_info.P = [1000.0, 0.0, 640.0, 0.0, 0.0, 1000.0, 360.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.camera_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        
        # Try to open ZED camera
        self.cap = None
        self.try_open_camera()
        
        if self.cap is None or not self.cap.isOpened():
            rospy.logwarn("Could not open ZED camera. Trying USB camera...")
            self.cap = cv2.VideoCapture(0)  # Try USB camera as fallback
        
        if self.cap is None or not self.cap.isOpened():
            rospy.logerr("Could not open any camera!")
            return
        
        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        rospy.loginfo("ZED camera node initialized")
        
        # Start publishing
        self.publish_thread = threading.Thread(target=self.publish_loop)
        self.publish_thread.daemon = True
        self.publish_thread.start()
    
    def try_open_camera(self):
        """Try to open ZED camera on different devices"""
        for device_id in range(4):  # Try devices 0-3
            try:
                cap = cv2.VideoCapture(device_id)
                if cap.isOpened():
                    ret, frame = cap.read()
                    if ret and frame is not None:
                        rospy.loginfo("Found camera on device %d", device_id)
                        self.cap = cap
                        return
                    cap.release()
            except Exception as e:
                rospy.logwarn("Error trying device %d: %s", device_id, str(e))
    
    def publish_loop(self):
        """Main publishing loop"""
        rate = rospy.Rate(30)  # 30 Hz
        
        while not rospy.is_shutdown():
            try:
                if self.cap is None or not self.cap.isOpened():
                    rospy.logwarn("Camera not available")
                    time.sleep(1)
                    continue
                
                ret, frame = self.cap.read()
                if not ret or frame is None:
                    rospy.logwarn("Failed to read frame")
                    continue
                
                # Split frame into left and right (if stereo)
                height, width = frame.shape[:2]
                if width > height:  # Likely stereo
                    left_frame = frame[:, :width//2]
                    right_frame = frame[:, width//2:]
                else:  # Single camera
                    left_frame = frame
                    right_frame = frame.copy()
                
                # Create timestamps
                timestamp = rospy.Time.now()
                
                # Publish left image
                left_msg = self.bridge.cv2_to_imgmsg(left_frame, "bgr8")
                left_msg.header.stamp = timestamp
                left_msg.header.frame_id = "zed_camera_optical_frame"
                self.left_image_pub.publish(left_msg)
                
                # Publish right image
                right_msg = self.bridge.cv2_to_imgmsg(right_frame, "bgr8")
                right_msg.header.stamp = timestamp
                right_msg.header.frame_id = "zed_camera_optical_frame"
                self.right_image_pub.publish(right_msg)
                
                # Publish camera info
                self.camera_info.header.stamp = timestamp
                self.left_camera_info_pub.publish(self.camera_info)
                self.right_camera_info_pub.publish(self.camera_info)
                
                rospy.loginfo("Published stereo images: %dx%d", left_frame.shape[1], left_frame.shape[0])
                
            except Exception as e:
                rospy.logerr("Error in publish loop: %s", str(e))
            
            rate.sleep()
    
    def __del__(self):
        if self.cap is not None:
            self.cap.release()

if __name__ == '__main__':
    try:
        node = ZEDCameraNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
