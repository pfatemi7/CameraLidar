#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
import struct

class ObjectDetectionNode:
    def __init__(self):
        rospy.init_node('object_detection_node', anonymous=True)
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Publishers
        self.debug_image_pub = rospy.Publisher('/fused/debug_image', Image, queue_size=1)
        self.detection_pub = rospy.Publisher('/object_detection/result', String, queue_size=1)
        self.distance_pub = rospy.Publisher('/object_detection/distance', Float32, queue_size=1)
        
        # Subscribers
        self.image_sub = rospy.Subscriber('/zed/left/image_raw', Image, self.image_callback)
        self.cloud_sub = rospy.Subscriber('/fused/cloud', PointCloud2, self.cloud_callback)
        
        # Object detection parameters
        self.min_area = rospy.get_param('~min_area', 500)   # Minimum object area (smaller for better detection)
        self.max_area = rospy.get_param('~max_area', 100000) # Maximum object area
        self.distance_threshold = rospy.get_param('~distance_threshold', 5.0)  # meters
        
        # Store latest point cloud for distance calculation
        self.latest_cloud = None
        self.latest_cloud_time = None
        
        # Object detection history
        self.detected_objects = []
        self.frame_count = 0
        
        rospy.loginfo("Object Detection Node initialized")
    
    def detect_objects_camera(self, image):
        """Detect objects using camera (simple motion detection)"""
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, (21, 21), 0)
        
        # If this is the first frame, initialize background
        if not hasattr(self, 'background'):
            self.background = blurred.copy().astype("float")
            return []
        
        # Update background model (slower update for more stable detection)
        cv2.accumulateWeighted(blurred, self.background, 0.3)
        
        # Compute difference between current frame and background
        frame_delta = cv2.absdiff(blurred, self.background.astype("uint8"))
        
        # Threshold the delta image (lower threshold for better sensitivity)
        thresh = cv2.threshold(frame_delta, 15, 255, cv2.THRESH_BINARY)[1]
        
        # Dilate the thresholded image to fill in holes
        thresh = cv2.dilate(thresh, None, iterations=2)
        
        # Find contours
        contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        objects = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if self.min_area < area < self.max_area:
                # Get bounding box
                (x, y, w, h) = cv2.boundingRect(contour)
                objects.append({
                    'type': 'person',
                    'bbox': (x, y, w, h),
                    'area': area,
                    'center': (x + w//2, y + h//2)
                })
        
        return objects
    
    def calculate_distance_lidar(self, object_center, cloud_msg):
        """Calculate distance to object using LiDAR data"""
        if cloud_msg is None:
            return None
        
        try:
            # Extract point cloud data
            points = []
            point_step = cloud_msg.point_step
            point_fields = cloud_msg.fields
            
            # Find the offset for x, y, z fields
            x_offset = None
            y_offset = None
            z_offset = None
            
            for field in point_fields:
                if field.name == 'x':
                    x_offset = field.offset
                elif field.name == 'y':
                    y_offset = field.offset
                elif field.name == 'z':
                    z_offset = field.offset
            
            if x_offset is None or y_offset is None or z_offset is None:
                rospy.logwarn("Point cloud missing x, y, or z fields")
                return None
            
            # Extract points
            for i in range(0, len(cloud_msg.data), point_step):
                if i + point_step <= len(cloud_msg.data):
                    point_data = cloud_msg.data[i:i+point_step]
                    
                    # Extract x, y, z values using the correct offsets
                    x = struct.unpack('f', point_data[x_offset:x_offset+4])[0]
                    y = struct.unpack('f', point_data[y_offset:y_offset+4])[0]
                    z = struct.unpack('f', point_data[z_offset:z_offset+4])[0]
                    
                    points.append((x, y, z))
            
            if not points:
                return None
            
            # Find closest point to camera center (assuming camera is at origin)
            # For simplicity, we'll use the closest point in the general direction
            distances = []
            for x, y, z in points:
                # Filter points in front of the camera (positive Z)
                if z > 0:
                    # Calculate 2D distance in X-Y plane
                    dist_2d = np.sqrt(x*x + y*y)
                    distances.append(dist_2d)
            
            if distances:
                # Return the median distance (more robust than min)
                return np.median(distances)
            
        except Exception as e:
            rospy.logwarn("Error calculating distance: %s", str(e))
        
        return None
    
    def image_callback(self, image_msg):
        """Callback for camera images"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            
            # Increment frame counter
            self.frame_count += 1
            
            # Detect objects
            detected_objects = self.detect_objects_camera(cv_image)
            
            # Create debug image
            debug_image = cv_image.copy()
            
            # Draw detected objects
            for obj in detected_objects:
                x, y, w, h = obj['bbox']
                center_x, center_y = obj['center']
                
                # Draw bounding box
                cv2.rectangle(debug_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                
                # Calculate distance using LiDAR
                distance = self.calculate_distance_lidar(obj['center'], self.latest_cloud)
                
                # Draw object info
                if distance is not None:
                    label = "Person: %.2fm" % distance
                    cv2.putText(debug_image, label, (x, y - 10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    
                    # Publish distance
                    distance_msg = Float32()
                    distance_msg.data = distance
                    self.distance_pub.publish(distance_msg)
                    
                    # Publish detection result
                    detection_msg = String()
                    detection_msg.data = "Detected person at %.2f meters" % distance
                    self.detection_pub.publish(detection_msg)
                else:
                    cv2.putText(debug_image, "Person: Distance unknown", (x, y - 10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # Add system info
            cv2.putText(debug_image, "Object Detection Active", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(debug_image, "Objects: %d" % len(detected_objects), (10, 70), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(debug_image, "Frame: %d" % getattr(self, 'frame_count', 0), (10, 110), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Convert back to ROS image
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            debug_msg.header = image_msg.header
            
            # Publish debug image
            self.debug_image_pub.publish(debug_msg)
            
        except Exception as e:
            rospy.logerr("Error processing image: %s", str(e))
    
    def cloud_callback(self, cloud_msg):
        """Callback for point cloud messages"""
        # Store latest cloud for distance calculation
        self.latest_cloud = cloud_msg
        self.latest_cloud_time = rospy.Time.now()

if __name__ == '__main__':
    try:
        node = ObjectDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
