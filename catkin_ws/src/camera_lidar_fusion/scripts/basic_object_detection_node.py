#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
import math

class BasicObjectDetectionNode:
    def __init__(self):
        rospy.init_node('basic_object_detection_node', anonymous=True)
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Publishers
        self.debug_image_pub = rospy.Publisher('/fused/debug_image', Image, queue_size=1)
        self.detection_pub = rospy.Publisher('/object_detection/result', String, queue_size=1)
        self.distance_pub = rospy.Publisher('/object_detection/distance', Float32, queue_size=1)
        
        # Subscribers
        self.image_sub = rospy.Subscriber('/zed/left/image_raw', Image, self.image_callback)
        
        # Object detection parameters
        self.min_area = rospy.get_param('~min_area', 500)   # Minimum object area
        self.max_area = rospy.get_param('~max_area', 100000) # Maximum object area
        
        # Distance estimation parameters (based on object size)
        self.reference_distance = rospy.get_param('~reference_distance', 1.0)  # meters
        self.reference_area = rospy.get_param('~reference_area', 10000)  # pixels
        
        # Frame counter
        self.frame_count = 0
        
        rospy.loginfo("Basic Object Detection Node initialized")
    
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
        
        # Find contours (OpenCV 3.2.0 compatibility)
        contours = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]
        
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
    
    def estimate_distance_from_size(self, object_area):
        """Estimate distance based on object size (inverse square law)"""
        if object_area <= 0:
            return None
        
        # Using inverse square law: distance = reference_distance * sqrt(reference_area / object_area)
        distance = self.reference_distance * math.sqrt(float(self.reference_area) / float(object_area))
        
        # Clamp distance to reasonable range
        distance = max(0.1, min(10.0, distance))
        
        return distance
    
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
                
                # Estimate distance based on object size
                distance = self.estimate_distance_from_size(obj['area'])
                
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
            cv2.putText(debug_image, "Basic Object Detection", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(debug_image, "Objects: %d" % len(detected_objects), (10, 70), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(debug_image, "Frame: %d" % self.frame_count, (10, 110), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Convert back to ROS image
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            debug_msg.header = image_msg.header
            
            # Publish debug image
            self.debug_image_pub.publish(debug_msg)
            
            # Log every 30 frames
            if self.frame_count % 30 == 0:
                rospy.loginfo("Basic Object Detection: Processed %d frames, detected %d objects", 
                             self.frame_count, len(detected_objects))
            
        except Exception as e:
            rospy.logerr("Error processing image: %s", str(e))
            import traceback
            traceback.print_exc()

if __name__ == '__main__':
    try:
        node = BasicObjectDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
