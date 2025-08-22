#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import math
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
import struct

class LidarCameraFusionNode:
    def __init__(self):
        rospy.init_node('lidar_camera_fusion_node', anonymous=True)
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Publishers
        self.debug_image_pub = rospy.Publisher('/fused/debug_image', Image, queue_size=1)
        self.detection_pub = rospy.Publisher('/object_detection/result', String, queue_size=1)
        self.distance_pub = rospy.Publisher('/object_detection/distance', Float32, queue_size=1)
        
        # Subscribers
        self.image_sub = rospy.Subscriber('/zed/left/image_raw', Image, self.image_callback)
        self.cloud_sub = rospy.Subscriber('/fused/cloud', PointCloud2, self.cloud_callback)
        
        # Camera-LiDAR calibration parameters (you need to calibrate these!)
        # These are example values - you need to calibrate for your setup
        self.lidar_to_camera_x = rospy.get_param('~lidar_to_camera_x', 0.0)  # meters
        self.lidar_to_camera_y = rospy.get_param('~lidar_to_camera_y', 0.0)  # meters  
        self.lidar_to_camera_z = rospy.get_param('~lidar_to_camera_z', 0.1)  # meters (LiDAR above camera)
        
        # Camera parameters (you need to calibrate these!)
        self.camera_fov_horizontal = rospy.get_param('~camera_fov_horizontal', 90.0)  # degrees
        self.camera_fov_vertical = rospy.get_param('~camera_fov_vertical', 60.0)  # degrees
        self.image_width = rospy.get_param('~image_width', 640)  # pixels
        self.image_height = rospy.get_param('~image_height', 480)  # pixels
        
        # Object detection parameters
        self.min_area = rospy.get_param('~min_area', 500)
        self.max_area = rospy.get_param('~max_area', 100000)
        
        # Store latest data
        self.latest_cloud = None
        self.latest_image = None
        self.frame_count = 0
        
        rospy.loginfo("LiDAR-Camera Fusion Node initialized")
        rospy.loginfo("Camera FOV: %.1f° x %.1f°", self.camera_fov_horizontal, self.camera_fov_vertical)
        rospy.loginfo("LiDAR offset: (%.3f, %.3f, %.3f)", 
                     self.lidar_to_camera_x, self.lidar_to_camera_y, self.lidar_to_camera_z)
    
    def detect_objects_camera(self, image):
        """Detect objects using camera (motion detection)"""
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, (21, 21), 0)
        
        # If this is the first frame, initialize background
        if not hasattr(self, 'background'):
            self.background = blurred.copy().astype("float")
            return []
        
        # Update background model
        cv2.accumulateWeighted(blurred, self.background, 0.3)
        
        # Compute difference between current frame and background
        frame_delta = cv2.absdiff(blurred, self.background.astype("uint8"))
        
        # Threshold the delta image
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
    
    def image_to_lidar_coordinates(self, image_x, image_y):
        """Convert image coordinates to LiDAR coordinates"""
        # Convert image coordinates to camera angles
        # Assuming camera center is at image center
        center_x = self.image_width / 2.0
        center_y = self.image_height / 2.0
        
        # Calculate angles from center
        angle_x = (image_x - center_x) * (self.camera_fov_horizontal / self.image_width)
        angle_y = (image_y - center_y) * (self.camera_fov_vertical / self.image_height)
        
        # Convert to radians
        angle_x_rad = math.radians(angle_x)
        angle_y_rad = math.radians(angle_y)
        
        return angle_x_rad, angle_y_rad
    
    def find_lidar_distance_in_direction(self, angle_x, angle_y):
        """Find LiDAR distance in the specified direction"""
        if self.latest_cloud is None:
            return None
        
        try:
            # Extract point cloud data
            points = []
            point_step = self.latest_cloud.point_step
            point_fields = self.latest_cloud.fields
            
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
                return None
            
            # Extract points
            for i in range(0, len(self.latest_cloud.data), point_step):
                if i + point_step <= len(self.latest_cloud.data):
                    point_data = self.latest_cloud.data[i:i+point_step]
                    
                    # Extract x, y, z values
                    x = struct.unpack('f', point_data[x_offset:x_offset+4])[0]
                    y = struct.unpack('f', point_data[y_offset:y_offset+4])[0]
                    z = struct.unpack('f', point_data[z_offset:z_offset+4])[0]
                    
                    # Transform LiDAR coordinates to camera coordinates
                    # Apply LiDAR to camera offset
                    x_cam = x - self.lidar_to_camera_x
                    y_cam = y - self.lidar_to_camera_y
                    z_cam = z - self.lidar_to_camera_z
                    
                    # Calculate angle from camera to this point
                    if z_cam > 0:  # Only consider points in front of camera
                        point_angle_x = math.atan2(x_cam, z_cam)
                        point_angle_y = math.atan2(y_cam, z_cam)
                        
                        # Check if this point is in the direction we're looking
                        angle_tolerance = math.radians(5.0)  # 5 degree tolerance
                        if (abs(point_angle_x - angle_x) < angle_tolerance and 
                            abs(point_angle_y - angle_y) < angle_tolerance):
                            
                            # Calculate distance
                            distance = math.sqrt(x_cam*x_cam + y_cam*y_cam + z_cam*z_cam)
                            points.append(distance)
            
            if points:
                # Return the closest point in this direction
                return min(points)
            
        except Exception as e:
            rospy.logwarn("Error processing LiDAR data: %s", str(e))
        
        return None
    
    def image_callback(self, image_msg):
        """Callback for camera images"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            self.latest_image = cv_image
            
            # Update image dimensions if not set
            if self.image_width == 640:  # Default value
                self.image_width = cv_image.shape[1]
                self.image_height = cv_image.shape[0]
            
            # Increment frame counter
            self.frame_count += 1
            
            # Detect objects
            detected_objects = self.detect_objects_camera(cv_image)
            
            # Create debug image
            debug_image = cv_image.copy()
            
            # Draw detected objects and get LiDAR distances
            for obj in detected_objects:
                x, y, w, h = obj['bbox']
                center_x, center_y = obj['center']
                
                # Draw bounding box
                cv2.rectangle(debug_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                
                # Convert image coordinates to LiDAR direction
                angle_x, angle_y = self.image_to_lidar_coordinates(center_x, center_y)
                
                # Get LiDAR distance in this direction
                lidar_distance = self.find_lidar_distance_in_direction(angle_x, angle_y)
                
                # Draw object info
                if lidar_distance is not None:
                    label = "Person: %.2fm (LiDAR)" % lidar_distance
                    cv2.putText(debug_image, label, (x, y - 10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    
                    # Publish distance
                    distance_msg = Float32()
                    distance_msg.data = lidar_distance
                    self.distance_pub.publish(distance_msg)
                    
                    # Publish detection result
                    detection_msg = String()
                    detection_msg.data = "Detected person at %.2f meters (LiDAR)" % lidar_distance
                    self.detection_pub.publish(detection_msg)
                else:
                    cv2.putText(debug_image, "Person: No LiDAR data", (x, y - 10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # Add system info
            cv2.putText(debug_image, "LiDAR-Camera Fusion", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(debug_image, "Objects: %d" % len(detected_objects), (10, 70), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(debug_image, "Frame: %d" % self.frame_count, (10, 110), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Add calibration info
            cv2.putText(debug_image, "LiDAR offset: (%.2f, %.2f, %.2f)" % 
                       (self.lidar_to_camera_x, self.lidar_to_camera_y, self.lidar_to_camera_z), 
                       (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            
            # Convert back to ROS image
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            debug_msg.header = image_msg.header
            
            # Publish debug image
            self.debug_image_pub.publish(debug_msg)
            
            # Log every 30 frames
            if self.frame_count % 30 == 0:
                rospy.loginfo("LiDAR-Camera Fusion: Processed %d frames, detected %d objects", 
                             self.frame_count, len(detected_objects))
            
        except Exception as e:
            rospy.logerr("Error processing image: %s", str(e))
    
    def cloud_callback(self, cloud_msg):
        """Callback for point cloud messages"""
        self.latest_cloud = cloud_msg

if __name__ == '__main__':
    try:
        node = LidarCameraFusionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
