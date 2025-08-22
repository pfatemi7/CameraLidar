#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import math
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String
from cv_bridge import CvBridge
import struct

class CalibrationTool:
    def __init__(self):
        rospy.init_node('calibration_tool', anonymous=True)
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Publishers
        self.debug_image_pub = rospy.Publisher('/fused/debug_image', Image, queue_size=1)
        
        # Subscribers
        self.image_sub = rospy.Subscriber('/zed/left/image_raw', Image, self.image_callback)
        self.cloud_sub = rospy.Subscriber('/fused/cloud', PointCloud2, self.cloud_callback)
        
        # Calibration parameters (adjust these!)
        self.lidar_to_camera_x = rospy.get_param('~lidar_to_camera_x', 0.0)
        self.lidar_to_camera_y = rospy.get_param('~lidar_to_camera_y', 0.0)
        self.lidar_to_camera_z = rospy.get_param('~lidar_to_camera_z', 0.1)
        
        # Camera parameters (adjust these!)
        self.camera_fov_horizontal = rospy.get_param('~camera_fov_horizontal', 90.0)
        self.camera_fov_vertical = rospy.get_param('~camera_fov_vertical', 60.0)
        
        # Store latest data
        self.latest_cloud = None
        self.latest_image = None
        self.frame_count = 0
        
        # Calibration mode
        self.calibration_mode = "distance_check"  # or "fov_check"
        
        rospy.loginfo("Calibration Tool initialized")
        rospy.loginfo("Current LiDAR offset: (%.3f, %.3f, %.3f)", 
                     self.lidar_to_camera_x, self.lidar_to_camera_y, self.lidar_to_camera_z)
        rospy.loginfo("Current Camera FOV: %.1f° x %.1f°", 
                     self.camera_fov_horizontal, self.camera_fov_vertical)
        rospy.loginfo("Calibration mode: %s", self.calibration_mode)
    
    def analyze_lidar_data(self):
        """Analyze LiDAR data to understand the setup"""
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
                    
                    # Transform to camera coordinates
                    x_cam = x - self.lidar_to_camera_x
                    y_cam = y - self.lidar_to_camera_y
                    z_cam = z - self.lidar_to_camera_z
                    
                    if z_cam > 0:  # Only consider points in front of camera
                        distance = math.sqrt(x_cam*x_cam + y_cam*y_cam + z_cam*z_cam)
                        angle_x = math.degrees(math.atan2(x_cam, z_cam))
                        angle_y = math.degrees(math.atan2(y_cam, z_cam))
                        
                        points.append({
                            'distance': distance,
                            'angle_x': angle_x,
                            'angle_y': angle_y,
                            'x': x_cam,
                            'y': y_cam,
                            'z': z_cam
                        })
            
            if points:
                # Analyze the data
                distances = [p['distance'] for p in points]
                angles_x = [p['angle_x'] for p in points]
                angles_y = [p['angle_y'] for p in points]
                
                analysis = {
                    'total_points': len(points),
                    'min_distance': min(distances),
                    'max_distance': max(distances),
                    'avg_distance': sum(distances) / len(distances),
                    'min_angle_x': min(angles_x),
                    'max_angle_x': max(angles_x),
                    'min_angle_y': min(angles_y),
                    'max_angle_y': max(angles_y),
                    'fov_x_span': max(angles_x) - min(angles_x),
                    'fov_y_span': max(angles_y) - min(angles_y)
                }
                
                return analysis
            
        except Exception as e:
            rospy.logwarn("Error analyzing LiDAR data: %s", str(e))
        
        return None
    
    def image_callback(self, image_msg):
        """Callback for camera images"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            self.latest_image = cv_image
            
            # Increment frame counter
            self.frame_count += 1
            
            # Analyze LiDAR data
            lidar_analysis = self.analyze_lidar_data()
            
            # Create debug image
            debug_image = cv_image.copy()
            
            # Add calibration info
            cv2.putText(debug_image, "LiDAR-Camera Calibration Tool", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            
            # Add current parameters
            cv2.putText(debug_image, "LiDAR offset: (%.3f, %.3f, %.3f)" % 
                       (self.lidar_to_camera_x, self.lidar_to_camera_y, self.lidar_to_camera_z), 
                       (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)
            
            cv2.putText(debug_image, "Camera FOV: %.1f x %.1f degrees" % 
                       (self.camera_fov_horizontal, self.camera_fov_vertical), 
                       (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)
            
            # Add LiDAR analysis
            if lidar_analysis:
                cv2.putText(debug_image, "LiDAR Points: %d" % lidar_analysis['total_points'], 
                           (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
                
                cv2.putText(debug_image, "Distance: %.2f - %.2f m (avg: %.2f)" % 
                           (lidar_analysis['min_distance'], lidar_analysis['max_distance'], 
                            lidar_analysis['avg_distance']), 
                           (10, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
                
                cv2.putText(debug_image, "Angles X: %.1f to %.1f (span: %.1f)" % 
                           (lidar_analysis['min_angle_x'], lidar_analysis['max_angle_x'], 
                            lidar_analysis['fov_x_span']), 
                           (10, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
                
                cv2.putText(debug_image, "Angles Y: %.1f to %.1f (span: %.1f)" % 
                           (lidar_analysis['min_angle_y'], lidar_analysis['max_angle_y'], 
                            lidar_analysis['fov_y_span']), 
                           (10, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
                
                # Add calibration suggestions
                if lidar_analysis['fov_x_span'] > 0:
                    cv2.putText(debug_image, "Suggested FOV X: %.1f degrees" % 
                               lidar_analysis['fov_x_span'], 
                               (10, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
                
                if lidar_analysis['fov_y_span'] > 0:
                    cv2.putText(debug_image, "Suggested FOV Y: %.1f degrees" % 
                               lidar_analysis['fov_y_span'], 
                               (10, 280), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
            else:
                cv2.putText(debug_image, "No LiDAR data available", 
                           (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
            
            # Add instructions
            cv2.putText(debug_image, "Instructions:", (10, 320), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            cv2.putText(debug_image, "1. Place object at known distance (e.g., 1m)", 
                       (10, 350), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(debug_image, "2. Check if LiDAR distance matches", 
                       (10, 375), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(debug_image, "3. Adjust parameters in launch file", 
                       (10, 400), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Convert back to ROS image
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            debug_msg.header = image_msg.header
            
            # Publish debug image
            self.debug_image_pub.publish(debug_msg)
            
            # Log analysis every 30 frames
            if self.frame_count % 30 == 0 and lidar_analysis:
                rospy.loginfo("LiDAR Analysis: %d points, Distance: %.2f-%.2fm, FOV: %.1fx%.1f°", 
                             lidar_analysis['total_points'], 
                             lidar_analysis['min_distance'], lidar_analysis['max_distance'],
                             lidar_analysis['fov_x_span'], lidar_analysis['fov_y_span'])
            
        except Exception as e:
            rospy.logerr("Error in calibration tool: %s", str(e))
    
    def cloud_callback(self, cloud_msg):
        """Callback for point cloud messages"""
        self.latest_cloud = cloud_msg

if __name__ == '__main__':
    try:
        node = CalibrationTool()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
