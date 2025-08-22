#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import math
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String
from cv_bridge import CvBridge
import struct

class LidarDistanceTest:
    def __init__(self):
        rospy.init_node('lidar_distance_test', anonymous=True)
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Publishers
        self.debug_image_pub = rospy.Publisher('/fused/debug_image', Image, queue_size=1)
        
        # Subscribers
        self.image_sub = rospy.Subscriber('/zed/left/image_raw', Image, self.image_callback)
        self.cloud_sub = rospy.Subscriber('/fused/cloud', PointCloud2, self.cloud_callback)
        
        # Store latest data
        self.latest_cloud = None
        self.latest_image = None
        self.frame_count = 0
        
        # Distance test results
        self.closest_distance = None
        self.farthest_distance = None
        self.avg_distance = None
        self.total_points = 0
        
        rospy.loginfo("LiDAR Distance Test initialized")
    
    def analyze_lidar_distances(self):
        """Analyze LiDAR distances and show raw data"""
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
                rospy.logwarn("Point cloud missing x, y, or z fields")
                return None
            
            # Extract points
            for i in range(0, len(self.latest_cloud.data), point_step):
                if i + point_step <= len(self.latest_cloud.data):
                    point_data = self.latest_cloud.data[i:i+point_step]
                    
                    # Extract x, y, z values
                    x = struct.unpack('f', point_data[x_offset:x_offset+4])[0]
                    y = struct.unpack('f', point_data[y_offset:y_offset+4])[0]
                    z = struct.unpack('f', point_data[z_offset:z_offset+4])[0]
                    
                    # Calculate distance from origin (LiDAR position)
                    distance = math.sqrt(x*x + y*y + z*z)
                    
                    # Only consider reasonable distances (not too close, not too far)
                    if 0.1 < distance < 10.0:
                        points.append({
                            'x': x,
                            'y': y,
                            'z': z,
                            'distance': distance
                        })
            
            if points:
                # Analyze distances
                distances = [p['distance'] for p in points]
                self.closest_distance = min(distances)
                self.farthest_distance = max(distances)
                self.avg_distance = sum(distances) / len(distances)
                self.total_points = len(points)
                
                # Log some sample points
                rospy.loginfo("LiDAR Test - Points: %d, Closest: %.2fm, Farthest: %.2fm, Avg: %.2fm", 
                             self.total_points, self.closest_distance, self.farthest_distance, self.avg_distance)
                
                # Show first few points
                for i, point in enumerate(points[:5]):
                    rospy.loginfo("Point %d: (%.2f, %.2f, %.2f) = %.2fm", 
                                 i+1, point['x'], point['y'], point['z'], point['distance'])
                
                return True
            
        except Exception as e:
            rospy.logerr("Error analyzing LiDAR data: %s", str(e))
        
        return None
    
    def image_callback(self, image_msg):
        """Callback for camera images"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            self.latest_image = cv_image
            
            # Increment frame counter
            self.frame_count += 1
            
            # Analyze LiDAR distances
            lidar_working = self.analyze_lidar_distances()
            
            # Create debug image
            debug_image = cv_image.copy()
            
            # Add test info
            cv2.putText(debug_image, "LiDAR Distance Test", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            
            # Add LiDAR status
            if lidar_working:
                cv2.putText(debug_image, "LiDAR: WORKING", (10, 70), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                
                # Add distance info
                cv2.putText(debug_image, "Points: %d" % self.total_points, (10, 110), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                cv2.putText(debug_image, "Closest: %.2fm" % self.closest_distance, (10, 140), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                cv2.putText(debug_image, "Farthest: %.2fm" % self.farthest_distance, (10, 170), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                cv2.putText(debug_image, "Average: %.2fm" % self.avg_distance, (10, 200), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Add instructions
                cv2.putText(debug_image, "Test Instructions:", (10, 240), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                cv2.putText(debug_image, "1. Place object at 1m distance", (10, 265), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(debug_image, "2. Check if closest distance ~1.0m", (10, 290), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(debug_image, "3. Move object closer/farther", (10, 315), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
            else:
                cv2.putText(debug_image, "LiDAR: NOT WORKING", (10, 70), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                cv2.putText(debug_image, "No LiDAR data received", (10, 110), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Add frame counter
            cv2.putText(debug_image, "Frame: %d" % self.frame_count, (10, 350), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            
            # Convert back to ROS image
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            debug_msg.header = image_msg.header
            
            # Publish debug image
            self.debug_image_pub.publish(debug_msg)
            
        except Exception as e:
            rospy.logerr("Error in distance test: %s", str(e))
    
    def cloud_callback(self, cloud_msg):
        """Callback for point cloud messages"""
        self.latest_cloud = cloud_msg

if __name__ == '__main__':
    try:
        node = LidarDistanceTest()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
