#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs import point_cloud2
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, Vector3, Quaternion, TransformStamped
from std_msgs.msg import ColorRGBA, Header
import math

class ProximityVisualizer:
    def __init__(self):
        rospy.init_node('proximity_visualizer', anonymous=True)
        
        # Parameters
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.prox_topic = rospy.get_param('~prox_topic', '/mavros/obstacle/send')
        self.lidar_topic = rospy.get_param('~lidar_topic', '/unilidar/cloud')
        self.min_range = rospy.get_param('~min_range', 1.0)
        self.max_range = rospy.get_param('~max_range', 5.0)
        self.sector_width_deg = rospy.get_param('~sector_width_deg', 5.0)
        self.marker_ns = rospy.get_param('~marker_ns', 'prox_ring')
        self.alpha_valid = rospy.get_param('~alpha_valid', 0.9)
        self.alpha_invalid = rospy.get_param('~alpha_invalid', 0.15)
        self.show_invalid = rospy.get_param('~show_invalid', True)
        self.republish_cloud = rospy.get_param('~republish_cloud', True)
        
        # TF buffer for transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Publishers
        self.marker_pub = rospy.Publisher('~proximity_markers', MarkerArray, queue_size=1)
        if self.republish_cloud:
            self.cloud_pub = rospy.Publisher('~l1_cloud_base', PointCloud2, queue_size=1)
        
        # Subscribers
        rospy.Subscriber(self.prox_topic, LaserScan, self.proximity_callback)
        rospy.Subscriber(self.lidar_topic, PointCloud2, self.lidar_callback)
        
        # State
        self.latest_proximity = None
        self.marker_id_counter = 0
        
        rospy.loginfo(f"Proximity Visualizer initialized:")
        rospy.loginfo(f"  Base frame: {self.base_frame}")
        rospy.loginfo(f"  Proximity topic: {self.prox_topic}")
        rospy.loginfo(f"  LiDAR topic: {self.lidar_topic}")
        rospy.loginfo(f"  Range: {self.min_range}-{self.max_range}m")
        rospy.loginfo(f"  Sector width: {self.sector_width_deg}°")
        
    def proximity_callback(self, msg):
        """Handle proximity data and create visualization markers"""
        self.latest_proximity = msg
        self.create_proximity_markers(msg)
        
    def lidar_callback(self, msg):
        """Handle LiDAR point cloud and optionally republish in base frame"""
        if not self.republish_cloud:
            return
            
        try:
            # Transform point cloud to base frame
            transformed_cloud = self.transform_pointcloud(msg, self.base_frame)
            if transformed_cloud:
                self.cloud_pub.publish(transformed_cloud)
        except Exception as e:
            rospy.logwarn(f"Failed to transform point cloud: {e}")
    
    def transform_pointcloud(self, cloud_msg, target_frame):
        """Transform point cloud to target frame"""
        try:
            # Get transform
            transform = self.tf_buffer.lookup_transform(
                target_frame, cloud_msg.header.frame_id, 
                cloud_msg.header.stamp, rospy.Duration(0.1)
            )
            
            # Transform point cloud
            transformed_cloud = tf2_geometry_msgs.do_transform_cloud(cloud_msg, transform)
            transformed_cloud.header.frame_id = target_frame
            transformed_cloud.header.stamp = rospy.Time.now()
            
            return transformed_cloud
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF transform failed: {e}")
            return None
    
    def create_proximity_markers(self, proximity_msg):
        """Create 360° sector ring visualization"""
        marker_array = MarkerArray()
        
        # Clear previous markers
        clear_marker = Marker()
        clear_marker.header.frame_id = self.base_frame
        clear_marker.header.stamp = rospy.Time.now()
        clear_marker.ns = self.marker_ns
        clear_marker.id = 0
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        
        # Create range rings (min and max)
        marker_array.markers.extend(self.create_range_rings())
        
        # Create sector wedges
        if hasattr(proximity_msg, 'ranges') and proximity_msg.ranges:
            marker_array.markers.extend(self.create_sector_wedges(proximity_msg))
        
        # Publish markers
        self.marker_pub.publish(marker_array)
    
    def create_range_rings(self):
        """Create circular rings for min and max range"""
        markers = []
        
        for i, radius in enumerate([self.min_range, self.max_range]):
            marker = Marker()
            marker.header.frame_id = self.base_frame
            marker.header.stamp = rospy.Time.now()
            marker.ns = self.marker_ns
            marker.id = 1000 + i  # Use high IDs for rings
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            
            # Create circle points
            points = []
            for angle in np.linspace(0, 2*math.pi, 72):
                x = radius * math.cos(angle)
                y = radius * math.sin(angle)
                points.append(Point(x, y, 0))
            
            marker.points = points
            
            # Styling
            marker.scale.x = 0.05  # Line width
            marker.color = ColorRGBA(0.5, 0.5, 0.5, 0.3)  # Gray, semi-transparent
            
            markers.append(marker)
        
        return markers
    
    def create_sector_wedges(self, proximity_msg):
        """Create sector wedges based on proximity data"""
        markers = []
        
        # Get sector parameters
        angle_min = proximity_msg.angle_min
        angle_increment = proximity_msg.angle_increment
        ranges = proximity_msg.ranges
        
        for i, distance in enumerate(ranges):
            # Skip invalid readings if configured
            if not self.show_invalid and (distance <= 0 or distance > self.max_range):
                continue
            
            # Clamp distance to valid range
            clamped_distance = np.clip(distance, self.min_range, self.max_range)
            
            # Calculate sector angle
            sector_angle = angle_min + i * angle_increment
            
            # Create wedge marker
            marker = self.create_sector_wedge(i, sector_angle, clamped_distance, distance)
            if marker:
                markers.append(marker)
        
        return markers
    
    def create_sector_wedge(self, sector_id, angle, clamped_distance, original_distance):
        """Create a single sector wedge"""
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = self.marker_ns
        marker.id = sector_id + 1  # Start from 1
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        
        # Calculate sector width (use parameter or derive from message)
        sector_width = math.radians(self.sector_width_deg)
        
        # Create triangle points for the wedge
        points = []
        
        # Inner radius (min_range)
        inner_radius = self.min_range
        
        # Outer radius (clamped distance)
        outer_radius = clamped_distance
        
        # Create two triangles to form the wedge
        angle1 = angle - sector_width/2
        angle2 = angle + sector_width/2
        
        # Triangle 1: (0,0) -> (r1,a1) -> (r1,a2)
        points.extend([
            Point(0, 0, 0),
            Point(inner_radius * math.cos(angle1), inner_radius * math.sin(angle1), 0),
            Point(inner_radius * math.cos(angle2), inner_radius * math.sin(angle2), 0)
        ])
        
        # Triangle 2: (r1,a1) -> (r2,a1) -> (r2,a2)
        points.extend([
            Point(inner_radius * math.cos(angle1), inner_radius * math.sin(angle1), 0),
            Point(outer_radius * math.cos(angle1), outer_radius * math.sin(angle1), 0),
            Point(outer_radius * math.cos(angle2), outer_radius * math.sin(angle2), 0)
        ])
        
        # Triangle 3: (r1,a2) -> (r2,a2) -> (r1,a1) - to complete the wedge
        points.extend([
            Point(inner_radius * math.cos(angle2), inner_radius * math.sin(angle2), 0),
            Point(outer_radius * math.cos(angle2), outer_radius * math.sin(angle2), 0),
            Point(inner_radius * math.cos(angle1), inner_radius * math.sin(angle1), 0)
        ])
        
        marker.points = points
        
        # Color based on distance and validity
        if original_distance <= 0 or original_distance > self.max_range:
            # Invalid reading
            color = ColorRGBA(0.5, 0.5, 0.5, self.alpha_invalid)
        else:
            # Valid reading - color from green (far) to red (near)
            normalized_distance = (clamped_distance - self.min_range) / (self.max_range - self.min_range)
            if normalized_distance < 0.5:
                # Green to yellow
                r = normalized_distance * 2
                g = 1.0
                b = 0.0
            else:
                # Yellow to red
                r = 1.0
                g = 1.0 - (normalized_distance - 0.5) * 2
                b = 0.0
            
            color = ColorRGBA(r, g, b, self.alpha_valid)
        
        marker.color = color
        
        return marker

def main():
    try:
        visualizer = ProximityVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
