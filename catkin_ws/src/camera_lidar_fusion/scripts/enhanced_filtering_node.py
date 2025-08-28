#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Enhanced Filtering Node for Camera-LiDAR Fusion System
Implements advanced filtering techniques for improved accuracy and performance.
"""

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, Float32MultiArray
import struct
import threading
import time
from collections import deque
from sklearn.cluster import DBSCAN
from sklearn.linear_model import RANSACRegressor
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped

class EnhancedFilteringNode:
    def __init__(self):
        rospy.init_node('enhanced_filtering_node', anonymous=True)
        
        # Publishers
        self.filtered_cloud_pub = rospy.Publisher('/fused/enhanced_cloud', PointCloud2, queue_size=1)
        self.ground_cloud_pub = rospy.Publisher('/fused/ground_cloud', PointCloud2, queue_size=1)
        self.obstacle_cloud_pub = rospy.Publisher('/fused/obstacle_cloud', PointCloud2, queue_size=1)
        self.filtering_stats_pub = rospy.Publisher('/fused/filtering_stats', Float32MultiArray, queue_size=1)
        
        # Subscribers
        self.cloud_sub = rospy.Subscriber('/unilidar/cloud', PointCloud2, self.cloud_callback)
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Adaptive Voxel Grid Parameters
        self.base_voxel_size = rospy.get_param('~base_voxel_size', 0.05)  # meters
        self.max_voxel_size = rospy.get_param('~max_voxel_size', 0.15)    # meters
        self.min_voxel_size = rospy.get_param('~min_voxel_size', 0.02)    # meters
        self.density_threshold = rospy.get_param('~density_threshold', 100)  # points per voxel
        
        # Multi-resolution voxel grid parameters
        self.distance_ranges = rospy.get_param('~distance_ranges', [0.5, 2.0, 5.0, 10.0])  # meters
        self.voxel_sizes = rospy.get_param('~voxel_sizes', [0.02, 0.05, 0.08, 0.12])  # meters
        
        # RANSAC Ground Plane Detection
        self.ransac_threshold = rospy.get_param('~ransac_threshold', 0.1)  # meters
        self.ransac_min_samples = rospy.get_param('~ransac_min_samples', 3)
        self.ransac_max_trials = rospy.get_param('~ransac_max_trials', 1000)
        self.ground_angle_threshold = rospy.get_param('~ground_angle_threshold', 15.0)  # degrees
        
        # Enhanced Outlier Detection
        self.radius_outlier_radius = rospy.get_param('~radius_outlier_radius', 0.5)  # meters
        self.radius_outlier_min_neighbors = rospy.get_param('~radius_outlier_min_neighbors', 10)
        self.statistical_outlier_k = rospy.get_param('~statistical_outlier_k', 20)
        self.statistical_outlier_std = rospy.get_param('~statistical_outlier_std', 1.0)
        
        # Temporal Consistency
        self.temporal_window_size = rospy.get_param('~temporal_window_size', 5)
        self.temporal_consistency_threshold = rospy.get_param('~temporal_consistency_threshold', 0.1)  # meters
        self.point_history = deque(maxlen=self.temporal_window_size)
        
        # Cluster-based Filtering
        self.cluster_eps = rospy.get_param('~cluster_eps', 0.3)  # meters
        self.cluster_min_samples = rospy.get_param('~cluster_min_samples', 5)
        self.min_cluster_size = rospy.get_param('~min_cluster_size', 10)
        
        # ROI Filtering
        self.camera_fov_horizontal = rospy.get_param('~camera_fov_horizontal', 90.0)  # degrees
        self.camera_fov_vertical = rospy.get_param('~camera_fov_vertical', 60.0)  # degrees
        self.max_distance = rospy.get_param('~max_distance', 20.0)  # meters
        
        # Performance monitoring
        self.processing_times = deque(maxlen=100)
        self.frame_count = 0
        self.last_stats_time = time.time()
        
        # Threading for performance
        self.processing_lock = threading.Lock()
        
        rospy.loginfo("Enhanced Filtering Node initialized")
        rospy.loginfo("Base voxel size: %.3f m", self.base_voxel_size)
        rospy.loginfo("RANSAC threshold: %.3f m", self.ransac_threshold)
        rospy.loginfo("Temporal window size: %d", self.temporal_window_size)
    
    def pointcloud2_to_array(self, cloud_msg):
        """Convert ROS PointCloud2 to numpy array"""
        points_list = []
        
        # Get point step and field offsets
        point_step = cloud_msg.point_step
        x_offset = None
        y_offset = None
        z_offset = None
        intensity_offset = None
        
        for field in cloud_msg.fields:
            if field.name == 'x':
                x_offset = field.offset
            elif field.name == 'y':
                y_offset = field.offset
            elif field.name == 'z':
                z_offset = field.offset
            elif field.name == 'intensity':
                intensity_offset = field.offset
        
        if x_offset is None or y_offset is None or z_offset is None:
            rospy.logwarn("Missing x, y, or z field in point cloud")
            return np.array([])
        
        # Extract points
        for i in range(0, len(cloud_msg.data), point_step):
            if i + point_step <= len(cloud_msg.data):
                point_data = cloud_msg.data[i:i+point_step]
                
                x = struct.unpack('f', point_data[x_offset:x_offset+4])[0]
                y = struct.unpack('f', point_data[y_offset:y_offset+4])[0]
                z = struct.unpack('f', point_data[z_offset:z_offset+4])[0]
                
                intensity = 0.0
                if intensity_offset is not None:
                    intensity = struct.unpack('f', point_data[intensity_offset:intensity_offset+4])[0]
                
                points_list.append([x, y, z, intensity])
        
        return np.array(points_list)
    
    def adaptive_voxel_downsample(self, points):
        """Adaptive voxel downsampling based on point density"""
        if len(points) == 0:
            return points
        
        # Calculate point density in different regions
        distances = np.linalg.norm(points[:, :3], axis=1)
        
        # Adaptive voxel sizes based on distance
        voxel_sizes = np.ones(len(points)) * self.base_voxel_size
        
        for i, distance in enumerate(distances):
            # Find appropriate voxel size based on distance
            for j, range_limit in enumerate(self.distance_ranges):
                if distance <= range_limit:
                    voxel_sizes[i] = self.voxel_sizes[j]
                    break
        
        # Apply multi-resolution voxel downsampling
        filtered_points = []
        processed_voxels = set()
        
        for i, point in enumerate(points):
            voxel_size = voxel_sizes[i]
            voxel_key = tuple(np.floor(point[:3] / voxel_size).astype(int))
            
            if voxel_key not in processed_voxels:
                # Find all points in this voxel
                voxel_points = []
                for j, other_point in enumerate(points):
                    other_voxel_key = tuple(np.floor(other_point[:3] / voxel_size).astype(int))
                    if other_voxel_key == voxel_key:
                        voxel_points.append(other_point)
                
                if len(voxel_points) > 0:
                    # Keep the centroid of the voxel
                    centroid = np.mean(voxel_points, axis=0)
                    filtered_points.append(centroid)
                    processed_voxels.add(voxel_key)
        
        return np.array(filtered_points)
    
    def detect_ground_plane_ransac(self, points):
        """Detect ground plane using RANSAC"""
        if len(points) < self.ransac_min_samples:
            return None, points, points
        
        # Extract x, y, z coordinates
        xyz = points[:, :3]
        
        # Use RANSAC to fit a plane (ax + by + cz + d = 0)
        # We'll fit z = ax + by + c, then convert to plane equation
        xy = xyz[:, :2]
        z = xyz[:, 2]
        
        try:
            ransac = RANSACRegressor(
                residual_threshold=self.ransac_threshold,
                min_samples=self.ransac_min_samples,
                max_trials=self.ransac_max_trials,
                random_state=42
            )
            
            ransac.fit(xy, z)
            
            # Get inliers and outliers
            inlier_mask = ransac.inlier_mask_
            outliers_mask = ~inlier_mask
            
            ground_points = points[inlier_mask]
            obstacle_points = points[outliers_mask]
            
            # Check if the detected plane is actually ground (horizontal)
            if len(ground_points) > 0:
                # Calculate plane normal from coefficients
                a, b = ransac.estimator_.coef_
                c = -1.0
                d = ransac.estimator_.intercept_
                
                # Normalize the normal vector
                normal = np.array([a, b, c])
                normal_norm = np.linalg.norm(normal)
                if normal_norm > 0:
                    normal = normal / normal_norm
                    
                    # Calculate angle with vertical (0, 0, 1)
                    vertical = np.array([0, 0, 1])
                    angle = np.arccos(np.abs(np.dot(normal, vertical)))
                    angle_degrees = np.degrees(angle)
                    
                    # If angle is too large, it's not ground
                    if angle_degrees > self.ground_angle_threshold:
                        rospy.logdebug("Detected plane is not ground (angle: %.1f°)", angle_degrees)
                        return None, points, np.array([])
                    
                    rospy.logdebug("Ground plane detected with %d points, angle: %.1f°", 
                                 len(ground_points), angle_degrees)
                    return normal, ground_points, obstacle_points
            
        except Exception as e:
            rospy.logwarn("RANSAC ground detection failed: %s", str(e))
        
        return None, points, np.array([])
    
    def radius_outlier_removal(self, points):
        """Remove outliers using radius-based filtering"""
        if len(points) < self.radius_outlier_min_neighbors:
            return points
        
        # Use DBSCAN for radius-based outlier detection
        clustering = DBSCAN(
            eps=self.radius_outlier_radius,
            min_samples=self.radius_outlier_min_neighbors
        ).fit(points[:, :3])
        
        # Keep points that are not outliers (cluster labels >= 0)
        inlier_mask = clustering.labels_ >= 0
        return points[inlier_mask]
    
    def statistical_outlier_removal(self, points):
        """Remove statistical outliers"""
        if len(points) < self.statistical_outlier_k:
            return points
        
        # Calculate distances to k nearest neighbors
        from sklearn.neighbors import NearestNeighbors
        
        nbrs = NearestNeighbors(n_neighbors=self.statistical_outlier_k, algorithm='ball_tree').fit(points[:, :3])
        distances, indices = nbrs.kneighbors(points[:, :3])
        
        # Calculate mean distance for each point
        mean_distances = np.mean(distances, axis=1)
        
        # Calculate global mean and standard deviation
        global_mean = np.mean(mean_distances)
        global_std = np.std(mean_distances)
        
        # Filter points based on statistical threshold
        threshold = global_mean + self.statistical_outlier_std * global_std
        inlier_mask = mean_distances < threshold
        
        return points[inlier_mask]
    
    def cluster_based_filtering(self, points):
        """Remove isolated noise clusters"""
        if len(points) < self.cluster_min_samples:
            return points
        
        # Use DBSCAN for clustering
        clustering = DBSCAN(
            eps=self.cluster_eps,
            min_samples=self.cluster_min_samples
        ).fit(points[:, :3])
        
        # Count points in each cluster
        unique_labels, counts = np.unique(clustering.labels_, return_counts=True)
        
        # Keep only clusters with sufficient size
        valid_clusters = unique_labels[counts >= self.min_cluster_size]
        valid_clusters = valid_clusters[valid_clusters >= 0]  # Exclude noise (-1)
        
        # Filter points belonging to valid clusters
        valid_mask = np.isin(clustering.labels_, valid_clusters)
        return points[valid_mask]
    
    def temporal_consistency_filtering(self, points):
        """Apply temporal consistency filtering"""
        if len(self.point_history) == 0:
            self.point_history.append(points)
            return points
        
        # Get previous frame points
        prev_points = self.point_history[-1]
        
        if len(prev_points) == 0:
            self.point_history.append(points)
            return points
        
        # Find temporal correspondences using nearest neighbors
        from sklearn.neighbors import NearestNeighbors
        
        nbrs = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(prev_points[:, :3])
        distances, indices = nbrs.kneighbors(points[:, :3])
        
        # Keep points that are temporally consistent
        consistent_mask = distances.flatten() < self.temporal_consistency_threshold
        consistent_points = points[consistent_mask]
        
        # Update history
        self.point_history.append(points)
        
        return consistent_points
    
    def roi_filtering(self, points):
        """Apply ROI filtering based on camera FOV"""
        if len(points) == 0:
            return points
        
        # Calculate spherical coordinates
        x, y, z = points[:, 0], points[:, 1], points[:, 2]
        distances = np.sqrt(x**2 + y**2 + z**2)
        
        # Distance filtering
        distance_mask = distances <= self.max_distance
        
        # Angular filtering (camera FOV)
        horizontal_angles = np.degrees(np.arctan2(y, x))
        vertical_angles = np.degrees(np.arctan2(z, np.sqrt(x**2 + y**2)))
        
        horizontal_mask = np.abs(horizontal_angles) <= self.camera_fov_horizontal / 2.0
        vertical_mask = np.abs(vertical_angles) <= self.camera_fov_vertical / 2.0
        
        # Combine masks
        roi_mask = distance_mask & horizontal_mask & vertical_mask
        
        return points[roi_mask]
    
    def array_to_pointcloud2(self, points, header):
        """Convert numpy array to ROS PointCloud2"""
        cloud_msg = PointCloud2()
        cloud_msg.header = header
        
        if len(points) == 0:
            cloud_msg.height = 1
            cloud_msg.width = 0
            cloud_msg.fields = []
            cloud_msg.point_step = 0
            cloud_msg.row_step = 0
            cloud_msg.data = b''
            cloud_msg.is_dense = True
            return cloud_msg
        
        cloud_msg.height = 1
        cloud_msg.width = len(points)
        
        # Define fields
        cloud_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        
        cloud_msg.point_step = 16
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        
        # Pack data
        cloud_msg.data = b''
        for point in points:
            cloud_msg.data += struct.pack('ffff', point[0], point[1], point[2], point[3])
        
        cloud_msg.is_dense = True
        return cloud_msg
    
    def cloud_callback(self, cloud_msg):
        """Main callback for point cloud processing"""
        start_time = time.time()
        
        try:
            with self.processing_lock:
                # Convert to numpy array
                points = self.pointcloud2_to_array(cloud_msg)
                
                if len(points) == 0:
                    rospy.logwarn("Empty point cloud received")
                    return
                
                original_count = len(points)
                
                # Apply ROI filtering first
                points = self.roi_filtering(points)
                roi_count = len(points)
                
                # Adaptive voxel downsampling
                points = self.adaptive_voxel_downsample(points)
                voxel_count = len(points)
                
                # RANSAC ground plane detection
                ground_normal, ground_points, obstacle_points = self.detect_ground_plane_ransac(points)
                ground_count = len(ground_points) if ground_points is not None else 0
                obstacle_count = len(obstacle_points)
                
                # Enhanced outlier removal on obstacle points
                if len(obstacle_points) > 0:
                    # Radius-based outlier removal
                    obstacle_points = self.radius_outlier_removal(obstacle_points)
                    radius_count = len(obstacle_points)
                    
                    # Statistical outlier removal
                    obstacle_points = self.statistical_outlier_removal(obstacle_points)
                    statistical_count = len(obstacle_points)
                    
                    # Cluster-based filtering
                    obstacle_points = self.cluster_based_filtering(obstacle_points)
                    cluster_count = len(obstacle_points)
                    
                    # Temporal consistency filtering
                    obstacle_points = self.temporal_consistency_filtering(obstacle_points)
                    temporal_count = len(obstacle_points)
                else:
                    radius_count = statistical_count = cluster_count = temporal_count = 0
                
                # Publish filtered clouds
                if len(obstacle_points) > 0:
                    obstacle_cloud = self.array_to_pointcloud2(obstacle_points, cloud_msg.header)
                    self.obstacle_cloud_pub.publish(obstacle_cloud)
                
                if ground_points is not None and len(ground_points) > 0:
                    ground_cloud = self.array_to_pointcloud2(ground_points, cloud_msg.header)
                    self.ground_cloud_pub.publish(ground_cloud)
                
                # Publish combined filtered cloud
                if len(obstacle_points) > 0:
                    filtered_cloud = self.array_to_pointcloud2(obstacle_points, cloud_msg.header)
                    self.filtered_cloud_pub.publish(filtered_cloud)
                
                # Calculate processing time
                processing_time = time.time() - start_time
                self.processing_times.append(processing_time)
                
                # Publish statistics
                self.publish_filtering_stats(
                    original_count, roi_count, voxel_count, ground_count, 
                    obstacle_count, radius_count, statistical_count, 
                    cluster_count, temporal_count, processing_time
                )
                
                self.frame_count += 1
                
                # Log statistics periodically
                if self.frame_count % 30 == 0:
                    avg_time = np.mean(self.processing_times)
                    rospy.loginfo("Enhanced Filtering: Frame %d, Avg time: %.3fs, "
                                "Points: %d->%d (%.1f%% kept)", 
                                self.frame_count, avg_time, original_count, 
                                temporal_count, 
                                (temporal_count/original_count*100) if original_count > 0 else 0)
        
        except Exception as e:
            rospy.logerr("Error in enhanced filtering: %s", str(e))
    
    def publish_filtering_stats(self, original, roi, voxel, ground, obstacle, 
                              radius, statistical, cluster, temporal, processing_time):
        """Publish filtering statistics"""
        stats_msg = Float32MultiArray()
        stats_msg.data = [
            float(original), float(roi), float(voxel), float(ground),
            float(obstacle), float(radius), float(statistical), 
            float(cluster), float(temporal), processing_time
        ]
        self.filtering_stats_pub.publish(stats_msg)

if __name__ == '__main__':
    try:
        node = EnhancedFilteringNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
