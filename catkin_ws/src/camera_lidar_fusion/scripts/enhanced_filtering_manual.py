#!/usr/bin/env python

import rospy
import numpy as np
import struct
import time
import threading
import math
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32MultiArray, Bool, String
from geometry_msgs.msg import PoseArray, Pose
import tf2_ros
import tf2_geometry_msgs
from collections import deque

class EnhancedFilteringManual:
    def __init__(self):
        rospy.init_node('enhanced_filtering_manual', anonymous=True)
        
        # Publishers
        self.filtered_cloud_pub = rospy.Publisher('/fused/enhanced_cloud', PointCloud2, queue_size=1)
        self.ground_cloud_pub = rospy.Publisher('/fused/ground_cloud', PointCloud2, queue_size=1)
        self.obstacle_cloud_pub = rospy.Publisher('/fused/obstacle_cloud', PointCloud2, queue_size=1)
        self.stats_pub = rospy.Publisher('/filtering/statistics', Float32MultiArray, queue_size=1)
        self.status_pub = rospy.Publisher('/filtering/status', String, queue_size=1)
        
        # Subscribers
        self.cloud_sub = rospy.Subscriber('/fused/cloud', PointCloud2, self.cloud_callback, queue_size=1)
        
        # Parameters
        self.voxel_size = rospy.get_param('~voxel_size', 0.05)
        self.z_min = rospy.get_param('~z_min', -0.2)
        self.z_max = rospy.get_param('~z_max', 2.5)
        self.max_points = rospy.get_param('~max_points', 50000)
        self.ground_threshold = rospy.get_param('~ground_threshold', 0.1)
        self.outlier_radius = rospy.get_param('~outlier_radius', 0.1)
        self.min_neighbors = rospy.get_param('~min_neighbors', 5)
        self.ransac_threshold = rospy.get_param('~ransac_threshold', 0.05)
        self.ransac_max_iterations = rospy.get_param('~ransac_max_iterations', 100)
        
        # Processing state
        self.processing_lock = threading.Lock()
        self.point_history = deque(maxlen=10)
        self.processing_times = deque(maxlen=100)
        
        # Performance monitoring
        self.frame_count = 0
        self.start_time = time.time()
        
        rospy.loginfo("Enhanced Filtering Manual Node initialized")
        
    def pointcloud2_to_array(self, cloud_msg):
        """Convert PointCloud2 to numpy array"""
        try:
            cloud_data = cloud_msg.data
            point_step = cloud_msg.point_step
            num_points = len(cloud_data) // point_step
            
            points = []
            for i in range(num_points):
                offset = i * point_step
                x = struct.unpack('f', cloud_data[offset:offset+4])[0]
                y = struct.unpack('f', cloud_data[offset+4:offset+8])[0]
                z = struct.unpack('f', cloud_data[offset+8:offset+12])[0]
                points.append([x, y, z])
            
            return np.array(points)
        except Exception as e:
            rospy.logerr("Error converting pointcloud2 to array: {}".format(e))
            return np.array([])
    
    def array_to_pointcloud2(self, points, header):
        """Convert numpy array to PointCloud2"""
        try:
            cloud_msg = PointCloud2()
            cloud_msg.header = header
            cloud_msg.height = 1
            cloud_msg.width = len(points)
            
            cloud_msg.fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1)
            ]
            cloud_msg.point_step = 12
            cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
            
            cloud_msg.data = []
            for point in points:
                cloud_msg.data.extend(struct.pack('fff', point[0], point[1], point[2]))
            
            return cloud_msg
        except Exception as e:
            rospy.logerr("Error converting array to pointcloud2: {}".format(e))
            return None
    
    def roi_filtering(self, points):
        """Intelligent ROI filtering"""
        if len(points) == 0:
            return points
        
        # Z-axis filtering
        mask = (points[:, 2] >= self.z_min) & (points[:, 2] <= self.z_max)
        
        # Distance filtering with smooth transitions
        distances = np.sqrt(points[:, 0]**2 + points[:, 1]**2 + points[:, 2]**2)
        mask &= (distances >= 0.1) & (distances <= 10.0)
        
        # Frustum-based filtering (simple camera FOV approximation)
        horizontal_angle = np.arctan2(points[:, 0], points[:, 2])
        vertical_angle = np.arctan2(points[:, 1], points[:, 2])
        
        # 90-degree horizontal FOV, 60-degree vertical FOV
        mask &= (np.abs(horizontal_angle) < np.pi/4) & (np.abs(vertical_angle) < np.pi/6)
        
        return points[mask]
    
    def adaptive_voxel_downsample(self, points):
        """Adaptive voxel downsampling based on point density"""
        if len(points) == 0:
            return points
        
        # Calculate point density in different regions
        distances = np.sqrt(points[:, 0]**2 + points[:, 1]**2 + points[:, 2]**2)
        
        # Adaptive voxel size based on distance
        adaptive_voxel_sizes = np.where(distances < 2.0, self.voxel_size * 0.5, self.voxel_size)
        adaptive_voxel_sizes = np.where(distances > 5.0, self.voxel_size * 2.0, adaptive_voxel_sizes)
        
        # Apply adaptive voxel downsampling
        downsampled_points = []
        processed_voxels = set()
        
        for i, point in enumerate(points):
            voxel_size = adaptive_voxel_sizes[i]
            voxel_x = int(point[0] / voxel_size)
            voxel_y = int(point[1] / voxel_size)
            voxel_z = int(point[2] / voxel_size)
            voxel_key = (voxel_x, voxel_y, voxel_z)
            
            if voxel_key not in processed_voxels:
                downsampled_points.append(point)
                processed_voxels.add(voxel_key)
        
        return np.array(downsampled_points)
    
    def ransac_ground_detection(self, points):
        """Manual RANSAC implementation for ground plane detection"""
        if len(points) < 3:
            return np.array([]), np.array([]), points
        
        best_ground_points = []
        best_ground_normal = None
        max_inliers = 0
        
        for iteration in range(self.ransac_max_iterations):
            # Randomly select 3 points
            indices = np.random.choice(len(points), 3, replace=False)
            p1, p2, p3 = points[indices]
            
            # Calculate plane normal
            v1 = p2 - p1
            v2 = p3 - p1
            normal = np.cross(v1, v2)
            normal = normal / np.linalg.norm(normal)
            
            # Calculate distance from all points to plane
            distances = np.abs(np.dot(points - p1, normal))
            
            # Count inliers
            inliers = points[distances < self.ransac_threshold]
            
            if len(inliers) > max_inliers and len(inliers) > len(points) * 0.1:
                max_inliers = len(inliers)
                best_ground_points = inliers
                best_ground_normal = normal
        
        if best_ground_points is not None and len(best_ground_points) > 0:
            # Separate ground and obstacle points
            ground_mask = np.zeros(len(points), dtype=bool)
            for ground_point in best_ground_points:
                # Find matching points in original array
                for i, point in enumerate(points):
                    if np.allclose(point, ground_point):
                        ground_mask[i] = True
                        break
            
            ground_points = points[ground_mask]
            obstacle_points = points[~ground_mask]
        else:
            # Fallback to simple height-based ground detection
            ground_mask = points[:, 2] <= self.ground_threshold
            ground_points = points[ground_mask]
            obstacle_points = points[~ground_mask]
            best_ground_normal = np.array([0, 0, 1])
        
        return best_ground_normal, ground_points, obstacle_points
    
    def radius_outlier_removal(self, points):
        """Manual radius-based outlier removal"""
        if len(points) < 10:
            return points
        
        # Build KD-tree-like structure (simplified)
        filtered_points = []
        
        for i, point in enumerate(points):
            # Count neighbors within radius
            neighbor_count = 0
            for j, other_point in enumerate(points):
                if i != j:
                    distance = np.linalg.norm(point - other_point)
                    if distance < self.outlier_radius:
                        neighbor_count += 1
            
            if neighbor_count >= self.min_neighbors:
                filtered_points.append(point)
        
        return np.array(filtered_points) if filtered_points else points
    
    def statistical_outlier_removal(self, points):
        """Manual statistical outlier removal"""
        if len(points) < 10:
            return points
        
        # Calculate distances to nearest neighbors
        distances = []
        for i, point in enumerate(points):
            point_distances = []
            for j, other_point in enumerate(points):
                if i != j:
                    distance = np.linalg.norm(point - other_point)
                    point_distances.append(distance)
            
            if point_distances:
                # Get k nearest neighbors (k=5)
                point_distances.sort()
                k_nearest = point_distances[:5]
                distances.append(np.mean(k_nearest))
            else:
                distances.append(0)
        
        distances = np.array(distances)
        
        # Calculate mean and standard deviation
        mean_distance = np.mean(distances)
        std_distance = np.std(distances)
        
        # Remove outliers (points with distance > mean + 2*std)
        threshold = mean_distance + 2 * std_distance
        filtered_mask = distances <= threshold
        
        return points[filtered_mask]
    
    def cluster_based_filtering(self, points):
        """Manual cluster-based filtering"""
        if len(points) < 10:
            return points
        
        # Simple clustering based on distance
        clusters = []
        visited = set()
        
        for i, point in enumerate(points):
            if i in visited:
                continue
            
            # Start new cluster
            cluster = [i]
            visited.add(i)
            
            # Find all connected points
            to_visit = [i]
            while to_visit:
                current = to_visit.pop(0)
                for j, other_point in enumerate(points):
                    if j not in visited:
                        distance = np.linalg.norm(points[current] - other_point)
                        if distance < self.outlier_radius * 2:
                            cluster.append(j)
                            visited.add(j)
                            to_visit.append(j)
            
            clusters.append(cluster)
        
        # Keep only large clusters
        filtered_points = []
        for cluster in clusters:
            if len(cluster) >= 3:  # Minimum cluster size
                for idx in cluster:
                    filtered_points.append(points[idx])
        
        return np.array(filtered_points) if filtered_points else points
    
    def temporal_consistency_filtering(self, points):
        """Temporal consistency filtering"""
        if len(self.point_history) == 0:
            self.point_history.append(points)
            return points
        
        prev_points = self.point_history[-1]
        
        if len(prev_points) == 0:
            self.point_history.append(points)
            return points
        
        # Find points that are consistent with previous frame
        consistent_points = []
        for point in points:
            # Find closest point in previous frame
            distances = np.linalg.norm(prev_points - point, axis=1)
            min_distance = np.min(distances)
            
            # If point is close to a previous point, keep it
            if min_distance < 0.1:  # 10cm threshold
                consistent_points.append(point)
        
        self.point_history.append(points)
        
        return np.array(consistent_points) if consistent_points else points
    
    def cloud_callback(self, cloud_msg):
        start_time = time.time()
        
        try:
            with self.processing_lock:
                # Convert to numpy array
                points = self.pointcloud2_to_array(cloud_msg)
                if len(points) == 0:
                    return
                
                original_count = len(points)
                
                # Apply enhanced filters
                points = self.roi_filtering(points)
                roi_count = len(points)
                
                points = self.adaptive_voxel_downsample(points)
                voxel_count = len(points)
                
                ground_normal, ground_points, obstacle_points = self.ransac_ground_detection(points)
                ground_count = len(ground_points)
                obstacle_count = len(obstacle_points)
                
                if len(obstacle_points) > 0:
                    obstacle_points = self.radius_outlier_removal(obstacle_points)
                    radius_count = len(obstacle_points)
                    
                    obstacle_points = self.statistical_outlier_removal(obstacle_points)
                    statistical_count = len(obstacle_points)
                    
                    obstacle_points = self.cluster_based_filtering(obstacle_points)
                    cluster_count = len(obstacle_points)
                    
                    obstacle_points = self.temporal_consistency_filtering(obstacle_points)
                    temporal_count = len(obstacle_points)
                else:
                    radius_count = statistical_count = cluster_count = temporal_count = 0
                
                # Limit points for performance
                if len(obstacle_points) > self.max_points:
                    indices = np.random.choice(len(obstacle_points), self.max_points, replace=False)
                    obstacle_points = obstacle_points[indices]
                
                # Publish filtered clouds
                if len(obstacle_points) > 0:
                    filtered_cloud = self.array_to_pointcloud2(obstacle_points, cloud_msg.header)
                    if filtered_cloud:
                        self.filtered_cloud_pub.publish(filtered_cloud)
                
                if len(ground_points) > 0:
                    ground_cloud = self.array_to_pointcloud2(ground_points, cloud_msg.header)
                    if ground_cloud:
                        self.ground_cloud_pub.publish(ground_cloud)
                
                if len(obstacle_points) > 0:
                    obstacle_cloud = self.array_to_pointcloud2(obstacle_points, cloud_msg.header)
                    if obstacle_cloud:
                        self.obstacle_cloud_pub.publish(obstacle_cloud)
                
                # Calculate processing time
                processing_time = time.time() - start_time
                self.processing_times.append(processing_time)
                
                # Calculate FPS
                self.frame_count += 1
                elapsed_time = time.time() - self.start_time
                fps = self.frame_count / elapsed_time if elapsed_time > 0 else 0
                
                # Publish statistics
                stats = Float32MultiArray()
                stats.data = [
                    original_count, roi_count, voxel_count, ground_count,
                    obstacle_count, radius_count, statistical_count, cluster_count, temporal_count,
                    processing_time, fps
                ]
                self.stats_pub.publish(stats)
                
                # Publish status
                status_msg = String()
                status_msg.data = "Enhanced Processing: {:.3f}s, FPS: {:.1f}, Points: {}".format(processing_time, fps, len(obstacle_points))
                self.status_pub.publish(status_msg)
                
                # Log performance every 100 frames
                if self.frame_count % 100 == 0:
                    avg_time = np.mean(self.processing_times)
                    rospy.loginfo("Enhanced Filtering Manual - Avg time: {:.3f}s, FPS: {:.1f}, Points: {}".format(avg_time, fps, len(obstacle_points)))
                
        except Exception as e:
            rospy.logerr("Error in enhanced filtering: {}".format(e))
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = EnhancedFilteringManual()
        node.run()
    except rospy.ROSInterruptException:
        pass
