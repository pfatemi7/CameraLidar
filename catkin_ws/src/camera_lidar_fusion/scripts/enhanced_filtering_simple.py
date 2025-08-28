#!/usr/bin/env python3

import rospy
import numpy as np
import struct
import time
import threading
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32MultiArray, Bool, String
from geometry_msgs.msg import PoseArray, Pose
import tf2_ros
import tf2_geometry_msgs
from collections import deque

class EnhancedFilteringSimple:
    def __init__(self):
        rospy.init_node('enhanced_filtering_simple', anonymous=True)
        
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
        
        # Processing state
        self.processing_lock = threading.Lock()
        self.point_history = deque(maxlen=10)
        self.processing_times = deque(maxlen=100)
        
        # Performance monitoring
        self.frame_count = 0
        self.start_time = time.time()
        
        rospy.loginfo("Enhanced Filtering Simple Node initialized")
        
    def pointcloud2_to_array(self, cloud_msg):
        """Convert PointCloud2 to numpy array"""
        try:
            # Get point cloud data
            cloud_data = cloud_msg.data
            
            # Calculate number of points
            point_step = cloud_msg.point_step
            num_points = len(cloud_data) // point_step
            
            # Extract XYZ coordinates
            points = []
            for i in range(num_points):
                offset = i * point_step
                x = struct.unpack('f', cloud_data[offset:offset+4])[0]
                y = struct.unpack('f', cloud_data[offset+4:offset+8])[0]
                z = struct.unpack('f', cloud_data[offset+8:offset+12])[0]
                points.append([x, y, z])
            
            return np.array(points)
        except Exception as e:
            rospy.logerr(f"Error converting pointcloud2 to array: {e}")
            return np.array([])
    
    def array_to_pointcloud2(self, points, header):
        """Convert numpy array to PointCloud2"""
        try:
            # Create PointCloud2 message
            cloud_msg = PointCloud2()
            cloud_msg.header = header
            cloud_msg.height = 1
            cloud_msg.width = len(points)
            
            # Define fields
            cloud_msg.fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1)
            ]
            cloud_msg.point_step = 12
            cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
            
            # Pack data
            cloud_msg.data = []
            for point in points:
                cloud_msg.data.extend(struct.pack('fff', point[0], point[1], point[2]))
            
            return cloud_msg
        except Exception as e:
            rospy.logerr(f"Error converting array to pointcloud2: {e}")
            return None
    
    def roi_filtering(self, points):
        """Simple ROI filtering based on Z-axis and distance"""
        if len(points) == 0:
            return points
        
        # Z-axis filtering
        mask = (points[:, 2] >= self.z_min) & (points[:, 2] <= self.z_max)
        
        # Distance filtering (remove points too far or too close)
        distances = np.sqrt(points[:, 0]**2 + points[:, 1]**2 + points[:, 2]**2)
        mask &= (distances >= 0.1) & (distances <= 10.0)
        
        return points[mask]
    
    def simple_voxel_downsample(self, points):
        """Simple voxel downsampling using numpy"""
        if len(points) == 0:
            return points
        
        # Quantize points to voxel grid
        voxel_indices = np.floor(points / self.voxel_size).astype(int)
        
        # Create unique voxel keys
        voxel_keys = voxel_indices[:, 0] * 1000000 + voxel_indices[:, 1] * 1000 + voxel_indices[:, 2]
        
        # Find unique voxels and their representative points
        unique_voxels, inverse_indices = np.unique(voxel_keys, return_inverse=True)
        
        # Take the first point in each voxel
        downsampled_points = []
        for i in range(len(unique_voxels)):
            voxel_points = points[voxel_keys == unique_voxels[i]]
            downsampled_points.append(voxel_points[0])
        
        return np.array(downsampled_points)
    
    def detect_ground_simple(self, points):
        """Simple ground detection based on height"""
        if len(points) == 0:
            return np.array([]), np.array([])
        
        # Simple ground detection: points below threshold
        ground_mask = points[:, 2] <= self.ground_threshold
        ground_points = points[ground_mask]
        obstacle_points = points[~ground_mask]
        
        return ground_points, obstacle_points
    
    def simple_outlier_removal(self, points):
        """Simple outlier removal based on distance to neighbors"""
        if len(points) < 10:
            return points
        
        # Calculate distances between all points
        distances = []
        for i in range(len(points)):
            point_distances = []
            for j in range(len(points)):
                if i != j:
                    dist = np.linalg.norm(points[i] - points[j])
                    point_distances.append(dist)
            distances.append(point_distances)
        
        # Keep points with enough neighbors within radius
        filtered_points = []
        for i, point_distances in enumerate(distances):
            nearby_count = sum(1 for d in point_distances if d < self.outlier_radius)
            if nearby_count >= self.min_neighbors:
                filtered_points.append(points[i])
        
        return np.array(filtered_points) if filtered_points else points
    
    def temporal_consistency_filtering(self, points):
        """Simple temporal consistency filtering"""
        if len(self.point_history) == 0:
            self.point_history.append(points)
            return points
        
        # Get previous frame
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
                
                # Apply filters
                points = self.roi_filtering(points)
                roi_count = len(points)
                
                points = self.simple_voxel_downsample(points)
                voxel_count = len(points)
                
                ground_points, obstacle_points = self.detect_ground_simple(points)
                ground_count = len(ground_points)
                obstacle_count = len(obstacle_points)
                
                if len(obstacle_points) > 0:
                    obstacle_points = self.simple_outlier_removal(obstacle_points)
                    outlier_count = len(obstacle_points)
                    
                    obstacle_points = self.temporal_consistency_filtering(obstacle_points)
                    temporal_count = len(obstacle_points)
                else:
                    outlier_count = temporal_count = 0
                
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
                    obstacle_count, outlier_count, temporal_count,
                    processing_time, fps
                ]
                self.stats_pub.publish(stats)
                
                # Publish status
                status_msg = String()
                status_msg.data = f"Processing: {processing_time:.3f}s, FPS: {fps:.1f}, Points: {len(obstacle_points)}"
                self.status_pub.publish(status_msg)
                
                # Log performance every 100 frames
                if self.frame_count % 100 == 0:
                    avg_time = np.mean(self.processing_times)
                    rospy.loginfo(f"Enhanced Filtering - Avg time: {avg_time:.3f}s, FPS: {fps:.1f}, Points: {len(obstacle_points)}")
                
        except Exception as e:
            rospy.logerr(f"Error in enhanced filtering: {e}")
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = EnhancedFilteringSimple()
        node.run()
    except rospy.ROSInterruptException:
        pass
