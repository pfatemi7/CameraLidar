#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import PointCloud2, Image, CameraInfo, PointField
from geometry_msgs.msg import TransformStamped
import tf2_ros
from cv_bridge import CvBridge
import message_filters
from message_filters import ApproximateTimeSynchronizer, Subscriber
import struct

class CameraLidarFusionSimple:
    def __init__(self):
        rospy.init_node('camera_lidar_fusion_simple', anonymous=True)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize TF
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Publishers
        self.fused_cloud_pub = rospy.Publisher('/fused/cloud', PointCloud2, queue_size=1)
        self.colored_cloud_pub = rospy.Publisher('/fused/colored_cloud', PointCloud2, queue_size=1)
        self.debug_image_pub = rospy.Publisher('/fused/debug_image', Image, queue_size=1)
        
        # Subscribers
        cloud_sub = Subscriber('/unilidar/cloud', PointCloud2)
        image_sub = Subscriber('/zed/left/image_rect_color', Image)
        camera_info_sub = Subscriber('/zed/left/camera_info', CameraInfo)
        
        # Synchronizer
        self.ts = ApproximateTimeSynchronizer([cloud_sub, image_sub, camera_info_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.callback)
        
        # Calibration parameters (you'll need to calibrate these)
        self.camera_matrix = np.array([[1000.0, 0, 640.0],
                                      [0, 1000.0, 480.0],
                                      [0, 0, 1]], dtype=np.float64)
        self.dist_coeffs = np.zeros(5)
        
        # LiDAR to camera transform (you'll need to calibrate these)
        self.rotation_matrix = np.eye(3)
        self.translation_vector = np.array([0.1, 0, 0])  # 10cm offset in X
        
        # Processing parameters
        self.voxel_leaf_size = rospy.get_param('~voxel_leaf_size', 0.03)
        self.z_min = rospy.get_param('~z_min', -0.2)
        self.z_max = rospy.get_param('~z_max', 2.5)
        
        # Publish static transform
        self.publish_static_transform()
        
        rospy.loginfo("Camera-LiDAR fusion simple node initialized")
    
    def publish_static_transform(self):
        """Publish static transform from LiDAR to camera"""
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "camera_link"
        transform.child_frame_id = "lidar_link"
        
        transform.transform.translation.x = self.translation_vector[0]
        transform.transform.translation.y = self.translation_vector[1]
        transform.transform.translation.z = self.translation_vector[2]
        
        # Convert rotation matrix to quaternion
        from tf.transformations import quaternion_from_matrix
        rotation_matrix_4x4 = np.eye(4)
        rotation_matrix_4x4[:3, :3] = self.rotation_matrix
        quat = quaternion_from_matrix(rotation_matrix_4x4)
        
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]
        
        self.tf_broadcaster.sendTransform(transform)
    
    def filter_point_cloud(self, cloud_msg):
        """Apply filters to point cloud using numpy"""
        # Extract points from PointCloud2 message
        points = self.pointcloud2_to_array(cloud_msg)
        
        if len(points) == 0:
            return points
        
        # Z-axis filtering
        mask = (points[:, 2] >= self.z_min) & (points[:, 2] <= self.z_max)
        filtered_points = points[mask]
        
        # Voxel downsampling (simple grid-based)
        if len(filtered_points) > 0:
            voxel_indices = self.voxel_downsample(filtered_points, self.voxel_leaf_size)
            filtered_points = filtered_points[voxel_indices]
        
        return filtered_points
    
    def voxel_downsample(self, points, leaf_size):
        """Simple voxel downsampling using numpy"""
        # Calculate voxel indices
        voxel_indices = np.floor(points / leaf_size).astype(int)
        
        # Create unique voxel keys
        voxel_keys = voxel_indices[:, 0] + voxel_indices[:, 1] * 1000 + voxel_indices[:, 2] * 1000000
        
        # Find unique voxels and keep first point in each voxel
        _, unique_indices = np.unique(voxel_keys, return_index=True)
        
        return unique_indices
    
    def pointcloud2_to_array(self, cloud_msg):
        """Convert PointCloud2 message to numpy array"""
        # Get point cloud data
        cloud_data = cloud_msg.data
        
        # Calculate number of points
        num_points = cloud_msg.width * cloud_msg.height
        
        # Extract x, y, z coordinates
        points = []
        for i in range(num_points):
            offset = i * cloud_msg.point_step
            
            # Extract x, y, z (assuming they are the first 3 fields)
            x = struct.unpack('f', cloud_data[offset:offset+4])[0]
            y = struct.unpack('f', cloud_data[offset+4:offset+8])[0]
            z = struct.unpack('f', cloud_data[offset+8:offset+12])[0]
            
            points.append([x, y, z])
        
        return np.array(points)
    
    def array_to_pointcloud2(self, points, header):
        """Convert numpy array to PointCloud2 message"""
        cloud_msg = PointCloud2()
        cloud_msg.header = header
        
        cloud_msg.height = 1
        cloud_msg.width = len(points)
        
        # Define fields
        cloud_msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        
        cloud_msg.point_step = 12
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        
        # Pack points
        cloud_msg.data = b''
        for point in points:
            cloud_msg.data += struct.pack('fff', point[0], point[1], point[2])
        
        return cloud_msg
    
    def array_to_colored_pointcloud2(self, points, colors, header):
        """Convert numpy array with colors to PointCloud2 message"""
        cloud_msg = PointCloud2()
        cloud_msg.header = header
        
        cloud_msg.height = 1
        cloud_msg.width = len(points)
        
        # Define fields for colored point cloud
        cloud_msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="r", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="g", offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name="b", offset=20, datatype=PointField.FLOAT32, count=1)
        ]
        
        cloud_msg.point_step = 24
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        
        # Pack points with colors
        cloud_msg.data = b''
        for i, point in enumerate(points):
            color = colors[i] if i < len(colors) else [128, 128, 128]
            cloud_msg.data += struct.pack('ffffff', point[0], point[1], point[2], 
                                        color[0], color[1], color[2])
        
        return cloud_msg
    
    def color_point_cloud(self, points, image_msg, camera_info_msg):
        """Color point cloud with camera data"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except Exception as e:
            rospy.logerr("Error converting image: %s" % str(e))
            return
        
        if len(points) == 0:
            return
        
        # Create colored points array
        colors = np.full((len(points), 3), [128, 128, 128], dtype=np.uint8)  # Default gray
        
        debug_image = cv_image.copy()
        
        for i, point in enumerate(points):
            # Transform point from LiDAR to camera coordinates
            lidar_point = np.array([point[0], point[1], point[2]])
            camera_point = self.rotation_matrix @ lidar_point + self.translation_vector
            
            # Project to image plane
            if camera_point[2] > 0:  # Point is in front of camera
                image_point = self.camera_matrix @ camera_point
                u = image_point[0] / image_point[2]
                v = image_point[1] / image_point[2]
                
                # Check if point projects within image bounds
                if 0 <= u < cv_image.shape[1] and 0 <= v < cv_image.shape[0]:
                    color = cv_image[int(v), int(u)]
                    
                    colors[i] = [color[2], color[1], color[0]]  # BGR to RGB
                    
                    # Draw point on debug image
                    cv2.circle(debug_image, (int(u), int(v)), 2, (0, 255, 0), -1)
        
        # Create colored point cloud message
        colored_cloud_msg = self.array_to_colored_pointcloud2(points, colors, cloud_msg.header)
        self.colored_cloud_pub.publish(colored_cloud_msg)
        
        # Publish debug image
        try:
            debug_image_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            debug_image_msg.header = image_msg.header
            self.debug_image_pub.publish(debug_image_msg)
        except Exception as e:
            rospy.logerr("Error publishing debug image: %s" % str(e))
    
    def callback(self, cloud_msg, image_msg, camera_info_msg):
        """Main callback function"""
        try:
            # Filter point cloud
            filtered_points = self.filter_point_cloud(cloud_msg)
            
            # Publish filtered cloud
            filtered_cloud_msg = self.array_to_pointcloud2(filtered_points, cloud_msg.header)
            self.fused_cloud_pub.publish(filtered_cloud_msg)
            
            # Color point cloud
            self.color_point_cloud(filtered_points, image_msg, camera_info_msg)
            
            rospy.loginfo(f"Processed cloud with {len(filtered_points)} points")
            
        except Exception as e:
            rospy.logerr("Error in callback: %s" % str(e))

if __name__ == '__main__':
    try:
        fusion_node = CameraLidarFusionSimple()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
