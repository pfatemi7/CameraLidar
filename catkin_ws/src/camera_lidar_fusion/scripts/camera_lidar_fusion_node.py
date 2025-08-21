#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf2_geometry_msgs
from cv_bridge import CvBridge
import message_filters
from message_filters import ApproximateTimeSynchronizer, Subscriber
import pcl
import pcl_helper

class CameraLidarFusionPython:
    def __init__(self):
        rospy.init_node('camera_lidar_fusion_python', anonymous=True)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
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
        self.sor_mean_k = rospy.get_param('~sor_mean_k', 20)
        self.sor_std_dev = rospy.get_param('~sor_std_dev', 1.0)
        
        # Publish static transform
        self.publish_static_transform()
        
        rospy.loginfo("Camera-LiDAR fusion Python node initialized")
    
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
    
    def filter_point_cloud(self, cloud):
        """Apply filters to point cloud"""
        # Convert to PCL format
        pcl_cloud = pcl_helper.ros_to_pcl(cloud)
        
        # Voxel downsampling
        voxel_filter = pcl_cloud.make_voxel_grid_filter()
        voxel_filter.set_leaf_size(self.voxel_leaf_size, self.voxel_leaf_size, self.voxel_leaf_size)
        filtered_cloud = voxel_filter.filter()
        
        # Z-axis filtering
        pass_filter = filtered_cloud.make_passthrough_filter()
        pass_filter.set_filter_field_name('z')
        pass_filter.set_filter_limits(self.z_min, self.z_max)
        filtered_cloud = pass_filter.filter()
        
        # Statistical outlier removal
        sor_filter = filtered_cloud.make_statistical_outlier_filter()
        sor_filter.set_mean_k(self.sor_mean_k)
        sor_filter.set_std_dev_mul_thresh(self.sor_std_dev)
        filtered_cloud = sor_filter.filter()
        
        return filtered_cloud
    
    def color_point_cloud(self, cloud, image_msg, camera_info_msg):
        """Color point cloud with camera data"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")
            return
        
        # Convert PCL cloud to numpy array
        cloud_array = pcl_helper.pcl_to_ros(cloud)
        points = np.array(cloud_array.data).reshape(-1, 3)
        
        # Create colored points array
        colored_points = np.zeros((len(points), 6))  # x, y, z, r, g, b
        colored_points[:, :3] = points
        
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
                    
                    colored_points[i, 3] = color[2]  # R
                    colored_points[i, 4] = color[1]  # G
                    colored_points[i, 5] = color[0]  # B
                    
                    # Draw point on debug image
                    cv2.circle(debug_image, (int(u), int(v)), 2, (0, 255, 0), -1)
                else:
                    # Point outside image - use default color
                    colored_points[i, 3:6] = [128, 128, 128]
            else:
                # Point behind camera - use default color
                colored_points[i, 3:6] = [128, 128, 128]
        
        # Create colored point cloud message
        colored_cloud_msg = pcl_helper.array_to_pointcloud2(colored_points, cloud.header)
        colored_cloud_msg.header = cloud.header
        self.colored_cloud_pub.publish(colored_cloud_msg)
        
        # Publish debug image
        try:
            debug_image_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            debug_image_msg.header = image_msg.header
            self.debug_image_pub.publish(debug_image_msg)
        except Exception as e:
            rospy.logerr(f"Error publishing debug image: {e}")
    
    def callback(self, cloud_msg, image_msg, camera_info_msg):
        """Main callback function"""
        try:
            # Filter point cloud
            filtered_cloud = self.filter_point_cloud(cloud_msg)
            
            # Publish filtered cloud
            filtered_cloud_msg = pcl_helper.pcl_to_ros(filtered_cloud)
            filtered_cloud_msg.header = cloud_msg.header
            self.fused_cloud_pub.publish(filtered_cloud_msg)
            
            # Color point cloud
            self.color_point_cloud(filtered_cloud, image_msg, camera_info_msg)
            
            rospy.loginfo(f"Processed cloud with {len(filtered_cloud)} points")
            
        except Exception as e:
            rospy.logerr(f"Error in callback: {e}")

if __name__ == '__main__':
    try:
        fusion_node = CameraLidarFusionPython()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
