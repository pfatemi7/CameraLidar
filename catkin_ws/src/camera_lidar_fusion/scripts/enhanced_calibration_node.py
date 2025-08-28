#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Enhanced Calibration Node for Camera-LiDAR Fusion System
Implements automatic target detection, iterative refinement, and quality assessment.
"""

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from std_msgs.msg import String, Float32MultiArray, Bool
from cv_bridge import CvBridge
import struct
import json
import os
import time
from collections import deque
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray

class EnhancedCalibrationNode:
    def __init__(self):
        rospy.init_node('enhanced_calibration_node', anonymous=True)
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Publishers
        self.calibration_image_pub = rospy.Publisher('/calibration/debug_image', Image, queue_size=1)
        self.calibration_status_pub = rospy.Publisher('/calibration/status', String, queue_size=1)
        self.calibration_quality_pub = rospy.Publisher('/calibration/quality', Float32MultiArray, queue_size=1)
        self.calibration_complete_pub = rospy.Publisher('/calibration/complete', Bool, queue_size=1)
        self.target_poses_pub = rospy.Publisher('/calibration/target_poses', PoseArray, queue_size=1)
        self.visualization_pub = rospy.Publisher('/calibration/visualization', MarkerArray, queue_size=1)
        
        # Subscribers
        self.image_sub = rospy.Subscriber('/zed/left/image_raw', Image, self.image_callback)
        self.cloud_sub = rospy.Subscriber('/unilidar/cloud', PointCloud2, self.cloud_callback)
        self.camera_info_sub = rospy.Subscriber('/zed/left/camera_info', CameraInfo, self.camera_info_callback)
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Calibration parameters
        self.checkerboard_size = rospy.get_param('~checkerboard_size', (9, 6))  # (width, height)
        self.square_size = rospy.get_param('~square_size', 0.05)  # meters
        self.min_targets = rospy.get_param('~min_targets', 10)
        self.max_targets = rospy.get_param('~max_targets', 50)
        self.calibration_timeout = rospy.get_param('~calibration_timeout', 300)  # seconds
        
        # Target detection parameters
        self.min_corner_quality = rospy.get_param('~min_corner_quality', 0.1)
        self.max_corner_distance = rospy.get_param('~max_corner_distance', 0.1)  # meters
        self.target_stability_threshold = rospy.get_param('~target_stability_threshold', 0.02)  # meters
        
        # Calibration state
        self.calibration_active = False
        self.calibration_start_time = None
        self.target_poses = []
        self.camera_matrix = None
        self.dist_coeffs = None
        self.image_size = None
        
        # Target tracking
        self.current_target_pose = None
        self.target_stability_counter = 0
        self.target_stability_threshold_frames = 10
        
        # Quality metrics
        self.reprojection_errors = deque(maxlen=100)
        self.target_quality_scores = deque(maxlen=100)
        self.calibration_quality_score = 0.0
        
        # Multiple target support
        self.target_types = ['checkerboard', 'aruco', 'circle_grid']
        self.current_target_type = 'checkerboard'
        self.target_detectors = {
            'checkerboard': self.detect_checkerboard,
            'aruco': self.detect_aruco,
            'circle_grid': self.detect_circle_grid
        }
        
        # Aruco parameters
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        # Circle grid parameters
        self.circle_grid_size = rospy.get_param('~circle_grid_size', (4, 11))
        
        # Calibration file
        self.calibration_file = rospy.get_param('~calibration_file', 
                                               os.path.expanduser('~/camera_lidar_calibration.json'))
        
        # Load existing calibration if available
        self.load_calibration()
        
        # Service for starting/stopping calibration
        self.start_calibration_service = rospy.Service('/calibration/start', 
                                                      rospy.Service, self.start_calibration_callback)
        self.stop_calibration_service = rospy.Service('/calibration/stop', 
                                                     rospy.Service, self.stop_calibration_callback)
        
        rospy.loginfo("Enhanced Calibration Node initialized")
        rospy.loginfo("Checkerboard size: %dx%d", self.checkerboard_size[0], self.checkerboard_size[1])
        rospy.loginfo("Square size: %.3f m", self.square_size)
        rospy.loginfo("Min targets: %d", self.min_targets)
    
    def load_calibration(self):
        """Load existing calibration from file"""
        try:
            if os.path.exists(self.calibration_file):
                with open(self.calibration_file, 'r') as f:
                    calibration_data = json.load(f)
                
                self.camera_matrix = np.array(calibration_data['camera_matrix'])
                self.dist_coeffs = np.array(calibration_data['dist_coeffs'])
                self.target_poses = calibration_data.get('target_poses', [])
                
                rospy.loginfo("Loaded existing calibration from %s", self.calibration_file)
                return True
        except Exception as e:
            rospy.logwarn("Failed to load calibration: %s", str(e))
        
        return False
    
    def save_calibration(self):
        """Save calibration to file"""
        try:
            calibration_data = {
                'camera_matrix': self.camera_matrix.tolist(),
                'dist_coeffs': self.dist_coeffs.tolist(),
                'target_poses': self.target_poses,
                'timestamp': time.time(),
                'quality_score': self.calibration_quality_score
            }
            
            with open(self.calibration_file, 'w') as f:
                json.dump(calibration_data, f, indent=2)
            
            rospy.loginfo("Calibration saved to %s", self.calibration_file)
            return True
        except Exception as e:
            rospy.logerr("Failed to save calibration: %s", str(e))
            return False
    
    def detect_checkerboard(self, image):
        """Detect checkerboard pattern"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Find checkerboard corners
        ret, corners = cv2.findChessboardCorners(gray, self.checkerboard_size, None)
        
        if ret:
            # Refine corners
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            
            # Create object points
            objp = np.zeros((self.checkerboard_size[0] * self.checkerboard_size[1], 3), np.float32)
            objp[:, :2] = np.mgrid[0:self.checkerboard_size[0], 0:self.checkerboard_size[1]].T.reshape(-1, 2)
            objp *= self.square_size
            
            return True, corners, objp
        
        return False, None, None
    
    def detect_aruco(self, image):
        """Detect ArUco markers"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Detect ArUco markers
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        
        if ids is not None and len(ids) > 0:
            # Use the first detected marker
            marker_corners = corners[0][0]
            
            # Create object points for ArUco marker (assuming 6x6 marker)
            marker_size = 0.04  # 4cm marker
            objp = np.array([
                [0, 0, 0],
                [marker_size, 0, 0],
                [marker_size, marker_size, 0],
                [0, marker_size, 0]
            ], dtype=np.float32)
            
            return True, marker_corners.reshape(-1, 1, 2), objp
        
        return False, None, None
    
    def detect_circle_grid(self, image):
        """Detect circle grid pattern"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Find circle grid
        ret, centers = cv2.findCirclesGrid(gray, self.circle_grid_size, 
                                         flags=cv2.CALIB_CB_SYMMETRIC_GRID)
        
        if ret:
            # Create object points
            objp = np.zeros((self.circle_grid_size[0] * self.circle_grid_size[1], 3), np.float32)
            objp[:, :2] = np.mgrid[0:self.circle_grid_size[0], 0:self.circle_grid_size[1]].T.reshape(-1, 2)
            objp *= self.square_size
            
            return True, centers.reshape(-1, 1, 2), objp
        
        return False, None, None
    
    def estimate_target_pose(self, corners, objp):
        """Estimate target pose using PnP"""
        if self.camera_matrix is None or self.dist_coeffs is None:
            return None
        
        # Solve PnP
        ret, rvec, tvec = cv2.solvePnP(objp, corners, self.camera_matrix, self.dist_coeffs)
        
        if ret:
            # Convert rotation vector to rotation matrix
            rmat, _ = cv2.Rodrigues(rvec)
            
            # Create transformation matrix
            transform = np.eye(4)
            transform[:3, :3] = rmat
            transform[:3, 3] = tvec.flatten()
            
            return transform
        
        return None
    
    def calculate_reprojection_error(self, corners, objp, rvec, tvec):
        """Calculate reprojection error"""
        if self.camera_matrix is None or self.dist_coeffs is None:
            return float('inf')
        
        # Project object points
        projected_points, _ = cv2.projectPoints(objp, rvec, tvec, self.camera_matrix, self.dist_coeffs)
        
        # Calculate error
        error = cv2.norm(corners, projected_points, cv2.NORM_L2) / len(corners)
        
        return error
    
    def assess_target_quality(self, corners, objp, transform):
        """Assess quality of detected target"""
        if transform is None:
            return 0.0
        
        # Calculate reprojection error
        rvec, _ = cv2.Rodrigues(transform[:3, :3])
        tvec = transform[:3, 3].reshape(3, 1)
        reproj_error = self.calculate_reprojection_error(corners, objp, rvec, tvec)
        
        # Calculate target distance
        target_distance = np.linalg.norm(tvec)
        
        # Calculate corner quality (using corner response)
        gray = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2GRAY)
        corner_quality = cv2.cornerMinEigenVal(gray, 3)
        avg_corner_quality = np.mean(corner_quality)
        
        # Combine quality metrics
        quality_score = 1.0 / (1.0 + reproj_error) * np.exp(-target_distance / 5.0) * avg_corner_quality
        
        return quality_score
    
    def check_target_stability(self, new_pose):
        """Check if target pose is stable"""
        if self.current_target_pose is None:
            self.current_target_pose = new_pose
            self.target_stability_counter = 1
            return False
        
        # Calculate pose difference
        pose_diff = np.linalg.norm(new_pose[:3, 3] - self.current_target_pose[:3, 3])
        
        if pose_diff < self.target_stability_threshold:
            self.target_stability_counter += 1
        else:
            self.target_stability_counter = 0
            self.current_target_pose = new_pose
        
        return self.target_stability_counter >= self.target_stability_threshold_frames
    
    def add_target_pose(self, transform, corners, objp):
        """Add target pose to calibration data"""
        if len(self.target_poses) >= self.max_targets:
            # Remove oldest target
            self.target_poses.pop(0)
        
        target_data = {
            'transform': transform.tolist(),
            'corners': corners.tolist(),
            'objp': objp.tolist(),
            'timestamp': time.time(),
            'quality_score': self.assess_target_quality(corners, objp, transform)
        }
        
        self.target_poses.append(target_data)
        
        rospy.loginfo("Added target pose %d/%d (quality: %.3f)", 
                     len(self.target_poses), self.max_targets, target_data['quality_score'])
    
    def perform_calibration(self):
        """Perform camera calibration using collected target poses"""
        if len(self.target_poses) < self.min_targets:
            return False
        
        # Prepare calibration data
        object_points = []
        image_points = []
        
        for target in self.target_poses:
            object_points.append(target['objp'])
            image_points.append(np.array(target['corners']).reshape(-1, 2))
        
        # Perform calibration
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            object_points, image_points, self.image_size, None, None
        )
        
        if ret:
            self.camera_matrix = camera_matrix
            self.dist_coeffs = dist_coeffs
            
            # Calculate quality metrics
            self.calculate_calibration_quality(rvecs, tvecs, object_points, image_points)
            
            # Save calibration
            self.save_calibration()
            
            rospy.loginfo("Calibration completed successfully!")
            rospy.loginfo("Quality score: %.3f", self.calibration_quality_score)
            
            return True
        
        return False
    
    def calculate_calibration_quality(self, rvecs, tvecs, object_points, image_points):
        """Calculate overall calibration quality"""
        total_error = 0
        total_points = 0
        
        for i in range(len(object_points)):
            projected_points, _ = cv2.projectPoints(
                object_points[i], rvecs[i], tvecs[i], 
                self.camera_matrix, self.dist_coeffs
            )
            
            error = cv2.norm(image_points[i], projected_points, cv2.NORM_L2) / len(projected_points)
            total_error += error
            total_points += 1
        
        mean_error = total_error / total_points
        self.calibration_quality_score = 1.0 / (1.0 + mean_error)
        
        # Store quality metrics
        self.reprojection_errors.append(mean_error)
        self.target_quality_scores.append(np.mean([t['quality_score'] for t in self.target_poses]))
    
    def publish_calibration_status(self):
        """Publish calibration status"""
        if self.calibration_active:
            elapsed_time = time.time() - self.calibration_start_time
            remaining_time = max(0, self.calibration_timeout - elapsed_time)
            
            status_msg = String()
            status_msg.data = "Calibrating: %d/%d targets, %.1fs remaining" % (
                len(self.target_poses), self.min_targets, remaining_time
            )
            self.calibration_status_pub.publish(status_msg)
            
            # Publish quality metrics
            quality_msg = Float32MultiArray()
            quality_msg.data = [
                float(len(self.target_poses)),
                float(self.min_targets),
                float(self.calibration_quality_score),
                float(remaining_time)
            ]
            self.calibration_quality_pub.publish(quality_msg)
    
    def publish_target_visualization(self):
        """Publish target visualization markers"""
        marker_array = MarkerArray()
        
        for i, target in enumerate(self.target_poses):
            transform = np.array(target['transform'])
            
            # Create marker for target pose
            marker = Marker()
            marker.header.frame_id = "camera_link"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "calibration_targets"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # Set pose
            marker.pose.position.x = transform[0, 3]
            marker.pose.position.y = transform[1, 3]
            marker.pose.position.z = transform[2, 3]
            
            # Convert rotation matrix to quaternion
            rmat = transform[:3, :3]
            # This is a simplified conversion - in practice you'd use proper quaternion conversion
            marker.pose.orientation.w = 1.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            
            # Set scale and color based on quality
            quality = target['quality_score']
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            
            marker.color.r = 1.0 - quality
            marker.color.g = quality
            marker.color.b = 0.0
            marker.color.a = 0.8
            
            marker_array.markers.append(marker)
        
        self.visualization_pub.publish(marker_array)
    
    def image_callback(self, image_msg):
        """Callback for camera images"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            self.latest_image = cv_image
            
            if self.calibration_active:
                # Detect target
                detector = self.target_detectors[self.current_target_type]
                detected, corners, objp = detector(cv_image)
                
                debug_image = cv_image.copy()
                
                if detected:
                    # Draw detected corners
                    cv2.drawChessboardCorners(debug_image, self.checkerboard_size, corners, detected)
                    
                    # Estimate pose
                    transform = self.estimate_target_pose(corners, objp)
                    
                    if transform is not None:
                        # Check stability
                        if self.check_target_stability(transform):
                            # Add target pose
                            self.add_target_pose(transform, corners, objp)
                            
                            # Draw success indicator
                            cv2.putText(debug_image, "TARGET STABLE - ADDED", (10, 30),
                                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        else:
                            # Draw stability indicator
                            cv2.putText(debug_image, "TARGET DETECTED - STABILIZING", (10, 30),
                                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                else:
                    cv2.putText(debug_image, "NO TARGET DETECTED", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                
                # Add calibration info
                cv2.putText(debug_image, "Targets: %d/%d" % (len(self.target_poses), self.min_targets), 
                           (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                if self.calibration_quality_score > 0:
                    cv2.putText(debug_image, "Quality: %.3f" % self.calibration_quality_score, 
                               (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                # Convert back to ROS image
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
                debug_msg.header = image_msg.header
                self.calibration_image_pub.publish(debug_msg)
                
                # Publish status
                self.publish_calibration_status()
                
                # Publish visualization
                self.publish_target_visualization()
        
        except Exception as e:
            rospy.logerr("Error processing image: %s", str(e))
    
    def cloud_callback(self, cloud_msg):
        """Callback for point cloud messages"""
        # For now, just store the latest cloud
        self.latest_cloud = cloud_msg
    
    def camera_info_callback(self, camera_info_msg):
        """Callback for camera info"""
        if self.camera_matrix is None:
            # Initialize camera matrix from camera info
            self.camera_matrix = np.array(camera_info_msg.K).reshape(3, 3)
            self.dist_coeffs = np.array(camera_info_msg.D)
            self.image_size = (camera_info_msg.width, camera_info_msg.height)
            
            rospy.loginfo("Initialized camera matrix from camera info")
    
    def start_calibration_callback(self, req):
        """Service callback to start calibration"""
        if not self.calibration_active:
            self.calibration_active = True
            self.calibration_start_time = time.time()
            self.target_poses = []
            self.current_target_pose = None
            self.target_stability_counter = 0
            
            rospy.loginfo("Calibration started")
            return True
        return False
    
    def stop_calibration_callback(self, req):
        """Service callback to stop calibration"""
        if self.calibration_active:
            self.calibration_active = False
            
            # Perform final calibration
            success = self.perform_calibration()
            
            # Publish completion status
            complete_msg = Bool()
            complete_msg.data = success
            self.calibration_complete_pub.publish(complete_msg)
            
            rospy.loginfo("Calibration stopped - Success: %s", success)
            return True
        return False

if __name__ == '__main__':
    try:
        node = EnhancedCalibrationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
