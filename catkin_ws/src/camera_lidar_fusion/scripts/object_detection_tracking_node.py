#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Object Detection and Tracking Node for Camera-LiDAR Fusion System
Integrates YOLO-like detection with 3D point cloud projection and tracking.
"""

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo
from std_msgs.msg import String, Float32MultiArray, Header
from cv_bridge import CvBridge
import struct
import time
from collections import deque, defaultdict
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, PoseArray, Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
import os

class ObjectDetectionTrackingNode:
    def __init__(self):
        rospy.init_node('object_detection_tracking_node', anonymous=True)
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Publishers
        self.detection_image_pub = rospy.Publisher('/detection/debug_image', Image, queue_size=1)
        self.object_cloud_pub = rospy.Publisher('/detection/object_cloud', PointCloud2, queue_size=1)
        self.object_poses_pub = rospy.Publisher('/detection/object_poses', PoseArray, queue_size=1)
        self.object_markers_pub = rospy.Publisher('/detection/object_markers', MarkerArray, queue_size=1)
        self.detection_stats_pub = rospy.Publisher('/detection/stats', Float32MultiArray, queue_size=1)
        
        # Subscribers
        self.image_sub = rospy.Subscriber('/zed/left/image_raw', Image, self.image_callback)
        self.cloud_sub = rospy.Subscriber('/fused/enhanced_cloud', PointCloud2, self.cloud_callback)
        self.camera_info_sub = rospy.Subscriber('/zed/left/camera_info', CameraInfo, self.camera_info_callback)
        
        # Optional: Subscribe to external YOLO detections
        self.yolo_detections_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', 
                                                   BoundingBoxes, self.yolo_detections_callback)
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Camera parameters
        self.camera_matrix = None
        self.dist_coeffs = None
        self.image_size = None
        
        # Object detection parameters
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.5)
        self.nms_threshold = rospy.get_param('~nms_threshold', 0.4)
        self.min_object_size = rospy.get_param('~min_object_size', 50)  # pixels
        self.max_object_size = rospy.get_param('~max_object_size', 1000)  # pixels
        
        # Object tracking parameters
        self.tracking_max_distance", 10.0)  # meters
        self.tracking_max_age = rospy.get_param('~tracking_max_age', 30)  # frames
        self.tracking_min_hits = rospy.get_param('~tracking_min_hits', 3)  # frames
        
        # 3D projection parameters
        self.projection_cone_angle = rospy.get_param('~projection_cone_angle', 15.0)  # degrees
        self.min_projection_points = rospy.get_param('~min_projection_points', 10)
        self.max_projection_distance = rospy.get_param('~max_projection_distance', 20.0)  # meters
        
        # Object classes of interest
        self.target_classes = rospy.get_param('~target_classes', 
                                            ['person', 'car', 'truck', 'bicycle', 'motorcycle'])
        
        # Detection and tracking state
        self.latest_image = None
        self.latest_cloud = None
        self.latest_camera_info = None
        self.detected_objects = []
        self.tracked_objects = {}
        self.next_track_id = 0
        self.frame_count = 0
        
        # Performance monitoring
        self.processing_times = deque(maxlen=100)
        self.detection_counts = deque(maxlen=100)
        
        # Load YOLO model (if available)
        self.yolo_net = None
        self.load_yolo_model()
        
        rospy.loginfo("Object Detection and Tracking Node initialized")
        rospy.loginfo("Target classes: %s", self.target_classes)
        rospy.loginfo("Confidence threshold: %.2f", self.confidence_threshold)
    
    def load_yolo_model(self):
        """Load YOLO model for object detection"""
        try:
            # Try to load YOLO model from common locations
            model_paths = [
                '/opt/ros/melodic/share/darknet_ros/yolo_network_config/weights/yolov3.weights',
                '/usr/local/share/darknet/yolov3.weights',
                os.path.expanduser('~/yolo_weights/yolov3.weights')
            ]
            
            config_paths = [
                '/opt/ros/melodic/share/darknet_ros/yolo_network_config/cfg/yolov3.cfg',
                '/usr/local/share/darknet/yolov3.cfg',
                os.path.expanduser('~/yolo_weights/yolov3.cfg')
            ]
            
            for model_path, config_path in zip(model_paths, config_paths):
                if os.path.exists(model_path) and os.path.exists(config_path):
                    self.yolo_net = cv2.dnn.readNetFromDarknet(config_path, model_path)
                    rospy.loginfo("Loaded YOLO model from %s", model_path)
                    return
            
            rospy.logwarn("YOLO model not found, using basic motion detection")
            
        except Exception as e:
            rospy.logwarn("Failed to load YOLO model: %s", str(e))
    
    def detect_objects_yolo(self, image):
        """Detect objects using YOLO"""
        if self.yolo_net is None:
            return self.detect_objects_motion(image)
        
        try:
            # Prepare image for YOLO
            blob = cv2.dnn.blobFromImage(image, 1/255.0, (416, 416), swapRB=True, crop=False)
            self.yolo_net.setInput(blob)
            
            # Get output layers
            layer_names = self.yolo_net.getLayerNames()
            output_layers = [layer_names[i[0] - 1] for i in self.yolo_net.getUnconnectedOutLayers()]
            
            # Forward pass
            outputs = self.yolo_net.forward(output_layers)
            
            # Process detections
            detections = []
            height, width = image.shape[:2]
            
            for output in outputs:
                for detection in output:
                    scores = detection[5:]
                    class_id = np.argmax(scores)
                    confidence = scores[class_id]
                    
                    if confidence > self.confidence_threshold:
                        # Get bounding box coordinates
                        center_x = int(detection[0] * width)
                        center_y = int(detection[1] * height)
                        w = int(detection[2] * width)
                        h = int(detection[3] * height)
                        
                        # Calculate top-left corner
                        x = int(center_x - w / 2)
                        y = int(center_y - h / 2)
                        
                        # Get class name
                        class_name = self.get_class_name(class_id)
                        
                        if class_name in self.target_classes:
                            detections.append({
                                'bbox': (x, y, w, h),
                                'class': class_name,
                                'confidence': float(confidence),
                                'center': (center_x, center_y)
                            })
            
            # Apply non-maximum suppression
            detections = self.apply_nms(detections)
            
            return detections
            
        except Exception as e:
            rospy.logwarn("YOLO detection failed: %s", str(e))
            return self.detect_objects_motion(image)
    
    def detect_objects_motion(self, image):
        """Fallback motion-based object detection"""
        if self.latest_image is None:
            self.latest_image = image
            return []
        
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        prev_gray = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2GRAY)
        
        # Calculate optical flow
        flow = cv2.calcOpticalFlowFarneback(prev_gray, gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)
        
        # Create motion mask
        magnitude, angle = cv2.cartToPolar(flow[..., 0], flow[..., 1])
        motion_mask = magnitude > 2.0
        
        # Find contours in motion mask
        contours, _ = cv2.findContours(motion_mask.astype(np.uint8), 
                                     cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detections = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if self.min_object_size < area < self.max_object_size:
                x, y, w, h = cv2.boundingRect(contour)
                detections.append({
                    'bbox': (x, y, w, h),
                    'class': 'motion',
                    'confidence': 0.7,
                    'center': (x + w//2, y + h//2)
                })
        
        self.latest_image = image
        return detections
    
    def get_class_name(self, class_id):
        """Get class name from YOLO class ID"""
        # COCO dataset classes (YOLO default)
        coco_classes = [
            'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat',
            'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat',
            'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack',
            'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
            'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
            'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
            'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake',
            'chair', 'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop',
            'mouse', 'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink',
            'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
        ]
        
        if 0 <= class_id < len(coco_classes):
            return coco_classes[class_id]
        return 'unknown'
    
    def apply_nms(self, detections):
        """Apply non-maximum suppression"""
        if not detections:
            return []
        
        # Convert to numpy arrays for processing
        boxes = np.array([d['bbox'] for d in detections])
        scores = np.array([d['confidence'] for d in detections])
        
        # Apply NMS
        indices = cv2.dnn.NMSBoxes(boxes, scores, self.confidence_threshold, self.nms_threshold)
        
        if len(indices) > 0:
            indices = indices.flatten()
            return [detections[i] for i in indices]
        
        return []
    
    def project_detection_to_3d(self, detection, point_cloud):
        """Project 2D detection to 3D point cloud"""
        if self.camera_matrix is None or point_cloud is None:
            return None
        
        x, y, w, h = detection['bbox']
        center_x, center_y = detection['center']
        
        # Convert point cloud to numpy array
        points = self.pointcloud2_to_array(point_cloud)
        
        if len(points) == 0:
            return None
        
        # Project points to image plane
        projected_points = []
        valid_points = []
        
        for point in points:
            x_3d, y_3d, z_3d = point[:3]
            
            # Skip points behind camera
            if z_3d <= 0:
                continue
            
            # Project 3D point to 2D
            point_3d = np.array([x_3d, y_3d, z_3d])
            point_2d = self.camera_matrix @ point_3d
            u, v = point_2d[0] / point_2d[2], point_2d[1] / point_2d[2]
            
            # Check if point projects within detection bounding box
            if (x <= u <= x + w and y <= v <= y + h):
                # Check distance
                distance = np.linalg.norm(point_3d)
                if distance <= self.max_projection_distance:
                    projected_points.append(point_3d)
                    valid_points.append(point)
        
        if len(projected_points) < self.min_projection_points:
            return None
        
        # Calculate 3D bounding box
        projected_points = np.array(projected_points)
        
        # Calculate centroid
        centroid = np.mean(projected_points, axis=0)
        
        # Calculate bounding box dimensions
        min_coords = np.min(projected_points, axis=0)
        max_coords = np.max(projected_points, axis=0)
        dimensions = max_coords - min_coords
        
        # Create 3D object representation
        object_3d = {
            'detection': detection,
            'centroid': centroid,
            'dimensions': dimensions,
            'points': valid_points,
            'confidence': detection['confidence'],
            'class': detection['class']
        }
        
        return object_3d
    
    def pointcloud2_to_array(self, cloud_msg):
        """Convert ROS PointCloud2 to numpy array"""
        points_list = []
        
        # Get point step and field offsets
        point_step = cloud_msg.point_step
        x_offset = None
        y_offset = None
        z_offset = None
        
        for field in cloud_msg.fields:
            if field.name == 'x':
                x_offset = field.offset
            elif field.name == 'y':
                y_offset = field.offset
            elif field.name == 'z':
                z_offset = field.offset
        
        if x_offset is None or y_offset is None or z_offset is None:
            return np.array([])
        
        # Extract points
        for i in range(0, len(cloud_msg.data), point_step):
            if i + point_step <= len(cloud_msg.data):
                point_data = cloud_msg.data[i:i+point_step]
                
                x = struct.unpack('f', point_data[x_offset:x_offset+4])[0]
                y = struct.unpack('f', point_data[y_offset:y_offset+4])[0]
                z = struct.unpack('f', point_data[z_offset:z_offset+4])[0]
                
                points_list.append([x, y, z])
        
        return np.array(points_list)
    
    def track_objects(self, detected_objects_3d):
        """Track detected objects across frames"""
        current_tracks = {}
        
        for obj_3d in detected_objects_3d:
            centroid = obj_3d['centroid']
            best_match_id = None
            best_distance = float('inf')
            
            # Find best matching existing track
            for track_id, track in self.tracked_objects.items():
                if track['age'] < self.tracking_max_age:
                    distance = np.linalg.norm(centroid - track['centroid'])
                    if distance < self.tracking_max_distance and distance < best_distance:
                        best_distance = distance
                        best_match_id = track_id
            
            if best_match_id is not None:
                # Update existing track
                track = self.tracked_objects[best_match_id]
                track['centroid'] = centroid
                track['detection'] = obj_3d['detection']
                track['dimensions'] = obj_3d['dimensions']
                track['points'] = obj_3d['points']
                track['age'] = 0
                track['hits'] += 1
                track['class'] = obj_3d['class']
                track['confidence'] = obj_3d['confidence']
                
                current_tracks[best_match_id] = track
            else:
                # Create new track
                track_id = self.next_track_id
                self.next_track_id += 1
                
                new_track = {
                    'id': track_id,
                    'centroid': centroid,
                    'detection': obj_3d['detection'],
                    'dimensions': obj_3d['dimensions'],
                    'points': obj_3d['points'],
                    'age': 0,
                    'hits': 1,
                    'class': obj_3d['class'],
                    'confidence': obj_3d['confidence']
                }
                
                current_tracks[track_id] = new_track
        
        # Age existing tracks
        for track_id, track in self.tracked_objects.items():
            if track_id not in current_tracks:
                track['age'] += 1
                if track['age'] < self.tracking_max_age:
                    current_tracks[track_id] = track
        
        # Update tracked objects
        self.tracked_objects = current_tracks
    
    def create_3d_bounding_box(self, track):
        """Create 3D bounding box for tracked object"""
        centroid = track['centroid']
        dimensions = track['dimensions']
        
        # Create bounding box corners
        half_dims = dimensions / 2
        corners = np.array([
            [centroid[0] - half_dims[0], centroid[1] - half_dims[1], centroid[2] - half_dims[2]],
            [centroid[0] + half_dims[0], centroid[1] - half_dims[1], centroid[2] - half_dims[2]],
            [centroid[0] + half_dims[0], centroid[1] + half_dims[1], centroid[2] - half_dims[2]],
            [centroid[0] - half_dims[0], centroid[1] + half_dims[1], centroid[2] - half_dims[2]],
            [centroid[0] - half_dims[0], centroid[1] - half_dims[1], centroid[2] + half_dims[2]],
            [centroid[0] + half_dims[0], centroid[1] - half_dims[1], centroid[2] + half_dims[2]],
            [centroid[0] + half_dims[0], centroid[1] + half_dims[1], centroid[2] + half_dims[2]],
            [centroid[0] - half_dims[0], centroid[1] + half_dims[1], centroid[2] + half_dims[2]]
        ])
        
        return corners
    
    def publish_object_visualization(self):
        """Publish object visualization markers"""
        marker_array = MarkerArray()
        
        for track_id, track in self.tracked_objects.items():
            if track['hits'] >= self.tracking_min_hits:
                # Create bounding box marker
                marker = Marker()
                marker.header.frame_id = "camera_link"
                marker.header.stamp = rospy.Time.now()
                marker.ns = "detected_objects"
                marker.id = track_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                
                # Set pose
                marker.pose.position.x = track['centroid'][0]
                marker.pose.position.y = track['centroid'][1]
                marker.pose.position.z = track['centroid'][2]
                marker.pose.orientation.w = 1.0
                
                # Set scale
                marker.scale.x = track['dimensions'][0]
                marker.scale.y = track['dimensions'][1]
                marker.scale.z = track['dimensions'][2]
                
                # Set color based on class
                class_colors = {
                    'person': (1.0, 0.0, 0.0),  # Red
                    'car': (0.0, 1.0, 0.0),     # Green
                    'truck': (0.0, 0.0, 1.0),   # Blue
                    'bicycle': (1.0, 1.0, 0.0), # Yellow
                    'motorcycle': (1.0, 0.0, 1.0) # Magenta
                }
                
                color = class_colors.get(track['class'], (0.5, 0.5, 0.5))
                marker.color.r = color[0]
                marker.color.g = color[1]
                marker.color.b = color[2]
                marker.color.a = 0.7
                
                marker_array.markers.append(marker)
                
                # Create text marker for label
                text_marker = Marker()
                text_marker.header.frame_id = "camera_link"
                text_marker.header.stamp = rospy.Time.now()
                text_marker.ns = "object_labels"
                text_marker.id = track_id + 1000
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                
                text_marker.pose.position.x = track['centroid'][0]
                text_marker.pose.position.y = track['centroid'][1]
                text_marker.pose.position.z = track['centroid'][2] + track['dimensions'][2] / 2 + 0.1
                text_marker.pose.orientation.w = 1.0
                
                text_marker.scale.z = 0.1
                text_marker.color.r = 1.0
                text_marker.color.g = 1.0
                text_marker.color.b = 1.0
                text_marker.color.a = 1.0
                
                distance = np.linalg.norm(track['centroid'])
                text_marker.text = f"{track['class']} {track_id}\n{distance:.1f}m"
                
                marker_array.markers.append(text_marker)
        
        self.object_markers_pub.publish(marker_array)
    
    def image_callback(self, image_msg):
        """Callback for camera images"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            self.latest_image = cv_image
            
            # Detect objects
            detections = self.detect_objects_yolo(cv_image)
            
            # Project detections to 3D if point cloud is available
            detected_objects_3d = []
            if self.latest_cloud is not None:
                for detection in detections:
                    object_3d = self.project_detection_to_3d(detection, self.latest_cloud)
                    if object_3d is not None:
                        detected_objects_3d.append(object_3d)
            
            # Track objects
            self.track_objects(detected_objects_3d)
            
            # Create debug image
            debug_image = cv_image.copy()
            
            # Draw 2D detections
            for detection in detections:
                x, y, w, h = detection['bbox']
                cv2.rectangle(debug_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(debug_image, f"{detection['class']} {detection['confidence']:.2f}", 
                           (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Add tracking info
            active_tracks = sum(1 for track in self.tracked_objects.values() 
                              if track['hits'] >= self.tracking_min_hits)
            cv2.putText(debug_image, f"Active Tracks: {active_tracks}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(debug_image, f"Detections: {len(detections)}", 
                       (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            
            # Convert back to ROS image
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            debug_msg.header = image_msg.header
            self.detection_image_pub.publish(debug_msg)
            
            # Publish visualization
            self.publish_object_visualization()
            
            # Update statistics
            self.detection_counts.append(len(detections))
            self.frame_count += 1
            
            # Log statistics periodically
            if self.frame_count % 30 == 0:
                avg_detections = np.mean(self.detection_counts)
                rospy.loginfo("Object Detection: Frame %d, Avg detections: %.1f, Active tracks: %d", 
                             self.frame_count, avg_detections, active_tracks)
        
        except Exception as e:
            rospy.logerr("Error in object detection: %s", str(e))
    
    def cloud_callback(self, cloud_msg):
        """Callback for point cloud messages"""
        self.latest_cloud = cloud_msg
    
    def camera_info_callback(self, camera_info_msg):
        """Callback for camera info"""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(camera_info_msg.K).reshape(3, 3)
            self.dist_coeffs = np.array(camera_info_msg.D)
            self.image_size = (camera_info_msg.width, camera_info_msg.height)
            
            rospy.loginfo("Initialized camera matrix from camera info")
    
    def yolo_detections_callback(self, detections_msg):
        """Callback for external YOLO detections"""
        # This can be used if you have a separate YOLO node running
        # For now, we'll use our internal detection
        pass

if __name__ == '__main__':
    try:
        node = ObjectDetectionTrackingNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
