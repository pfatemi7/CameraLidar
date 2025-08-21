#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from geometry_msgs.msg import TransformStamped
import tf2_ros
from cv_bridge import CvBridge
import message_filters
from message_filters import ApproximateTimeSynchronizer, Subscriber
import pcl
import pcl_helper

class CalibrationNode:
    def __init__(self):
        rospy.init_node('calibration_node', anonymous=True)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize TF
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Publishers
        self.debug_image_pub = rospy.Publisher('/calibration/debug_image', Image, queue_size=1)
        
        # Subscribers
        cloud_sub = Subscriber('/unilidar/cloud', PointCloud2)
        image_sub = Subscriber('/zed/left/image_rect_color', Image)
        camera_info_sub = Subscriber('/zed/left/camera_info', CameraInfo)
        
        # Synchronizer
        self.ts = ApproximateTimeSynchronizer([cloud_sub, image_sub, camera_info_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.callback)
        
        # Calibration parameters (these will be updated during calibration)
        self.camera_matrix = None
        self.dist_coeffs = None
        self.rotation_matrix = np.eye(3)
        self.translation_vector = np.array([0.1, 0, 0])
        
        # Calibration state
        self.calibration_points = []
        self.calibration_mode = False
        
        # UI parameters
        self.point_size = 5
        self.line_thickness = 2
        
        rospy.loginfo("Calibration node initialized")
        rospy.loginfo("Press 'c' to start calibration mode")
        rospy.loginfo("Press 's' to save calibration")
        rospy.loginfo("Press 'r' to reset calibration")
    
    def callback(self, cloud_msg, image_msg, camera_info_msg):
        """Main callback function for calibration"""
        try:
            # Get camera matrix from camera info
            if self.camera_matrix is None:
                self.camera_matrix = np.array(camera_info_msg.K).reshape(3, 3)
                self.dist_coeffs = np.array(camera_info_msg.D)
                rospy.loginfo("Camera matrix loaded from camera info")
            
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            
            # Process point cloud
            pcl_cloud = pcl_helper.ros_to_pcl(cloud_msg)
            points = np.array(pcl_cloud)
            
            # Create debug image
            debug_image = cv_image.copy()
            
            # Project points to image
            for point in points[:1000]:  # Limit to first 1000 points for performance
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
                        # Draw point on debug image
                        cv2.circle(debug_image, (int(u), int(v)), self.point_size, (0, 255, 0), -1)
            
            # Draw calibration instructions
            cv2.putText(debug_image, "Press 'c' to start calibration", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(debug_image, "Press 's' to save calibration", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(debug_image, "Press 'r' to reset calibration", (10, 90), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            if self.calibration_mode:
                cv2.putText(debug_image, "CALIBRATION MODE ACTIVE", (10, 120), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Publish debug image
            debug_image_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            debug_image_msg.header = image_msg.header
            self.debug_image_pub.publish(debug_image_msg)
            
            # Publish transform
            self.publish_transform()
            
        except Exception as e:
            rospy.logerr(f"Error in calibration callback: {e}")
    
    def publish_transform(self):
        """Publish the current calibration transform"""
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
    
    def save_calibration(self):
        """Save calibration parameters to file"""
        try:
            calibration_data = {
                'camera_matrix': self.camera_matrix.tolist(),
                'dist_coeffs': self.dist_coeffs.tolist(),
                'rotation_matrix': self.rotation_matrix.tolist(),
                'translation_vector': self.translation_vector.tolist()
            }
            
            import json
            with open('/tmp/camera_lidar_calibration.json', 'w') as f:
                json.dump(calibration_data, f, indent=2)
            
            rospy.loginfo("Calibration saved to /tmp/camera_lidar_calibration.json")
            
        except Exception as e:
            rospy.logerr(f"Error saving calibration: {e}")
    
    def load_calibration(self):
        """Load calibration parameters from file"""
        try:
            import json
            with open('/tmp/camera_lidar_calibration.json', 'r') as f:
                calibration_data = json.load(f)
            
            self.camera_matrix = np.array(calibration_data['camera_matrix'])
            self.dist_coeffs = np.array(calibration_data['dist_coeffs'])
            self.rotation_matrix = np.array(calibration_data['rotation_matrix'])
            self.translation_vector = np.array(calibration_data['translation_vector'])
            
            rospy.loginfo("Calibration loaded from file")
            
        except Exception as e:
            rospy.logwarn(f"Could not load calibration: {e}")

def keyboard_callback():
    """Handle keyboard input for calibration"""
    import sys
    import tty
    import termios
    
    def getch():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    
    while not rospy.is_shutdown():
        try:
            key = getch()
            if key == 'c':
                calibration_node.calibration_mode = not calibration_node.calibration_mode
                rospy.loginfo(f"Calibration mode: {'ON' if calibration_node.calibration_mode else 'OFF'}")
            elif key == 's':
                calibration_node.save_calibration()
            elif key == 'r':
                calibration_node.rotation_matrix = np.eye(3)
                calibration_node.translation_vector = np.array([0.1, 0, 0])
                rospy.loginfo("Calibration reset")
            elif key == 'q':
                break
        except KeyboardInterrupt:
            break

if __name__ == '__main__':
    try:
        calibration_node = CalibrationNode()
        
        # Start keyboard listener in a separate thread
        import threading
        keyboard_thread = threading.Thread(target=keyboard_callback)
        keyboard_thread.daemon = True
        keyboard_thread.start()
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
