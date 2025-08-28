#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, Quaternion
from std_msgs.msg import Header
import math

class L1TFPublisher:
    def __init__(self):
        rospy.init_node('l1_tf_publisher', anonymous=True)
        
        # Parameters
        self.l1_frame = rospy.get_param('~l1_frame', 'l1_link')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        
        # Position parameters (meters)
        self.x_m = rospy.get_param('~x_m', 0.0)
        self.y_m = rospy.get_param('~y_m', 0.0)
        self.z_m = rospy.get_param('~z_m', 0.0)
        
        # Orientation parameters (degrees)
        self.roll_deg = rospy.get_param('~roll_deg', 0.0)
        self.pitch_deg = rospy.get_param('~pitch_deg', 0.0)
        self.yaw_deg = rospy.get_param('~yaw_deg', 0.0)
        
        # Axis inversion parameters
        self.invert_x = rospy.get_param('~invert_x', False)
        self.invert_y = rospy.get_param('~invert_y', False)
        self.invert_z = rospy.get_param('~invert_z', False)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        
        # Publish transform
        self.publish_transform()
        
        rospy.loginfo(f"L1 TF Publisher initialized:")
        rospy.loginfo(f"  L1 frame: {self.l1_frame}")
        rospy.loginfo(f"  Base frame: {self.base_frame}")
        rospy.loginfo(f"  Position: ({self.x_m}, {self.y_m}, {self.z_m}) m")
        rospy.loginfo(f"  Orientation: ({self.roll_deg}, {self.pitch_deg}, {self.yaw_deg}) deg")
        rospy.loginfo(f"  Inversions: X={self.invert_x}, Y={self.invert_y}, Z={self.invert_z}")
        
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles (in radians) to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = Quaternion()
        q.w = cy * cp * cr + sy * sp * sr
        q.x = cy * cp * sr - sy * sp * cr
        q.y = sy * cp * sr + cy * sp * cr
        q.z = sy * cp * cr - cy * sp * sr
        
        return q
    
    def apply_axis_inversions(self, x, y, z, q):
        """Apply axis inversions to position and orientation"""
        # Apply position inversions
        if self.invert_x:
            x = -x
        if self.invert_y:
            y = -y
        if self.invert_z:
            z = -z
        
        # Apply orientation inversions (simplified - for complex cases, 
        # you might need to modify the quaternion directly)
        if self.invert_x:
            # Flip Y and Z components of quaternion
            q.y = -q.y
            q.z = -q.z
        if self.invert_y:
            # Flip X and Z components of quaternion
            q.x = -q.x
            q.z = -q.z
        if self.invert_z:
            # Flip X and Y components of quaternion
            q.x = -q.x
            q.y = -q.y
        
        return x, y, z, q
    
    def publish_transform(self):
        """Publish the static transform from base_link to l1_link"""
        transform = TransformStamped()
        
        # Header
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = self.base_frame
        transform.child_frame_id = self.l1_frame
        
        # Position
        transform.transform.translation.x = self.x_m
        transform.transform.translation.y = self.y_m
        transform.transform.translation.z = self.z_m
        
        # Orientation (convert degrees to radians)
        roll_rad = math.radians(self.roll_deg)
        pitch_rad = math.radians(self.pitch_deg)
        yaw_rad = math.radians(self.yaw_deg)
        
        quaternion = self.euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)
        
        # Apply axis inversions
        x, y, z, quaternion = self.apply_axis_inversions(
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
            quaternion
        )
        
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z
        transform.transform.rotation = quaternion
        
        # Publish transform
        self.tf_broadcaster.sendTransform(transform)
        
        rospy.loginfo(f"Published transform: {self.base_frame} -> {self.l1_frame}")
        rospy.loginfo(f"  Translation: ({x:.3f}, {y:.3f}, {z:.3f})")
        rospy.loginfo(f"  Rotation: ({quaternion.x:.3f}, {quaternion.y:.3f}, {quaternion.z:.3f}, {quaternion.w:.3f})")

def main():
    try:
        publisher = L1TFPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
