#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
import numpy as np
from sensor_msgs.msg import Range, PointCloud2
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32
import struct

class CameraLidarMAVROSNode:
    def __init__(self):
        rospy.init_node('camera_lidar_mavros_node', anonymous=True)
        
        # Publishers
        self.rangefinder_pub = rospy.Publisher('/mavros/rangefinder/rangefinder', Range, queue_size=1)
        self.velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)
        
        # Subscribers
        self.cloud_sub = rospy.Subscriber('/fused/cloud', PointCloud2, self.cloud_callback)
        self.distance_sub = rospy.Subscriber('/object_detection/distance', Float32, self.distance_callback)
        
        # Parameters
        self.safety_distance = rospy.get_param('~safety_distance', 5.0)  # meters
        self.max_velocity = rospy.get_param('~max_velocity', 2.0)  # m/s
        self.enable_velocity_control = rospy.get_param('~enable_velocity_control', True)
        
        # Range message
        self.range_msg = Range()
        self.range_msg.header.frame_id = "base_link"
        self.range_msg.radiation_type = Range.ULTRASOUND
        self.range_msg.field_of_view = math.radians(30.0)
        self.range_msg.min_range = 0.1
        self.range_msg.max_range = 30.0
        
        # Velocity message
        self.velocity_msg = TwistStamped()
        self.velocity_msg.header.frame_id = "base_link"
        
        # State variables
        self.latest_cloud = None
        self.latest_distance = None
        self.closest_obstacle_distance = float('inf')
        
        # Publish rate
        self.publish_rate = rospy.Rate(10)  # 10 Hz
        
        rospy.loginfo("Camera-LiDAR MAVROS Node initialized")
        rospy.loginfo("Safety distance: %.1f m", self.safety_distance)
        rospy.loginfo("Max velocity: %.1f m/s", self.max_velocity)
    
    def cloud_callback(self, cloud_msg):
        """Process point cloud to find closest obstacle"""
        self.latest_cloud = cloud_msg
        self.process_point_cloud()
    
    def distance_callback(self, distance_msg):
        """Process object detection distance"""
        self.latest_distance = distance_msg.data
        rospy.loginfo("Object detected at %.2f meters", self.latest_distance)
    
    def process_point_cloud(self):
        """Extract closest obstacle distance from point cloud"""
        if self.latest_cloud is None:
            return
        
        try:
            # Extract point cloud data
            point_step = self.latest_cloud.point_step
            point_fields = self.latest_cloud.fields
            
            # Find the offset for x, y, z fields
            x_offset = None
            y_offset = None
            z_offset = None
            
            for field in point_fields:
                if field.name == 'x':
                    x_offset = field.offset
                elif field.name == 'y':
                    y_offset = field.offset
                elif field.name == 'z':
                    z_offset = field.offset
            
            if x_offset is None or y_offset is None or z_offset is None:
                return
            
            # Find closest point in front of the vehicle
            closest_distance = float('inf')
            
            for i in range(0, len(self.latest_cloud.data), point_step):
                if i + point_step <= len(self.latest_cloud.data):
                    point_data = self.latest_cloud.data[i:i+point_step]
                    
                    # Extract x, y, z values
                    x = struct.unpack('f', point_data[x_offset:x_offset+4])[0]
                    y = struct.unpack('f', point_data[y_offset:y_offset+4])[0]
                    z = struct.unpack('f', point_data[z_offset:z_offset+4])[0]
                    
                    # Calculate distance
                    distance = math.sqrt(x*x + y*y + z*z)
                    
                    # Only consider points in front (positive x) and within reasonable range
                    if x > 0 and 0.1 < distance < 30.0:
                        # Check if point is within the sensor FOV (30 degrees)
                        angle = math.degrees(math.atan2(y, x))
                        if abs(angle) < 15:  # 30 degree FOV
                            if distance < closest_distance:
                                closest_distance = distance
            
            if closest_distance < float('inf'):
                self.closest_obstacle_distance = closest_distance
            else:
                self.closest_obstacle_distance = 30.0  # No obstacle detected
            
        except Exception as e:
            rospy.logerr("Error processing point cloud: %s", str(e))
    
    def publish_data(self):
        """Publish rangefinder and velocity data"""
        try:
            # Publish rangefinder data
            self.range_msg.header.stamp = rospy.Time.now()
            self.range_msg.range = self.closest_obstacle_distance
            self.rangefinder_pub.publish(self.range_msg)
            
            # Publish velocity control if enabled
            if self.enable_velocity_control:
                self.velocity_msg.header.stamp = rospy.Time.now()
                
                # Adjust velocity based on obstacle distance
                if self.closest_obstacle_distance < self.safety_distance:
                    # Obstacle detected - slow down
                    velocity_factor = self.closest_obstacle_distance / self.safety_distance
                    velocity = self.max_velocity * max(0.1, velocity_factor)
                    rospy.logwarn("Obstacle at %.2f m - reducing velocity to %.1f m/s", 
                                 self.closest_obstacle_distance, velocity)
                else:
                    # No obstacle - normal speed
                    velocity = self.max_velocity
                
                self.velocity_msg.twist.linear.x = velocity
                self.velocity_msg.twist.linear.y = 0.0
                self.velocity_msg.twist.linear.z = 0.0
                self.velocity_msg.twist.angular.x = 0.0
                self.velocity_msg.twist.angular.y = 0.0
                self.velocity_msg.twist.angular.z = 0.0
                
                self.velocity_pub.publish(self.velocity_msg)
            
        except Exception as e:
            rospy.logerr("Error publishing data: %s", str(e))
    
    def run(self):
        """Main loop"""
        rospy.loginfo("Camera-LiDAR MAVROS Node starting...")
        
        while not rospy.is_shutdown():
            try:
                self.publish_data()
                self.publish_rate.sleep()
                
            except rospy.ROSInterruptException:
                break
            except Exception as e:
                rospy.logerr("Error in main loop: %s", str(e))
                rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        node = CameraLidarMAVROSNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
