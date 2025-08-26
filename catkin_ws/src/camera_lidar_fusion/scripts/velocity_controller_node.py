#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
import numpy as np
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import ObstacleDistance
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud2
import struct

class VelocityControllerNode:
    def __init__(self):
        rospy.init_node('velocity_controller_node', anonymous=True)
        
        # Publishers
        self.velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)
        
        # Subscribers
        self.obstacle_sub = rospy.Subscriber('/mavros/obstacle/send', ObstacleDistance, self.obstacle_callback)
        self.cloud_sub = rospy.Subscriber('/fused/cloud', PointCloud2, self.cloud_callback)
        
        # Parameters
        self.max_velocity = rospy.get_param('~max_velocity', 2.0)  # m/s
        self.safety_distance = rospy.get_param('~safety_distance', 5.0)  # meters
        self.enable_avoidance = rospy.get_param('~enable_avoidance', True)
        
        # Control variables
        self.current_velocity = TwistStamped()
        self.current_velocity.header.frame_id = "base_link"
        
        # Obstacle avoidance state
        self.obstacle_detected = False
        self.closest_obstacle_distance = float('inf')
        self.obstacle_angle = 0.0
        
        # Publish rate
        self.publish_rate = rospy.Rate(20)  # 20 Hz
        
        rospy.loginfo("Velocity Controller Node initialized")
        rospy.loginfo("Max velocity: %.1f m/s", self.max_velocity)
        rospy.loginfo("Safety distance: %.1f m", self.safety_distance)
        rospy.loginfo("Obstacle avoidance: %s", "enabled" if self.enable_avoidance else "disabled")
    
    def obstacle_callback(self, obstacle_msg):
        """Callback for obstacle distance messages"""
        if not self.enable_avoidance:
            return
        
        try:
            # Find the closest obstacle
            distances = np.array(obstacle_msg.distances)
            valid_distances = distances[distances > 0]  # Filter out unknown (0) distances
            
            if len(valid_distances) > 0:
                min_distance_idx = np.argmin(valid_distances)
                self.closest_obstacle_distance = valid_distances[min_distance_idx] / 100.0  # Convert cm to m
                
                # Calculate obstacle angle
                angle_increment = obstacle_msg.increment
                angle_offset = obstacle_msg.angle_offset
                self.obstacle_angle = angle_offset + min_distance_idx * angle_increment
                
                self.obstacle_detected = True
                
                # Log if obstacle is close
                if self.closest_obstacle_distance < self.safety_distance:
                    rospy.logwarn("Obstacle detected at %.2f m, angle %.1f°", 
                                 self.closest_obstacle_distance, math.degrees(self.obstacle_angle))
            else:
                self.obstacle_detected = False
                self.closest_obstacle_distance = float('inf')
                
        except Exception as e:
            rospy.logerr("Error processing obstacle message: %s", str(e))
    
    def cloud_callback(self, cloud_msg):
        """Callback for point cloud messages (for additional processing if needed)"""
        # This can be used for more sophisticated obstacle avoidance
        pass
    
    def calculate_avoidance_velocity(self):
        """Calculate velocity setpoint based on obstacle avoidance"""
        if not self.obstacle_detected or self.closest_obstacle_distance > self.safety_distance:
            # No obstacle detected or obstacle is far away
            # Set forward velocity
            self.current_velocity.twist.linear.x = self.max_velocity
            self.current_velocity.twist.linear.y = 0.0
            self.current_velocity.twist.linear.z = 0.0
            self.current_velocity.twist.angular.x = 0.0
            self.current_velocity.twist.angular.y = 0.0
            self.current_velocity.twist.angular.z = 0.0
        else:
            # Obstacle detected - implement avoidance behavior
            avoidance_strength = 1.0 - (self.closest_obstacle_distance / self.safety_distance)
            avoidance_strength = max(0.0, min(1.0, avoidance_strength))
            
            # Reduce forward velocity based on obstacle proximity
            forward_velocity = self.max_velocity * (1.0 - avoidance_strength * 0.8)
            
            # Calculate lateral velocity for avoidance
            # Turn away from obstacle
            avoidance_angle = self.obstacle_angle + math.pi  # Turn away from obstacle
            lateral_velocity = self.max_velocity * 0.5 * avoidance_strength * math.sin(avoidance_angle)
            
            # Set velocities
            self.current_velocity.twist.linear.x = forward_velocity
            self.current_velocity.twist.linear.y = lateral_velocity
            self.current_velocity.twist.linear.z = 0.0
            
            # Add some yaw rate for smoother avoidance
            yaw_rate = self.max_velocity * 0.3 * avoidance_strength * math.sin(avoidance_angle)
            self.current_velocity.twist.angular.x = 0.0
            self.current_velocity.twist.angular.y = 0.0
            self.current_velocity.twist.angular.z = yaw_rate
    
    def publish_velocity(self):
        """Publish velocity setpoint"""
        try:
            # Update timestamp
            self.current_velocity.header.stamp = rospy.Time.now()
            
            # Calculate avoidance velocity if enabled
            if self.enable_avoidance:
                self.calculate_avoidance_velocity()
            
            # Publish
            self.velocity_pub.publish(self.current_velocity)
            
        except Exception as e:
            rospy.logerr("Error publishing velocity: %s", str(e))
    
    def run(self):
        """Main loop"""
        rospy.loginfo("Velocity Controller starting...")
        
        # Wait for MAVROS to be ready
        rospy.sleep(2.0)
        
        while not rospy.is_shutdown():
            try:
                self.publish_velocity()
                self.publish_rate.sleep()
                
            except rospy.ROSInterruptException:
                break
            except Exception as e:
                rospy.logerr("Error in velocity controller loop: %s", str(e))
                rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        node = VelocityControllerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

