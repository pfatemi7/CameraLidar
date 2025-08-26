#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
import numpy as np
from sensor_msgs.msg import PointCloud2
from mavros_msgs.msg import ObstacleDistance
from std_msgs.msg import Float32
import struct

class MAVROSObstacleNode:
    def __init__(self):
        rospy.init_node('mavros_obstacle_node', anonymous=True)
        
        # Publishers
        self.obstacle_pub = rospy.Publisher('/mavros/obstacle/send', ObstacleDistance, queue_size=1)
        
        # Subscribers
        self.cloud_sub = rospy.Subscriber('/fused/cloud', PointCloud2, self.cloud_callback)
        self.distance_sub = rospy.Subscriber('/object_detection/distance', Float32, self.distance_callback)
        
        # Obstacle avoidance parameters
        self.fov_horizontal = rospy.get_param('~fov_horizontal', 180.0)  # degrees
        self.fov_vertical = rospy.get_param('~fov_vertical', 60.0)      # degrees
        self.max_distance = rospy.get_param('~max_distance', 30.0)      # meters
        self.angle_increment = rospy.get_param('~angle_increment', 2.0)  # degrees
        
        # Calculate number of distance measurements
        self.num_measurements = int(self.fov_horizontal / self.angle_increment) + 1
        
        # Initialize distance array (all unknown = 0)
        self.distances = np.zeros(self.num_measurements, dtype=np.uint16)
        
        # Obstacle message
        self.obstacle_msg = ObstacleDistance()
        self.obstacle_msg.header.frame_id = "base_link"
        self.obstacle_msg.sensor_type = ObstacleDistance.MAV_DISTANCE_SENSOR_LASER
        self.obstacle_msg.min_distance = 0.1  # 10cm minimum
        self.obstacle_msg.max_distance = self.max_distance * 100  # Convert to cm
        self.obstacle_msg.field_of_view = math.radians(self.fov_horizontal)
        self.obstacle_msg.increment = math.radians(self.angle_increment)
        self.obstacle_msg.angle_offset = math.radians(-self.fov_horizontal / 2.0)  # Center the FOV
        
        # Store latest point cloud
        self.latest_cloud = None
        
        # Publish rate
        self.publish_rate = rospy.Rate(10)  # 10 Hz
        
        rospy.loginfo("MAVROS Obstacle Node initialized")
        rospy.loginfo("FOV: %.1f° x %.1f°", self.fov_horizontal, self.fov_vertical)
        rospy.loginfo("Max distance: %.1f m", self.max_distance)
        rospy.loginfo("Angle increment: %.1f°", self.angle_increment)
        rospy.loginfo("Number of measurements: %d", self.num_measurements)
    
    def cloud_callback(self, cloud_msg):
        """Callback for point cloud messages"""
        self.latest_cloud = cloud_msg
        self.process_point_cloud()
    
    def distance_callback(self, distance_msg):
        """Callback for object detection distance"""
        # This can be used for additional object-specific avoidance
        rospy.loginfo("Object detected at %.2f meters", distance_msg.data)
    
    def process_point_cloud(self):
        """Process point cloud to create obstacle distance array"""
        if self.latest_cloud is None:
            return
        
        try:
            # Reset distances to unknown
            self.distances.fill(0)
            
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
            
            # Process each point
            for i in range(0, len(self.latest_cloud.data), point_step):
                if i + point_step <= len(self.latest_cloud.data):
                    point_data = self.latest_cloud.data[i:i+point_step]
                    
                    # Extract x, y, z values
                    x = struct.unpack('f', point_data[x_offset:x_offset+4])[0]
                    y = struct.unpack('f', point_data[y_offset:y_offset+4])[0]
                    z = struct.unpack('f', point_data[z_offset:z_offset+4])[0]
                    
                    # Calculate distance and angle
                    distance = math.sqrt(x*x + y*y + z*z)
                    
                    # Only consider points within max distance
                    if distance > self.max_distance:
                        continue
                    
                    # Calculate horizontal angle (azimuth)
                    angle_horizontal = math.degrees(math.atan2(y, x))
                    
                    # Normalize angle to 0-360
                    if angle_horizontal < 0:
                        angle_horizontal += 360
                    
                    # Convert to our FOV range (-90 to +90 degrees)
                    if angle_horizontal > 180:
                        angle_horizontal -= 360
                    
                    # Check if point is within horizontal FOV
                    if abs(angle_horizontal) <= self.fov_horizontal / 2.0:
                        # Calculate vertical angle (elevation)
                        angle_vertical = math.degrees(math.atan2(z, math.sqrt(x*x + y*y)))
                        
                        # Check if point is within vertical FOV
                        if abs(angle_vertical) <= self.fov_vertical / 2.0:
                            # Find the corresponding distance array index
                            array_index = int((angle_horizontal + self.fov_horizontal / 2.0) / self.angle_increment)
                            
                            if 0 <= array_index < self.num_measurements:
                                # Convert distance to cm and ensure it's within valid range
                                distance_cm = int(distance * 100)
                                distance_cm = max(10, min(distance_cm, int(self.max_distance * 100)))  # 10cm to max_distance
                                
                                # Update distance if this point is closer than current measurement
                                if self.distances[array_index] == 0 or distance_cm < self.distances[array_index]:
                                    self.distances[array_index] = distance_cm
            
            # Publish obstacle message
            self.publish_obstacle_message()
            
        except Exception as e:
            rospy.logerr("Error processing point cloud: %s", str(e))
    
    def publish_obstacle_message(self):
        """Publish the obstacle distance message"""
        try:
            # Update message
            self.obstacle_msg.header.stamp = rospy.Time.now()
            self.obstacle_msg.distances = self.distances.tolist()
            
            # Publish
            self.obstacle_pub.publish(self.obstacle_msg)
            
            # Log statistics every 10 seconds
            if rospy.Time.now().secs % 10 == 0:
                valid_measurements = np.count_nonzero(self.distances)
                rospy.loginfo("Obstacle avoidance: %d/%d valid measurements", 
                             valid_measurements, self.num_measurements)
                
        except Exception as e:
            rospy.logerr("Error publishing obstacle message: %s", str(e))
    
    def run(self):
        """Main loop"""
        while not rospy.is_shutdown():
            # Process any pending point clouds
            if self.latest_cloud is not None:
                self.process_point_cloud()
            
            # Sleep
            self.publish_rate.sleep()

if __name__ == '__main__':
    try:
        node = MAVROSObstacleNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

