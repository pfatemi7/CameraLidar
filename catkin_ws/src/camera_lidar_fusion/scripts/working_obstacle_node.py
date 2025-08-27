#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
import numpy as np
from sensor_msgs.msg import PointCloud2, LaserScan
from std_msgs.msg import Float32
import struct

class WorkingObstacleNode:
    def __init__(self):
        rospy.init_node('working_obstacle_node', anonymous=True)
        
        # Publishers
        self.obstacle_pub = rospy.Publisher('/mavros/obstacle/send', LaserScan, queue_size=1)
        
        # Subscribers
        self.cloud_sub = rospy.Subscriber('/fused/cloud', PointCloud2, self.cloud_callback)
        self.distance_sub = rospy.Subscriber('/object_detection/distance', Float32, self.distance_callback)
        
        # Parameters
        self.safety_distance = rospy.get_param('~safety_distance', 5.0)  # meters
        self.max_distance = rospy.get_param('~max_distance', 30.0)  # meters
        self.angle_increment = rospy.get_param('~angle_increment', 2.0)  # degrees
        
        # Calculate number of measurements for 180 degrees
        self.num_measurements = int(180.0 / self.angle_increment) + 1
        
        # LaserScan message
        self.scan_msg = LaserScan()
        self.scan_msg.header.frame_id = "base_link"
        self.scan_msg.angle_min = math.radians(-90.0)  # -90 degrees
        self.scan_msg.angle_max = math.radians(90.0)   # +90 degrees
        self.scan_msg.angle_increment = math.radians(self.angle_increment)
        self.scan_msg.time_increment = 0.0
        self.scan_msg.scan_time = 0.1  # 10 Hz
        self.scan_msg.range_min = 0.1  # 10cm
        self.scan_msg.range_max = self.max_distance
        
        # State variables
        self.latest_cloud = None
        self.latest_distance = None
        
        # Publish rate
        self.publish_rate = rospy.Rate(10)  # 10 Hz
        
        rospy.loginfo("Working Obstacle Node initialized")
        rospy.loginfo("Safety distance: %.1f m", self.safety_distance)
        rospy.loginfo("Max distance: %.1f m", self.max_distance)
        rospy.loginfo("Angle increment: %.1f degrees", self.angle_increment)
        rospy.loginfo("Number of measurements: %d", self.num_measurements)
    
    def cloud_callback(self, cloud_msg):
        """Process point cloud to find obstacles"""
        self.latest_cloud = cloud_msg
        self.process_point_cloud()
    
    def distance_callback(self, distance_msg):
        """Process object detection distance"""
        self.latest_distance = distance_msg.data
        rospy.loginfo("Object detected at %.2f meters", self.latest_distance)
    
    def process_point_cloud(self):
        """Extract obstacle distances from point cloud and publish LaserScan"""
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
            
            # Initialize ranges array for 180 degrees
            ranges = [float('inf')] * self.num_measurements  # inf = no obstacle
            
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
                    
                    # Normalize angle to -90 to +90 degrees
                    if angle_horizontal > 180:
                        angle_horizontal -= 360
                    
                    # Check if point is within our FOV (-90 to +90 degrees)
                    if -90 <= angle_horizontal <= 90:
                        # Find the corresponding range array index
                        array_index = int((angle_horizontal + 90) / self.angle_increment)
                        
                        if 0 <= array_index < self.num_measurements:
                            # Update range if this point is closer than current measurement
                            if distance < ranges[array_index]:
                                ranges[array_index] = distance
            
            # Convert inf to 0 for LaserScan (0 = no obstacle)
            for i in range(len(ranges)):
                if ranges[i] == float('inf'):
                    ranges[i] = 0.0
            
            # Update and publish LaserScan message
            self.scan_msg.header.stamp = rospy.Time.now()
            self.scan_msg.ranges = ranges
            
            # Publish
            self.obstacle_pub.publish(self.scan_msg)
            
            # Log some statistics
            valid_measurements = sum(1 for r in ranges if r > 0)
            if valid_measurements > 0:
                min_distance = min(r for r in ranges if r > 0)
                rospy.loginfo("Obstacle data: %d/%d valid measurements, closest: %.2f m", 
                             valid_measurements, self.num_measurements, min_distance)
            
        except Exception as e:
            rospy.logerr("Error processing point cloud: %s", str(e))
    
    def run(self):
        """Main loop"""
        rospy.loginfo("Working Obstacle Node starting...")
        
        while not rospy.is_shutdown():
            try:
                # Process any pending point clouds
                if self.latest_cloud is not None:
                    self.process_point_cloud()
                
                self.publish_rate.sleep()
                
            except rospy.ROSInterruptException:
                break
            except Exception as e:
                rospy.logerr("Error in main loop: %s", str(e))
                rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        node = WorkingObstacleNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
