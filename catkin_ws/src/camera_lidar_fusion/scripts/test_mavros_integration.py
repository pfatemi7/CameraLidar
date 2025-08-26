#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
import numpy as np
from mavros_msgs.msg import ObstacleDistance
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud2
import struct

class MAVROSTestNode:
    def __init__(self):
        rospy.init_node('mavros_test_node', anonymous=True)
        
        # Publishers
        self.test_obstacle_pub = rospy.Publisher('/mavros/obstacle/send', ObstacleDistance, queue_size=1)
        
        # Test parameters
        self.fov_horizontal = 180.0  # degrees
        self.angle_increment = 2.0   # degrees
        self.num_measurements = int(self.fov_horizontal / self.angle_increment) + 1
        
        # Create test obstacle message
        self.test_obstacle_msg = ObstacleDistance()
        self.test_obstacle_msg.header.frame_id = "base_link"
        self.test_obstacle_msg.sensor_type = ObstacleDistance.MAV_DISTANCE_SENSOR_LASER
        self.test_obstacle_msg.min_distance = 10  # 10cm minimum
        self.test_obstacle_msg.max_distance = 3000  # 30m maximum
        self.test_obstacle_msg.field_of_view = math.radians(self.fov_horizontal)
        self.test_obstacle_msg.increment = math.radians(self.angle_increment)
        self.test_obstacle_msg.angle_offset = math.radians(-self.fov_horizontal / 2.0)
        
        # Create test distance array (simulating obstacles at different angles)
        self.test_distances = []
        for i in range(self.num_measurements):
            angle = -90 + i * self.angle_increment
            # Create a test obstacle at 45 degrees (5 meters away)
            if abs(angle - 45) < 10:  # 10 degree tolerance
                distance = 500  # 5 meters in cm
            elif abs(angle + 45) < 10:  # -45 degrees
                distance = 300  # 3 meters in cm
            else:
                distance = 0  # No obstacle
            self.test_distances.append(distance)
        
        self.test_obstacle_msg.distances = self.test_distances
        
        # Publish rate
        self.publish_rate = rospy.Rate(1)  # 1 Hz for testing
        
        rospy.loginfo("MAVROS Test Node initialized")
        rospy.loginfo("Publishing test obstacle data at 1 Hz")
        rospy.loginfo("Test obstacles at +45° (5m) and -45° (3m)")
    
    def run(self):
        """Main loop"""
        while not rospy.is_shutdown():
            try:
                # Update timestamp
                self.test_obstacle_msg.header.stamp = rospy.Time.now()
                
                # Publish test obstacle message
                self.test_obstacle_pub.publish(self.test_obstacle_msg)
                
                rospy.loginfo("Published test obstacle message with %d measurements", len(self.test_distances))
                
                # Sleep
                self.publish_rate.sleep()
                
            except rospy.ROSInterruptException:
                break
            except Exception as e:
                rospy.logerr("Error in test node: %s", str(e))
                rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        node = MAVROSTestNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
