#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
from mavros_msgs.msg import ObstacleDistance

def main():
    rospy.init_node('simple_obstacle_test', anonymous=True)
    
    # Create publisher
    pub = rospy.Publisher('/mavros/obstacle/send', ObstacleDistance, queue_size=1)
    
    # Create obstacle message
    msg = ObstacleDistance()
    msg.header.frame_id = "base_link"
    msg.sensor_type = ObstacleDistance.MAV_DISTANCE_SENSOR_LASER
    msg.min_distance = 10  # 10cm
    msg.max_distance = 3000  # 30m
    msg.field_of_view = math.radians(180.0)
    msg.increment = math.radians(2.0)
    msg.angle_offset = math.radians(-90.0)
    
    # Create test distances (91 measurements for 180 degrees)
    distances = []
    for i in range(91):
        angle = -90 + i * 2  # -90 to +90 degrees
        if abs(angle - 45) < 10:  # Obstacle at 45 degrees
            distance = 500  # 5 meters
        elif abs(angle + 45) < 10:  # Obstacle at -45 degrees
            distance = 300  # 3 meters
        else:
            distance = 0  # No obstacle
        distances.append(distance)
    
    msg.distances = distances
    
    rate = rospy.Rate(1)  # 1 Hz
    
    rospy.loginfo("Simple obstacle test node started")
    rospy.loginfo("Publishing test obstacles at +45° (5m) and -45° (3m)")
    
    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rospy.loginfo("Published obstacle message with %d measurements", len(distances))
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
