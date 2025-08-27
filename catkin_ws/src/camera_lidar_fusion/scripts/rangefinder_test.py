#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
from sensor_msgs.msg import Range
from geometry_msgs.msg import TwistStamped

def main():
    rospy.init_node('rangefinder_test', anonymous=True)
    
    # Create publishers for different directions
    front_pub = rospy.Publisher('/mavros/rangefinder/rangefinder', Range, queue_size=1)
    velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)
    
    # Create range message
    range_msg = Range()
    range_msg.header.frame_id = "base_link"
    range_msg.radiation_type = Range.ULTRASOUND
    range_msg.field_of_view = math.radians(30.0)  # 30 degree FOV
    range_msg.min_range = 0.1  # 10cm
    range_msg.max_range = 30.0  # 30m
    
    # Create velocity message
    velocity_msg = TwistStamped()
    velocity_msg.header.frame_id = "base_link"
    
    rate = rospy.Rate(10)  # 10 Hz
    
    rospy.loginfo("Rangefinder test node started")
    rospy.loginfo("Publishing simulated obstacle at 5 meters")
    
    while not rospy.is_shutdown():
        # Simulate obstacle at 5 meters
        range_msg.header.stamp = rospy.Time.now()
        range_msg.range = 5.0  # 5 meters
        
        # Publish range data
        front_pub.publish(range_msg)
        
        # Publish velocity (slow down when obstacle detected)
        velocity_msg.header.stamp = rospy.Time.now()
        if range_msg.range < 10.0:  # If obstacle closer than 10m
            velocity_msg.twist.linear.x = 1.0  # Slow down
            rospy.loginfo("Obstacle detected at %.1f meters - slowing down", range_msg.range)
        else:
            velocity_msg.twist.linear.x = 5.0  # Normal speed
            rospy.loginfo("No obstacle - normal speed")
        
        velocity_msg.twist.linear.y = 0.0
        velocity_msg.twist.linear.z = 0.0
        velocity_msg.twist.angular.x = 0.0
        velocity_msg.twist.angular.y = 0.0
        velocity_msg.twist.angular.z = 0.0
        
        velocity_pub.publish(velocity_msg)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
