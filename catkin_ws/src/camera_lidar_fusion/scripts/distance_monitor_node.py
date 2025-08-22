#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float32, String
import time

class DistanceMonitorNode:
    def __init__(self):
        rospy.init_node('distance_monitor_node', anonymous=True)
        
        # Publishers
        self.alert_pub = rospy.Publisher('/distance_monitor/alert', String, queue_size=1)
        self.status_pub = rospy.Publisher('/distance_monitor/status', String, queue_size=1)
        
        # Subscribers
        self.distance_sub = rospy.Subscriber('/object_detection/distance', Float32, self.distance_callback)
        
        # Distance thresholds
        self.warning_distance = rospy.get_param('~warning_distance', 0.9)  # meters (90cm)
        self.danger_distance = rospy.get_param('~danger_distance', 0.5)    # meters (50cm)
        
        # Status tracking
        self.last_distance = None
        self.last_update_time = None
        
        rospy.loginfo("Distance Monitor Node initialized")
        rospy.loginfo("Warning distance: %.1fm (90cm), Danger distance: %.1fm (50cm)", 
                     self.warning_distance, self.danger_distance)
    
    def distance_callback(self, distance_msg):
        """Callback for distance messages"""
        distance = distance_msg.data
        self.last_distance = distance
        self.last_update_time = rospy.Time.now()
        
        # Create status message
        status_msg = String()
        status_msg.data = "Distance: %.2f meters" % distance
        self.status_pub.publish(status_msg)
        
        # Check for alerts
        if distance <= self.danger_distance:
            alert_msg = String()
            alert_msg.data = "DANGER! Object too close: %.2fm" % distance
            self.alert_pub.publish(alert_msg)
            rospy.logwarn("DANGER ALERT: Object at %.2f meters", distance)
            
        elif distance <= self.warning_distance:
            alert_msg = String()
            alert_msg.data = "WARNING! Object nearby: %.2fm" % distance
            self.alert_pub.publish(alert_msg)
            rospy.logwarn("WARNING: Object at %.2f meters", distance)
        
        rospy.loginfo("Distance: %.2f meters", distance)
    
    def run(self):
        """Main loop"""
        rate = rospy.Rate(1)  # 1 Hz status check
        
        while not rospy.is_shutdown():
            # Check if we're still receiving distance updates
            if self.last_update_time is not None:
                time_since_update = (rospy.Time.now() - self.last_update_time).to_sec()
                if time_since_update > 5.0:  # No updates for 5 seconds
                    status_msg = String()
                    status_msg.data = "No object detected (%.1fs)" % time_since_update
                    self.status_pub.publish(status_msg)
            
            rate.sleep()

if __name__ == '__main__':
    try:
        node = DistanceMonitorNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
