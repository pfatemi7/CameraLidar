#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf2_ros
import geometry_msgs.msg
import tf2_geometry_msgs

class TFPublisher:
    def __init__(self):
        rospy.init_node('tf_publisher', anonymous=True)
        
        # Create TF broadcaster
        self.br = tf2_ros.TransformBroadcaster()
        
        # Create static transform broadcaster
        self.static_br = tf2_ros.StaticTransformBroadcaster()
        
        # Publish static transforms
        self.publish_static_transforms()
        
        rospy.loginfo("TF Publisher started")
        
    def publish_static_transforms(self):
        """Publish static transforms for the system"""
        
        # Create map to base_link transform
        map_to_base = geometry_msgs.msg.TransformStamped()
        map_to_base.header.stamp = rospy.Time.now()
        map_to_base.header.frame_id = "map"
        map_to_base.child_frame_id = "base_link"
        map_to_base.transform.translation.x = 0.0
        map_to_base.transform.translation.y = 0.0
        map_to_base.transform.translation.z = 0.0
        map_to_base.transform.rotation.x = 0.0
        map_to_base.transform.rotation.y = 0.0
        map_to_base.transform.rotation.z = 0.0
        map_to_base.transform.rotation.w = 1.0
        
        # Create base_link to unilidar_lidar transform
        base_to_lidar = geometry_msgs.msg.TransformStamped()
        base_to_lidar.header.stamp = rospy.Time.now()
        base_to_lidar.header.frame_id = "base_link"
        base_to_lidar.child_frame_id = "unilidar_lidar"
        base_to_lidar.transform.translation.x = 0.0
        base_to_lidar.transform.translation.y = 0.0
        base_to_lidar.transform.translation.z = 0.0
        base_to_lidar.transform.rotation.x = 0.0
        base_to_lidar.transform.rotation.y = 0.0
        base_to_lidar.transform.rotation.z = 0.0
        base_to_lidar.transform.rotation.w = 1.0
        
        # Publish static transforms
        self.static_br.sendTransform([map_to_base, base_to_lidar])
        rospy.loginfo("Published static transforms")
        
    def run(self):
        """Main loop"""
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            # Publish dynamic transforms if needed
            rate.sleep()

if __name__ == '__main__':
    try:
        tf_pub = TFPublisher()
        tf_pub.run()
    except rospy.ROSInterruptException:
        pass
