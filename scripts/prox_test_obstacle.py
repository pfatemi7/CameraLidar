#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, math
from sensor_msgs.msg import LaserScan

def main():
    rospy.init_node("prox_test_obstacle", anonymous=True)
    pub = rospy.Publisher("/mavros/obstacle/send", LaserScan, queue_size=1)
    rate = rospy.Rate(5)  # 5 Hz is fine

    msg = LaserScan()
    msg.header.frame_id = "base_link"
    msg.angle_min = 0.0
    msg.angle_max = 2 * math.pi  # 360 degrees
    msg.angle_increment = math.pi / 4  # 8 bins around 360 degrees (45 degrees each)
    msg.time_increment = 0.0
    msg.scan_time = 0.2  # 5 Hz
    msg.range_min = 0.2  # 20 cm
    msg.range_max = 4.0  # 400 cm
    msg.ranges = [1.0, 2.0, 3.0, 4.0, 1.0, 2.0, 3.0, 4.0]  # meters
    msg.intensities = []

    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    main()

