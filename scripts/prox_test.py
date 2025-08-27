#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, math
from mavros_msgs.msg import ObstacleDistance

def main():
    rospy.init_node("prox_test", anonymous=True)
    pub = rospy.Publisher("/mavros/obstacle/send", ObstacleDistance, queue_size=1)
    rate = rospy.Rate(5)  # 5 Hz is fine

    msg = ObstacleDistance()
    msg.time_usec = 0  # autopilot fills if 0
    msg.sensor_type = ObstacleDistance.MAV_DISTANCE_SENSOR_LASER
    msg.increment_f = math.radians(45.0)   # 8 bins around 360 degrees
    msg.angle_offset = 0.0
    msg.min_distance = 20                  # cm
    msg.max_distance = 400                 # cm
    msg.frame = ObstacleDistance.MAV_FRAME_BODY_FRD
    msg.distances = [100,200,300,400,100,200,300,400]  # cm (65535 = no return)

    while not rospy.is_shutdown():
        # Keep length at 8; autopilot accepts 72 as well, but 8 is fine for test
        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    main()
