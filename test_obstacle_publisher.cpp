#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_obstacle_publisher");
    ros::NodeHandle nh;
    
    ros::Publisher pub = nh.advertise<sensor_msgs::LaserScan>("/mavros/obstacle/send", 1);
    
    ROS_INFO("Test obstacle publisher initialized");
    ROS_INFO("Publishing to: /mavros/obstacle/send");
    
    ros::Rate rate(5);  // 5 Hz
    
    while (ros::ok()) {
        sensor_msgs::LaserScan msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "base_link";
        msg.angle_min = 0.0;
        msg.angle_max = 2 * M_PI;
        msg.angle_increment = M_PI / 4;  // 8 bins
        msg.time_increment = 0.0;
        msg.scan_time = 0.2;
        msg.range_min = 0.2;
        msg.range_max = 4.0;
        msg.ranges = {1.0, 2.0, 3.0, 4.0, 1.0, 2.0, 3.0, 4.0};
        msg.intensities = {};
        
        pub.publish(msg);
        rate.sleep();
    }
    
    return 0;
}

