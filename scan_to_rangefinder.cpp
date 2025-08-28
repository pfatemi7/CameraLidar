#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32MultiArray.h>
#include <vector>
#include <limits> // For std::numeric_limits
#include <cmath> // For M_PI, atan2, sqrt

// Define M_PI if not available
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class ScanToRangefinderBridge {
private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher rangefinder_pub_;
    ros::Publisher distances_pub_;
    ros::Publisher obstacle_send_pub_;  // New publisher for obstacle/send
    
    // Parameters
    int num_bins_;
    double min_distance_;
    double max_distance_;
    double angle_increment_;
    double angle_offset_;
    
    // New filtering parameters
    double min_range_;  // Minimum range to publish (default 1.0m)
    double max_range_;  // Maximum range to publish (default 5.0m)
    
public:
    ScanToRangefinderBridge() : nh_("~") {
        // Initialize parameters
        num_bins_ = 72;  // 5° each from 0° to 360°
        min_distance_ = 0.1;  // 10cm minimum (for sensor validation)
        max_distance_ = 50.0;  // 50m maximum (for sensor validation)
        angle_increment_ = 5.0;  // 5 degrees per bin
        angle_offset_ = 0.0;  // 0° forward
        
        // Initialize filtering parameters
        min_range_ = 1.0;  // Default: ignore readings closer than 1.0m
        max_range_ = 5.0;  // Default: treat readings >5.0m as no obstacle
        
        // Get parameters from parameter server
        nh_.param("num_bins", num_bins_, num_bins_);
        nh_.param("min_distance", min_distance_, min_distance_);
        nh_.param("max_distance", max_distance_, max_distance_);
        nh_.param("angle_increment", angle_increment_, angle_increment_);
        nh_.param("angle_offset", angle_offset_, angle_offset_);
        
        // Get filtering parameters
        nh_.param("min_range", min_range_, min_range_);
        nh_.param("max_range", max_range_, max_range_);
        
        // Setup subscribers and publishers
        scan_sub_ = nh_.subscribe("/scan", 1, &ScanToRangefinderBridge::scanCallback, this);
        // Publish to MAVROS distance_sensor topic that ArduPilot expects
        rangefinder_pub_ = nh_.advertise<sensor_msgs::Range>("/mavros/distance_sensor/rangefinder_pub", 1);
        distances_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/mavros/obstacle/distances", 1);
        obstacle_send_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/mavros/obstacle/send", 1);  // New publisher
        
        ROS_INFO("Scan to Rangefinder Bridge initialized");
        ROS_INFO("Subscribing to: /scan");
        ROS_INFO("Publishing to: /mavros/distance_sensor/rangefinder_pub");
        ROS_INFO("Publishing to: /mavros/obstacle/send");
        ROS_INFO("Parameters: bins=%d, min=%.1fm, max=%.1fm, inc=%.1f°, offset=%.1f°", 
                 num_bins_, min_distance_, max_distance_, angle_increment_, angle_offset_);
        ROS_INFO("Filtering: min_range=%.1fm, max_range=%.1fm (only publishing 1.0-5.0m range)", 
                 min_range_, max_range_);
    }
    
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
        // Convert scan data to 72 bins
        std::vector<double> binned_distances(num_bins_, std::numeric_limits<double>::infinity());
        
        // Statistics for logging
        int total_points = 0;
        int skipped_too_close = 0;
        int skipped_too_far = 0;
        int skipped_invalid = 0;
        int valid_filtered_points = 0;
        
        // Map scan angles to bins
        for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
            total_points++;
            double range = scan_msg->ranges[i];
            
            // Check if range is valid from sensor perspective (use sensor's own min/max)
            if (range < scan_msg->range_min || range > scan_msg->range_max) {
                skipped_invalid++;
                continue;
            }
            
            // Apply filtering: ignore readings outside our desired range
            if (range < min_range_) {
                skipped_too_close++;
                continue;
            }
            
            if (range > max_range_) {
                skipped_too_far++;
                continue;
            }
            
            // Range is valid and within our filtering bounds
            valid_filtered_points++;
            
            double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
            
            // Normalize angle to 0-360°
            while (angle < 0) angle += 2 * M_PI;
            while (angle >= 2 * M_PI) angle -= 2 * M_PI;
            
            // Convert to degrees
            double angle_deg = angle * 180.0 / M_PI;
            
            // Apply offset
            angle_deg += angle_offset_;
            while (angle_deg < 0) angle_deg += 360.0;
            while (angle_deg >= 360.0) angle_deg -= 360.0;
            
            // Find corresponding bin
            int bin = static_cast<int>(angle_deg / angle_increment_) % num_bins_;
            
            // Update bin with minimum distance (only valid filtered ranges)
            if (binned_distances[bin] > range) {
                binned_distances[bin] = range;
            }
        }
        
        // Log filtering statistics
        ROS_INFO("Filtering stats: total=%d, too_close(<%.1fm)=%d, too_far(>%.1fm)=%d, invalid=%d, valid_filtered=%d", 
                 total_points, min_range_, skipped_too_close, max_range_, skipped_too_far, 
                 skipped_invalid, valid_filtered_points);
        
        // Publish closest distance as Range message to MAVROS rangefinder
        sensor_msgs::Range range_msg;
        range_msg.header.stamp = ros::Time::now();
        range_msg.header.frame_id = "base_link";
        range_msg.radiation_type = sensor_msgs::Range::INFRARED;
        range_msg.field_of_view = 0.1;  // 0.1 radians
        range_msg.min_range = min_range_;  // Use filtered min range
        range_msg.max_range = max_range_;  // Use filtered max range
        
        // Find closest distance from filtered data
        double closest_distance = max_range_;  // Default to max range if no valid readings
        for (int i = 0; i < num_bins_; ++i) {
            if (!std::isinf(binned_distances[i]) && binned_distances[i] < closest_distance) {
                closest_distance = binned_distances[i];
            }
        }
        range_msg.range = closest_distance;
        rangefinder_pub_.publish(range_msg);
        
        // Publish all distances as Float32MultiArray
        std_msgs::Float32MultiArray distances_msg;
        distances_msg.data.resize(num_bins_);
        for (int i = 0; i < num_bins_; ++i) {
            if (std::isinf(binned_distances[i])) {
                distances_msg.data[i] = -1.0;  // No reading
            } else {
                distances_msg.data[i] = binned_distances[i];
            }
        }
        distances_pub_.publish(distances_msg);
        
        // Publish LaserScan for obstacle/send (proximity detection)
        sensor_msgs::LaserScan obstacle_msg;
        obstacle_msg.header.stamp = ros::Time::now();
        obstacle_msg.header.frame_id = "base_link";
        obstacle_msg.angle_min = 0.0;
        obstacle_msg.angle_max = 2 * M_PI;  // 360 degrees
        obstacle_msg.angle_increment = angle_increment_ * M_PI / 180.0;  // Convert degrees to radians
        obstacle_msg.time_increment = 0.0;
        obstacle_msg.scan_time = 0.1;  // 10 Hz
        obstacle_msg.range_min = min_range_;  // Use filtered min range
        obstacle_msg.range_max = max_range_;  // Use filtered max range
        
        // Convert binned distances to ranges array
        obstacle_msg.ranges.resize(num_bins_);
        obstacle_msg.intensities.resize(num_bins_);
        for (int i = 0; i < num_bins_; ++i) {
            if (std::isinf(binned_distances[i])) {
                obstacle_msg.ranges[i] = max_range_;  // No reading = max range (no obstacle)
            } else {
                obstacle_msg.ranges[i] = binned_distances[i];
            }
            obstacle_msg.intensities[i] = 1.0;  // Default intensity
        }
        obstacle_send_pub_.publish(obstacle_msg);
        
        // Log some statistics
        int valid_readings = 0;
        for (int i = 0; i < num_bins_; ++i) {
            if (!std::isinf(binned_distances[i])) {
                valid_readings++;
            }
        }
        
        ROS_DEBUG("Published rangefinder data: %d/%d valid readings, closest=%.2fm", 
                  valid_readings, num_bins_, closest_distance);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "scan_to_rangefinder");
    
    ScanToRangefinderBridge bridge;
    
    ros::spin();
    
    return 0;
}
