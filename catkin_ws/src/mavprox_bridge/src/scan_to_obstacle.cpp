#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32MultiArray.h>
#include <vector>

class ScanToObstacleBridge {
private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher range_pub_;
    ros::Publisher distances_pub_;
    
    // Parameters
    int num_bins_;
    double min_distance_;
    double max_distance_;
    double angle_increment_;
    double angle_offset_;
    
public:
    ScanToObstacleBridge() : nh_("~") {
        // Initialize parameters
        num_bins_ = 72;  // 5° each from 0° to 360°
        min_distance_ = 0.1;  // 10cm minimum
        max_distance_ = 50.0;  // 50m maximum
        angle_increment_ = 5.0;  // 5 degrees per bin
        angle_offset_ = 0.0;  // 0° forward
        
        // Get parameters from parameter server
        nh_.param("num_bins", num_bins_, num_bins_);
        nh_.param("min_distance", min_distance_, min_distance_);
        nh_.param("max_distance", max_distance_, max_distance_);
        nh_.param("angle_increment", angle_increment_, angle_increment_);
        nh_.param("angle_offset", angle_offset_, angle_offset_);
        
        // Setup subscribers and publishers
        scan_sub_ = nh_.subscribe("/scan", 1, &ScanToObstacleBridge::scanCallback, this);
        range_pub_ = nh_.advertise<sensor_msgs::Range>("/mavros/obstacle/range", 1);
        distances_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/mavros/obstacle/distances", 1);
        
        ROS_INFO("Scan to Obstacle Bridge initialized");
        ROS_INFO("Subscribing to: /scan");
        ROS_INFO("Publishing to: /mavros/obstacle/range and /mavros/obstacle/distances");
        ROS_INFO("Parameters: bins=%d, min=%.1fm, max=%.1fm, inc=%.1f°, offset=%.1f°", 
                 num_bins_, min_distance_, max_distance_, angle_increment_, angle_offset_);
    }
    
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
        // Convert scan data to 72 bins
        std::vector<double> binned_distances(num_bins_, std::numeric_limits<double>::infinity());
        
        // Map scan angles to bins
        for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
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
            
            // Update bin with minimum distance
            double range = scan_msg->ranges[i];
            if (range >= scan_msg->range_min && range <= scan_msg->range_max && 
                range >= min_distance_ && range <= max_distance_) {
                if (binned_distances[bin] > range) {
                    binned_distances[bin] = range;
                }
            }
        }
        
        // Publish closest distance as Range message
        sensor_msgs::Range range_msg;
        range_msg.header.stamp = ros::Time::now();
        range_msg.header.frame_id = "base_link";
        range_msg.radiation_type = sensor_msgs::Range::INFRARED;
        range_msg.field_of_view = 0.1;  // 0.1 radians
        range_msg.min_range = min_distance_;
        range_msg.max_range = max_distance_;
        
        // Find closest distance
        double closest_distance = max_distance_;
        for (int i = 0; i < num_bins_; ++i) {
            if (!std::isinf(binned_distances[i]) && binned_distances[i] < closest_distance) {
                closest_distance = binned_distances[i];
            }
        }
        range_msg.range = closest_distance;
        range_pub_.publish(range_msg);
        
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
        
        // Log some statistics
        int valid_readings = 0;
        for (int i = 0; i < num_bins_; ++i) {
            if (!std::isinf(binned_distances[i])) {
                valid_readings++;
            }
        }
        
        ROS_DEBUG("Published obstacle data: %d/%d valid readings, closest=%.2fm", 
                  valid_readings, num_bins_, closest_distance);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "scan_to_obstacle");
    
    ScanToObstacleBridge bridge;
    
    ros::spin();
    
    return 0;
}
