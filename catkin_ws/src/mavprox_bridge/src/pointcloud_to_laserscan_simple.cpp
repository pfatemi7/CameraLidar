#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <vector>

class PointCloudToLaserScanSimple {
private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher scan_pub_;
    
    // Parameters
    std::string target_frame_;
    double min_height_;
    double max_height_;
    double angle_min_;
    double angle_max_;
    double angle_increment_;
    double range_min_;
    double range_max_;
    double scan_time_;
    
public:
    PointCloudToLaserScanSimple() : nh_("~") {
        // Initialize parameters
        target_frame_ = "base_link";
        min_height_ = -0.5;
        max_height_ = 2.0;
        angle_min_ = -M_PI;
        angle_max_ = M_PI;
        angle_increment_ = 0.0174533;  // 1 degree
        range_min_ = 0.1;
        range_max_ = 50.0;
        scan_time_ = 0.1;
        
        // Get parameters from parameter server
        nh_.param("target_frame", target_frame_, target_frame_);
        nh_.param("min_height", min_height_, min_height_);
        nh_.param("max_height", max_height_, max_height_);
        nh_.param("angle_min", angle_min_, angle_min_);
        nh_.param("angle_max", angle_max_, angle_max_);
        nh_.param("angle_increment", angle_increment_, angle_increment_);
        nh_.param("range_min", range_min_, range_min_);
        nh_.param("range_max", range_max_, range_max_);
        nh_.param("scan_time", scan_time_, scan_time_);
        
        // Setup subscribers and publishers
        cloud_sub_ = nh_.subscribe("/unilidar/cloud", 1, &PointCloudToLaserScanSimple::cloudCallback, this);
        scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan", 1);
        
        ROS_INFO("PointCloud to LaserScan converter (simple) initialized");
        ROS_INFO("Subscribing to: /unilidar/cloud");
        ROS_INFO("Publishing to: /scan");
        ROS_INFO("Target frame: %s", target_frame_.c_str());
    }
    
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
        // Create LaserScan message
        sensor_msgs::LaserScan scan_msg;
        scan_msg.header.stamp = ros::Time::now();
        scan_msg.header.frame_id = target_frame_;
        scan_msg.angle_min = angle_min_;
        scan_msg.angle_max = angle_max_;
        scan_msg.angle_increment = angle_increment_;
        scan_msg.time_increment = 0.0;
        scan_msg.scan_time = scan_time_;
        scan_msg.range_min = range_min_;
        scan_msg.range_max = range_max_;
        
        // Calculate number of ranges
        int num_ranges = static_cast<int>((angle_max_ - angle_min_) / angle_increment_) + 1;
        scan_msg.ranges.resize(num_ranges, std::numeric_limits<float>::infinity());
        
        // Iterate through point cloud
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");
        
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            float x = *iter_x;
            float y = *iter_y;
            float z = *iter_z;
            
            // Filter by height
            if (z < min_height_ || z > max_height_) {
                continue;
            }
            
            // Calculate angle and range
            double angle = atan2(y, x);
            double range = sqrt(x * x + y * y);
            
            // Check range limits
            if (range < range_min_ || range > range_max_) {
                continue;
            }
            
            // Normalize angle to [angle_min, angle_max]
            while (angle < angle_min_) angle += 2 * M_PI;
            while (angle > angle_max_) angle -= 2 * M_PI;
            
            // Find corresponding range index
            int index = static_cast<int>((angle - angle_min_) / angle_increment_);
            if (index >= 0 && index < num_ranges) {
                // Update range if this point is closer
                if (range < scan_msg.ranges[index]) {
                    scan_msg.ranges[index] = range;
                }
            }
        }
        
        // Publish scan
        scan_pub_.publish(scan_msg);
        
        // Log statistics
        int valid_ranges = 0;
        for (const auto& range : scan_msg.ranges) {
            if (!std::isinf(range)) {
                valid_ranges++;
            }
        }
        
        ROS_DEBUG("Published LaserScan: %d/%d valid ranges", valid_ranges, num_ranges);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_to_laserscan_simple");
    
    PointCloudToLaserScanSimple converter;
    
    ros::spin();
    
    return 0;
}
