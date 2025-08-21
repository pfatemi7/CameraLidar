#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class CameraLidarFusion {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // Publishers
    ros::Publisher fused_cloud_pub_;
    ros::Publisher colored_cloud_pub_;
    ros::Publisher debug_image_pub_;
    
    // Subscribers
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
    message_filters::Subscriber<sensor_msgs::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_sub_;
    
    // Synchronizer
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::PointCloud2, sensor_msgs::Image, sensor_msgs::CameraInfo> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;
    
    // TF
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
    
    // Calibration parameters
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    cv::Mat rotation_matrix_;
    cv::Mat translation_vector_;
    
    // Processing parameters
    double voxel_leaf_size_;
    double z_min_, z_max_;
    int sor_mean_k_;
    double sor_std_dev_;
    
    // Point cloud processing
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;
    pcl::PassThrough<pcl::PointXYZ> pass_through_filter_;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_filter_;

public:
    CameraLidarFusion() : private_nh_("~") {
        // Initialize parameters
        private_nh_.param("voxel_leaf_size", voxel_leaf_size_, 0.03);
        private_nh_.param("z_min", z_min_, -0.2);
        private_nh_.param("z_max", z_max_, 2.5);
        private_nh_.param("sor_mean_k", sor_mean_k_, 20);
        private_nh_.param("sor_std_dev", sor_std_dev_, 1.0);
        
        // Initialize publishers
        fused_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/fused/cloud", 1);
        colored_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/fused/colored_cloud", 1);
        debug_image_pub_ = nh_.advertise<sensor_msgs::Image>("/fused/debug_image", 1);
        
        // Initialize subscribers
        cloud_sub_.subscribe(nh_, "/unilidar/cloud", 1);
        image_sub_.subscribe(nh_, "/zed/left/image_rect_color", 1);
        camera_info_sub_.subscribe(nh_, "/zed/left/camera_info", 1);
        
        // Initialize synchronizer
        sync_.reset(new Sync(SyncPolicy(10), cloud_sub_, image_sub_, camera_info_sub_));
        sync_->registerCallback(boost::bind(&CameraLidarFusion::callback, this, _1, _2, _3));
        
        // Initialize filters
        voxel_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        pass_through_filter_.setFilterFieldName("z");
        pass_through_filter_.setFilterLimits(z_min_, z_max_);
        sor_filter_.setMeanK(sor_mean_k_);
        sor_filter_.setStddevMulThresh(sor_std_dev_);
        
        // Load calibration (you'll need to provide these values)
        loadCalibration();
        
        // Publish static transform from LiDAR to camera
        publishStaticTransform();
        
        ROS_INFO("Camera-LiDAR fusion node initialized");
    }
    
    void loadCalibration() {
        // These values should be obtained from calibration
        // For now, using default values - you'll need to calibrate your setup
        camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
        camera_matrix_.at<double>(0, 0) = 1000.0; // fx
        camera_matrix_.at<double>(1, 1) = 1000.0; // fy
        camera_matrix_.at<double>(0, 2) = 640.0;  // cx
        camera_matrix_.at<double>(1, 2) = 480.0;  // cy
        
        dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
        
        // Rotation and translation from LiDAR to camera
        rotation_matrix_ = cv::Mat::eye(3, 3, CV_64F);
        translation_vector_ = cv::Mat::zeros(3, 1, CV_64F);
        translation_vector_.at<double>(0, 0) = 0.1; // 10cm offset in X
    }
    
    void publishStaticTransform() {
        geometry_msgs::TransformStamped transform;
        transform.header.stamp = ros::Time::now();
        transform.header.frame_id = "camera_link";
        transform.child_frame_id = "lidar_link";
        
        // Set transform based on calibration
        transform.transform.translation.x = translation_vector_.at<double>(0, 0);
        transform.transform.translation.y = translation_vector_.at<double>(1, 0);
        transform.transform.translation.z = translation_vector_.at<double>(2, 0);
        
        // Convert rotation matrix to quaternion
        cv::Mat R = rotation_matrix_;
        tf2::Matrix3x3 tf_rotation(
            R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
            R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
            R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2)
        );
        tf2::Quaternion q;
        tf_rotation.getRotation(q);
        
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
        
        static_tf_broadcaster_.sendTransform(transform);
    }
    
    void callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
                  const sensor_msgs::Image::ConstPtr& image_msg,
                  const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg) {
        
        // Process point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud);
        
        // Apply filters
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        // Voxel downsampling
        voxel_filter_.setInputCloud(cloud);
        voxel_filter_.filter(*filtered_cloud);
        
        // Z-axis filtering
        pass_through_filter_.setInputCloud(filtered_cloud);
        pass_through_filter_.filter(*filtered_cloud);
        
        // Statistical outlier removal
        sor_filter_.setInputCloud(filtered_cloud);
        sor_filter_.filter(*filtered_cloud);
        
        // Publish filtered cloud
        sensor_msgs::PointCloud2 filtered_msg;
        pcl::toROSMsg(*filtered_cloud, filtered_msg);
        filtered_msg.header = cloud_msg->header;
        fused_cloud_pub_.publish(filtered_msg);
        
        // Color the point cloud with camera data
        colorPointCloud(filtered_cloud, image_msg, camera_info_msg);
        
        ROS_INFO("Processed cloud with %lu points", filtered_cloud->size());
    }
    
    void colorPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                        const sensor_msgs::Image::ConstPtr& image_msg,
                        const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg) {
        
        // Convert ROS image to OpenCV
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        
        // Create colored point cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        colored_cloud->points.resize(cloud->size());
        
        cv::Mat debug_image = cv_ptr->image.clone();
        
        for (size_t i = 0; i < cloud->size(); ++i) {
            const pcl::PointXYZ& pt = cloud->points[i];
            
            // Transform point from LiDAR to camera coordinates
            cv::Mat lidar_point(3, 1, CV_64F);
            lidar_point.at<double>(0, 0) = pt.x;
            lidar_point.at<double>(1, 0) = pt.y;
            lidar_point.at<double>(2, 0) = pt.z;
            
            cv::Mat camera_point = rotation_matrix_ * lidar_point + translation_vector_;
            
            // Project to image plane
            if (camera_point.at<double>(2, 0) > 0) { // Point is in front of camera
                cv::Mat image_point = camera_matrix_ * camera_point;
                double u = image_point.at<double>(0, 0) / image_point.at<double>(2, 0);
                double v = image_point.at<double>(1, 0) / image_point.at<double>(2, 0);
                
                // Check if point projects within image bounds
                if (u >= 0 && u < cv_ptr->image.cols && v >= 0 && v < cv_ptr->image.rows) {
                    cv::Vec3b color = cv_ptr->image.at<cv::Vec3b>(v, u);
                    
                    pcl::PointXYZRGB& colored_pt = colored_cloud->points[i];
                    colored_pt.x = pt.x;
                    colored_pt.y = pt.y;
                    colored_pt.z = pt.z;
                    colored_pt.r = color[2];
                    colored_pt.g = color[1];
                    colored_pt.b = color[0];
                    
                    // Draw point on debug image
                    cv::circle(debug_image, cv::Point(u, v), 2, cv::Scalar(0, 255, 0), -1);
                } else {
                    // Point outside image - use default color
                    pcl::PointXYZRGB& colored_pt = colored_cloud->points[i];
                    colored_pt.x = pt.x;
                    colored_pt.y = pt.y;
                    colored_pt.z = pt.z;
                    colored_pt.r = 128;
                    colored_pt.g = 128;
                    colored_pt.b = 128;
                }
            } else {
                // Point behind camera - use default color
                pcl::PointXYZRGB& colored_pt = colored_cloud->points[i];
                colored_pt.x = pt.x;
                colored_pt.y = pt.y;
                colored_pt.z = pt.z;
                colored_pt.r = 128;
                colored_pt.g = 128;
                colored_pt.b = 128;
            }
        }
        
        // Publish colored cloud
        sensor_msgs::PointCloud2 colored_msg;
        pcl::toROSMsg(*colored_cloud, colored_msg);
        colored_msg.header = cloud->header;
        colored_cloud_pub_.publish(colored_msg);
        
        // Publish debug image
        sensor_msgs::ImagePtr debug_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", debug_image).toImageMsg();
        debug_msg->header = image_msg->header;
        debug_image_pub_.publish(debug_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_lidar_fusion_node");
    
    CameraLidarFusion fusion_node;
    
    ros::spin();
    
    return 0;
}
