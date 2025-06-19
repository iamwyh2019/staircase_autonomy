#ifndef _SIMPLE_CLUTTER_SEGMENTATION_NODE_HPP_
#define _SIMPLE_CLUTTER_SEGMENTATION_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "staircase_msgs/msg/staircase_msg.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_parallel_plane.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>

#include "pcl_ros/transforms.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include "staircase_perception/utils/stair_utilities.hpp"

class SimpleClutterSegmentationNode : public rclcpp::Node
{
public:
    SimpleClutterSegmentationNode();

private:
    // Main processing loop triggered by a timer
    void segment_pointclouds();

    // --- ROS 2 Components ---
    rclcpp::Subscription<staircase_msgs::msg::StaircaseMsg>::SharedPtr staircse_estimate_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr global_tf_sub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr segmented_cloud_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    // Callbacks to cache the latest data from each topic
    void staircase_callback(const staircase_msgs::msg::StaircaseMsg::ConstSharedPtr& msg);
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
    void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
    void global_tf_handler(const geometry_msgs::msg::TransformStamped::SharedPtr msg);

    std::string pointcloud_topic_, odom_topic_, global_tf_topic_;
    std::string staircase_estimates_topic_, segmented_stair_surface_topic_;
    std::string global_frame_id_, body_frame_id_, odom_frame_id_, robot_name_;

    double processing_rate_; 
    bool transform_detections_to_global_, simulation_, combine_scans_for_pointcloud_;
    std::string robots_topics_prefix_;

    // Odometry Variables
    double vehicle_roll_, vehicle_pitch_, vehicle_yaw_;
    double vehicle_x_, vehicle_y_, vehicle_z_;
    geometry_msgs::msg::Pose vehicle_pose_;
    stair_utility::RobotPositionInfo robot_pos_;

    // Transform Variables (using tf2 types)
    geometry_msgs::msg::TransformStamped global_to_odom_tf_stamped_;
    tf2::Transform robot_pose_transform_; // tf2::Transform for calculations
    tf2::Transform global_tf_;         
    tf2::Transform odom_to_robot_tf_, global_to_odom_tf_;  
    
    // Core segmentation logic
    void segment_stair_surfaces(
        const staircase_msgs::msg::StaircaseMsg::ConstSharedPtr& stair_msg, 
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& point_cloud_in, 
        pcl::PointCloud<pcl::PointXYZI>::Ptr& segmented_stair_surfaces);
    
    // Lidar Variables
    rclcpp::Time lasercloud_time_;
    double min_range_x_, max_range_x_, min_range_y_, max_range_y_, min_range_z_, max_range_z_;
    double leaf_size_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr lasercloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr lasercloud_stacked_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr lasercloud_local_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr lasercloud_cropped_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr lasercloud_transformed_;
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr lasercloud_stair_surface_;
    
    staircase_msgs::msg::StaircaseMsg::ConstSharedPtr latest_stair_msg_;

    bool new_stair_data_ = false;
    bool new_cloud_data_ = false;
    bool new_odom_data_ = false;

    // Parameters for segmentation logic
    double max_surface_thickness_;

    // Parameters for point cloud pre-processing (placeholders)
    // PCL Cloud Processing Objects
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter_;
    pcl::CropBox<pcl::PointXYZI> box_filter_;
    double min_x_, max_x_, min_y_, max_y_, min_z_, max_z_; // Variables for Box filter
};

#endif // simple_clutter_segmentation_node_HPP_
