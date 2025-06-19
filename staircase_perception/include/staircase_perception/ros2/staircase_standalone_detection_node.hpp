#ifndef _STAIRCASE_STANDALONE_DETECTION_NODE_H_
#define _STAIRCASE_STANDALONE_DETECTION_NODE_H_

// ROS 2 Headers
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// TF2 Headers
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2/transform_datatypes.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/convert.h"

// PCL Headers
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include "pcl_ros/transforms.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>

#include "staircase_msgs/msg/staircase_measurement.hpp"

// Project-specific Headers
#include "staircase_perception/utils/stair_utilities.hpp"
#include "staircase_perception/core/stair_detector.hpp"

// C++ Standard Libraries
#include <string>
#include <iostream>
#include <algorithm>
#include <vector>
#include <stdlib.h>
#include <chrono>
#include <unordered_map>

// Using statements for easier access to message types
using std::placeholders::_1;

class StaircaseStandaloneDetectionNode : public rclcpp::Node
{
    public:
        StaircaseStandaloneDetectionNode();

    private:
        // ROS 2 Subscribers
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

        // ROS 2 Publishers
        rclcpp::Publisher<staircase_msgs::msg::StaircaseMeasurement>::SharedPtr detection_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr detection_marker_pub_;
        
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr process_cl_pub1_; 
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr process_cl_pub2_; 
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr line_marker_pub_;


        // ROS 2 Timers
        rclcpp::TimerBase::SharedPtr detection_timer_;

        // Main perception loop
        void PerceiveStaircases();

        // Publishing methods
        void publishStaircaseDetection(const stair_utility::StaircaseMeasurement& measurement, bool ascending);
        void publishLineMarkers();

        // Callback methods
        void PointCloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        // Helper method
        inline void averagePoint(const geometry_msgs::msg::Point& A, const geometry_msgs::msg::Point& B, geometry_msgs::msg::Point& C)
        {
            C.x = (A.x + B.x) / 2;
            C.y = (A.y + B.y) / 2;
            C.z = (A.z + B.z) / 2;
        }

        // Member Variables
        // Topic Names
        std::string pointcloud_topic_;
        std::string staircase_detections_topic_, staircase_markers_topic_;

        // ROSNode Variables
        std::string camera_frame_id_, robot_name_;

        double ros_rate_; // This is used to set the timer frequency
        bool debug_, simulation_;

        std::string robots_topics_prefix_;

        // Lidar Variables
        rclcpp::Time lasercloud_time_;
        double min_range_x_, max_range_x_, min_range_y_, max_range_y_, min_range_z_, max_range_z_;
        double leaf_size_;

        pcl::PointCloud<pcl::PointXYZI>::Ptr lasercloud_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr lasercloud_local_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr lasercloud_cropped_;

        bool new_lasercloud_;

        pcl::PointCloud<pcl::PointXYZI>::Ptr lasercloud_processed_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr lasercloud_processed2_;

        // PCL Cloud Processing Objects
        pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter_;
        pcl::CropBox<pcl::PointXYZI> box_filter_;

        // Staircase Detector Objects and Parameters
        stair_utility::LineExtractorParams line_params_;
        stair_utility::StaircaseDetectorParams detector_params_;
        StairDetector detector_;

        stair_utility::StaircaseDetectorResult stair_detected_;
        stair_utility::StaircaseMeasurement staircase_up_, staircase_down_;
        std::unordered_map<int, int> prev_step_counts_for_markers_; // For managing marker deletion

};

#endif // _STAIRCASE_ROBOTNODE_H_
