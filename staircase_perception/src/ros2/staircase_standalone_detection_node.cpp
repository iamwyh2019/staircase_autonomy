#include "staircase_perception/ros2/staircase_standalone_detection_node.hpp"

// Constructor for the ROS 2 Node
StaircaseStandaloneDetectionNode::StaircaseStandaloneDetectionNode() : Node("staircase_standalone_detection_node")
{
    // === 1. Declare and Get Parameters ===    

    this->declare_parameter<std::string>("staircase_detection_topics.point_cloud_topic", "depth_camera/points");
    this->declare_parameter<std::string>("staircase_detection_topics.staircase_detections_topic", "staircase_standalone_detection_node/detections");
    this->declare_parameter<std::string>("staircase_detection_topics.staircase_markers_topic", "staircase_standalone_detection_node/markers");


    this->get_parameter("staircase_detection_topics.point_cloud_topic", pointcloud_topic_);
    this->get_parameter("staircase_detection_topics.staircase_detections_topic", staircase_detections_topic_);
    this->get_parameter("staircase_detection_topics.staircase_markers_topic", staircase_markers_topic_);
    
    // ROS Node parameters
    this->get_parameter("use_sim_time", simulation_);

    this->declare_parameter<double>("ros_rate", 10.0);
    this->declare_parameter<bool>("debug", false);
    this->declare_parameter<std::string>("camera_frame_id", "base_link");
    this->declare_parameter<std::string>("robot_topics_prefix", "");

    this->get_parameter("ros_rate", ros_rate_);
    this->get_parameter("debug", debug_);
    this->get_parameter("camera_frame_id", camera_frame_id_);
    this->get_parameter("robot_topics_prefix", robots_topics_prefix_);

    // Point cloud pre-processing params
    this->declare_parameter<double>("stair_pointcloud.leaf_size", 0.025);
    this->declare_parameter<double>("stair_pointcloud.min_range_y", -3.0);
    this->declare_parameter<double>("stair_pointcloud.max_range_y", 3.0);
    this->declare_parameter<double>("stair_pointcloud.min_range_x", 0.0);
    this->declare_parameter<double>("stair_pointcloud.max_range_x", 4.0);
    this->declare_parameter<double>("stair_pointcloud.min_range_z", -2.5);
    this->declare_parameter<double>("stair_pointcloud.max_range_z", 1.0);

    this->get_parameter("stair_pointcloud.leaf_size", leaf_size_);
    this->get_parameter("stair_pointcloud.min_range_y", min_range_y_);
    this->get_parameter("stair_pointcloud.max_range_y", max_range_y_);
    this->get_parameter("stair_pointcloud.min_range_x", min_range_x_);
    this->get_parameter("stair_pointcloud.max_range_x", max_range_x_);
    this->get_parameter("stair_pointcloud.min_range_z", min_range_z_);
    this->get_parameter("stair_pointcloud.max_range_z", max_range_z_);

    // Staircase Detector Parameters
    this->declare_parameter<double>("stair_detector.angle_resolution", 1.0);
    this->declare_parameter<double>("stair_detector.robot_height", 0.5);
    this->declare_parameter<int>("stair_detector.min_stair_count", 3.0);
    this->declare_parameter<double>("stair_detector.stair_slope_min", 0.35);
    this->declare_parameter<double>("stair_detector.stair_slope_max", 1.22);
    this->declare_parameter<bool>("stair_detector.use_ramp_detection", true);
    this->declare_parameter<double>("stair_detector.initialization_range", 0.5);
    this->declare_parameter<double>("stair_detector.ground_height_buffer", 0.05);

    this->declare_parameter<double>("stair_detector.min_stair_width", 0.75);
    this->declare_parameter<double>("stair_detector.min_stair_height", 0.1);
    this->declare_parameter<double>("stair_detector.max_stair_height", 0.25);
    this->declare_parameter<double>("stair_detector.mix_stair_depth", 0.125);
    this->declare_parameter<double>("stair_detector.max_stair_depth", 0.35);
    this->declare_parameter<double>("stair_detector.max_stair_curvature", 0.55);

    this->get_parameter("stair_detector.angle_resolution", detector_params_.angle_resolution);
    this->get_parameter("stair_detector.robot_height", detector_params_.robot_height);
    this->get_parameter("stair_detector.min_stair_count", detector_params_.min_stair_count);
    this->get_parameter("stair_detector.stair_slope_min", detector_params_.stair_slope_min);
    this->get_parameter("stair_detector.stair_slope_max", detector_params_.stair_slope_max);
    this->get_parameter("stair_detector.use_ramp_detection", detector_params_.use_ramp_detection);
    this->get_parameter("stair_detector.initialization_range", detector_params_.initialization_range);
    this->get_parameter("stair_detector.ground_height_buffer", detector_params_.ground_height_buffer);

    this->get_parameter<double>("stair_detector.min_stair_width", detector_params_.min_stair_width);
    this->get_parameter<double>("stair_detector.min_stair_height", detector_params_.min_stair_height);
    this->get_parameter<double>("stair_detector.max_stair_height", detector_params_.max_stair_height);
    this->get_parameter<double>("stair_detector.mix_stair_depth", detector_params_.min_stair_depth);
    this->get_parameter<double>("stair_detector.max_stair_depth", detector_params_.max_stair_depth);
    this->get_parameter<double>("stair_detector.max_stair_curvature", detector_params_.max_stair_curvature);

    detector_params_.leaf_size = leaf_size_;
    detector_params_.x_max = max_range_x_;
    detector_params_.x_min = min_range_x_;
    detector_params_.y_max = max_range_y_;
    detector_params_.y_min = min_range_y_;
    detector_params_.z_max = max_range_z_;
    detector_params_.z_min = min_range_z_;
    
    // Line Extractor parameters
    this->declare_parameter<double>("stair_line_extractor.bearing_variance", 0.0001);
    this->declare_parameter<double>("stair_line_extractor.range_variance", 0.001);
    this->declare_parameter<double>("stair_line_extractor.z_variance", 0.0004);
    this->declare_parameter<double>("stair_line_extractor.least_sq_angle_thresh", 0.05);
    this->declare_parameter<double>("stair_line_extractor.least_sq_radius_thresh", 0.075);
    this->declare_parameter<double>("stair_line_extractor.max_line_gap", 0.2);
    this->declare_parameter<double>("stair_line_extractor.min_range", 0.1);
    this->declare_parameter<double>("stair_line_extractor.max_range", 5.0);
    this->declare_parameter<double>("stair_line_extractor.min_split_distance", 0.2);
    this->declare_parameter<double>("stair_line_extractor.outlier_distance", 0.2);
    this->declare_parameter<int>("stair_line_extractor.min_line_points", 7);

    int min_line_points_int;
    this->get_parameter("stair_line_extractor.bearing_variance", line_params_.bearing_var);
    this->get_parameter("stair_line_extractor.range_variance", line_params_.range_var);
    this->get_parameter("stair_line_extractor.z_variance", line_params_.z_var);
    this->get_parameter("stair_line_extractor.least_sq_angle_thresh", line_params_.least_sq_angle_thresh);
    this->get_parameter("stair_line_extractor.least_sq_radius_thresh", line_params_.least_sq_radius_thresh);
    this->get_parameter("stair_line_extractor.max_line_gap", line_params_.max_line_gap);
    this->get_parameter("stair_line_extractor.min_range", line_params_.min_range);
    this->get_parameter("stair_line_extractor.max_range", line_params_.max_range);
    this->get_parameter("stair_line_extractor.min_split_distance", line_params_.min_split_dist);
    this->get_parameter("stair_line_extractor.outlier_distance", line_params_.outlier_dist);
    this->get_parameter("stair_line_extractor.min_line_points", min_line_points_int);
    line_params_.min_line_points = min_line_points_int;

    // === 2. Initialize Member Variables and Objects ===
    // This section is largely the same, just initializing variables.

    lasercloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    lasercloud_local_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    lasercloud_cropped_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    lasercloud_processed_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    lasercloud_processed2_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());

    lasercloud_->reserve(300000);
    long max_points_in_voxel_grid = round(((max_range_x_ - min_range_x_) / leaf_size_) * ((max_range_y_ - min_range_y_) / leaf_size_) * ((max_range_z_ - min_range_z_) / leaf_size_));
    lasercloud_cropped_->reserve(max_points_in_voxel_grid);
    lasercloud_local_->reserve(max_points_in_voxel_grid);

    new_lasercloud_ = false;
    // Set parameters for the Staircase Detector and the Line Extractor
    detector_ = StairDetector(detector_params_, line_params_);

    // === 3. Create Publishers, Subscribers, and Timers ===
    // QoS profile for sensor data
    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(robots_topics_prefix_ + "/" + pointcloud_topic_, sensor_qos, std::bind(&StaircaseStandaloneDetectionNode::PointCloudHandler, this, _1));
    
    detection_pub_ = this->create_publisher<staircase_msgs::msg::StaircaseMeasurement>(staircase_detections_topic_, 10);
    detection_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(staircase_markers_topic_, 10);

    detection_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / ros_rate_),
        std::bind(&StaircaseStandaloneDetectionNode::PerceiveStaircases, this));

    if (debug_)
    {
        process_cl_pub1_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/staircase_estimation_robot_node/stair_debug_cloud1", 10);
        process_cl_pub2_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/staircase_estimation_robot_node/stair_debug_cloud2", 10);
        line_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/staircase_estimation_robot_node/stair_debug_line_marker", 10);
    }

    RCLCPP_INFO(this->get_logger(), "\033[1;35m Staircase Node started, Debug: %d, Sim Time: %d \033[0m", debug_, simulation_);
}

void StaircaseStandaloneDetectionNode::PerceiveStaircases()
{   
    if (new_lasercloud_)
    {   

        
        detector_.setPointCloudAndOdometry(lasercloud_local_);
        stair_detected_ = detector_.detectStaircase(staircase_up_, staircase_down_);
                
        if (stair_detected_ != stair_utility::StaircaseDetectorResult::NoStairsDetected)
        {   
            if(stair_detected_ == stair_utility::StaircaseDetectorResult::StairsDetectedUp || stair_detected_ == stair_utility::StaircaseDetectorResult::StairsDetectedBoth){                
                publishStaircaseDetection(staircase_up_, true);
            }
            if(stair_detected_ == stair_utility::StaircaseDetectorResult::StairsDetectedDown || stair_detected_ == stair_utility::StaircaseDetectorResult::StairsDetectedBoth){
                publishStaircaseDetection(staircase_down_, false);
            }
        }

        new_lasercloud_ = false;

        if (debug_)
        {   
            // Publish input voxel grid     
            sensor_msgs::msg::PointCloud2 debug_cloud1;
            pcl::toROSMsg(*lasercloud_local_, debug_cloud1);
            debug_cloud1.header.stamp = this->now();
            debug_cloud1.header.frame_id = camera_frame_id_;
            process_cl_pub1_->publish(debug_cloud1);

            /// Publish processed cloud. Stage - 1 for top-view, 2 for cylindrical-view
            detector_.getProcessedCloud(lasercloud_processed_, 2);            
            sensor_msgs::msg::PointCloud2 debug_cloud2;
            pcl::toROSMsg(*lasercloud_processed_, debug_cloud2);
            debug_cloud2.header.stamp = this->now();
            debug_cloud2.header.frame_id = camera_frame_id_;
            process_cl_pub2_->publish(debug_cloud2);

            publishLineMarkers();
        }
    }

}


void StaircaseStandaloneDetectionNode::PointCloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{

        lasercloud_->clear();
        pcl::fromROSMsg(*msg, *lasercloud_);
        
        box_filter_.setMin(Eigen::Vector4f(min_range_x_, min_range_y_, min_range_z_, -1.0));
        box_filter_.setMax(Eigen::Vector4f(max_range_x_, max_range_y_, max_range_z_, 1.0));
        box_filter_.setInputCloud(lasercloud_);
        box_filter_.filter(*lasercloud_cropped_);

        voxel_grid_filter_.setInputCloud(lasercloud_cropped_);
        voxel_grid_filter_.setLeafSize(leaf_size_, leaf_size_, leaf_size_);  
        voxel_grid_filter_.filter(*lasercloud_local_);
        
        new_lasercloud_ = true;

}


// Placeholder implementations for the publish functions
void StaircaseStandaloneDetectionNode::publishStaircaseDetection(const stair_utility::StaircaseMeasurement& measurement, bool ascending) {
    
    // Publish the StaircaseMeasurement message
    auto msg = std::make_unique<staircase_msgs::msg::StaircaseMeasurement>();    

    msg->stair_count = measurement.stair_count;
    msg->frame_id = camera_frame_id_;
    
    for(const auto& step : measurement.steps) {
        geometry_msgs::msg::Point start, end;
        
        start.x = step.start_p.x();
        start.y = step.start_p.y();
        start.z = step.start_p.z();
        end.x = step.end_p.x();
        end.y = step.end_p.y();
        end.z = step.end_p.z();
                
        msg->steps_start_p.push_back(start);
        msg->steps_end_p.push_back(end);
        msg->step_lengths.push_back(step.step_width);
        msg->step_radii.push_back(step.line_polar_form(0));
        msg->step_angles.push_back(step.line_polar_form(1));
        msg->r_r_covariance.push_back(step.step_covariance(0, 0));
        msg->r_t_covariance.push_back(step.step_covariance(1, 0));
        msg->t_t_covariance.push_back(step.step_covariance(1, 1));
        msg->z_covariance.push_back(step.step_covariance(2, 2));
    }
    msg->is_ascending = ascending;
    detection_pub_->publish(std::move(msg));

    // Publish visualization markers
    visualization_msgs::msg::MarkerArray marker_array_msg;
    int stair_id_ros = ascending ? 1 : 2;
    int current_step_count = measurement.stair_count;

    int old_step_count = 0;
    if (prev_step_counts_for_markers_.count(stair_id_ros)) {
        old_step_count = prev_step_counts_for_markers_[stair_id_ros];
    }

    for (int i = 0; i < current_step_count; ++i) {
        const auto& current_step = measurement.steps[i];
        visualization_msgs::msg::Marker step_marker;

        step_marker.header.frame_id = camera_frame_id_;
        step_marker.header.stamp = this->get_clock()->now();
        step_marker.ns = "measurement_cube_" + std::to_string(stair_id_ros);
        step_marker.id = (stair_id_ros * 100) + i;
        step_marker.action = visualization_msgs::msg::Marker::ADD;
        step_marker.type = visualization_msgs::msg::Marker::CUBE;
        step_marker.lifetime = rclcpp::Duration::from_seconds(1000.0);

        step_marker.color.r = 0.028f;
        step_marker.color.g = 0.681f;
        step_marker.color.b = 0.960f;
        step_marker.color.a = 1.0f;

        // Points are already in camera_frame_id_
        const auto& start_p = current_step.start_p;
        const auto& end_p = current_step.end_p;

        double center_x = (start_p.x() + end_p.x()) / 2.0;
        double center_y = (start_p.y() + end_p.y()) / 2.0;
        double center_z = (start_p.z() + end_p.z()) / 2.0;

        double stairs_orient = std::atan2(end_p.y() - start_p.y(), end_p.x() - start_p.x());
        tf2::Quaternion quat_tf;
        quat_tf.setRPY(0, 0, stairs_orient);
        step_marker.pose.orientation = tf2::toMsg(quat_tf);

        double height, depth;
        if (i == current_step_count - 1 && current_step_count > 1) {
            const auto& prev_step = measurement.steps[i-1];
            height = std::abs(start_p.z() - prev_step.start_p.z());
            depth = std::sqrt(std::pow(start_p.x() - prev_step.start_p.x(), 2) + std::pow(start_p.y() - prev_step.start_p.y(), 2));
        } else if (current_step_count > 1) {
            const auto& next_step = measurement.steps[i+1];
            height = std::abs(next_step.start_p.z() - start_p.z());
            depth = std::sqrt(std::pow(next_step.start_p.x() - start_p.x(), 2) + std::pow(next_step.start_p.y() - start_p.y(), 2));
        } else { // Handle case with only one step
            height = 0.1;
            depth = 0.2;
        }
        // Ensure height and depth are not zero to avoid invisible markers
        if (height < 0.01) height = 0.01;
        if (depth < 0.01) depth = 0.01;


        step_marker.pose.position.x = center_x;
        step_marker.pose.position.y = center_y;
        step_marker.pose.position.z = center_z - height / 2.0; // Position at the base of the tread

        step_marker.scale.x = current_step.step_width > 0.01 ? current_step.step_width : 0.01;
        step_marker.scale.y = depth;
        step_marker.scale.z = height;

        marker_array_msg.markers.push_back(step_marker);
    }

    // Delete old markers if the number of stairs has decreased
    if (old_step_count > current_step_count) {
        for (int i = current_step_count; i < old_step_count; ++i) {
            visualization_msgs::msg::Marker delete_marker;
            delete_marker.header.frame_id = camera_frame_id_;
            delete_marker.header.stamp = this->get_clock()->now();
            delete_marker.ns = "measurement_cube_" + std::to_string(stair_id_ros);
            delete_marker.id = (stair_id_ros * 100) + i;
            delete_marker.action = visualization_msgs::msg::Marker::DELETE;
            marker_array_msg.markers.push_back(delete_marker);
        }
    }

    if (!marker_array_msg.markers.empty()) {
        detection_marker_pub_->publish(marker_array_msg);
    }
    prev_step_counts_for_markers_[stair_id_ros] = current_step_count;
}

void StaircaseStandaloneDetectionNode::publishLineMarkers() {
    auto marker_array_msg = std::make_unique<visualization_msgs::msg::MarkerArray>();
    
    visualization_msgs::msg::Marker marker_up, marker_down, marker_ground;
    
    // Configure 'up' marker
    marker_up.header.frame_id = camera_frame_id_;

    marker_up.header.stamp = this->now();
    marker_up.ns = "line_extraction_marker_up";
    marker_up.id = 1;
    marker_up.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker_up.action = visualization_msgs::msg::Marker::ADD;
    marker_up.scale.x = 0.05;
    marker_up.color.b = 1.0;
    marker_up.color.a = 1.0;
    
    // Configure 'down' marker
    marker_down = marker_up; // Copy properties
    marker_down.ns = "line_extraction_marker_down";
    marker_down.id = 2;
    marker_down.color.r = 1.0;
    marker_down.color.g = 0.0;
    marker_down.color.b = 0.0;

    // Configure 'ground' marker
    marker_ground = marker_up; // Copy properties
    marker_ground.ns = "line_extraction_marker_ground";
    marker_ground.id = 3; // Different ID
    marker_ground.color.r = 0.0;
    marker_ground.color.g = 1.0;
    marker_ground.color.b = 0.0;

    // Get all lines above ground
    const auto& linesAbove = detector_.getDetectedLinesAbove();
    for (const auto& lines : linesAbove) {
        for(size_t i = 0; i < lines->size(); i++){
            geometry_msgs::msg::Point p_start, p_end;
            p_start.x = lines->at(i).line_start[0];
            p_start.y = lines->at(i).line_start[1];
            p_start.z = lines->at(i).line_start[2];
            p_end.x = lines->at(i).line_end[0];
            p_end.y = lines->at(i).line_end[1];
            p_end.z = lines->at(i).line_end[2];

            marker_up.points.push_back(p_start);
            marker_up.points.push_back(p_end);
        }
    }
    marker_array_msg->markers.push_back(marker_up);

    // Get all lines below ground
    const auto& linesBelow = detector_.getDetectedLinesBelow();
    for (const auto& lines : linesBelow) {
        for(size_t i = 0; i < lines->size(); i++){
            geometry_msgs::msg::Point p_start, p_end;
            p_start.x = lines->at(i).line_start[0];
            p_start.y = lines->at(i).line_start[1];
            p_start.z = lines->at(i).line_start[2];
            p_end.x = lines->at(i).line_end[0];
            p_end.y = lines->at(i).line_end[1];
            p_end.z = lines->at(i).line_end[2];

            marker_down.points.push_back(p_start);
            marker_down.points.push_back(p_end);
        }
    }
    marker_array_msg->markers.push_back(marker_down);

    // Get all lines on ground
    const auto& linesGround = detector_.getDetectedLinesGround();
    for (const auto& lines : linesGround) {
        for(size_t i = 0; i < lines->size(); i++){
            geometry_msgs::msg::Point p_start, p_end;
            p_start.x = lines->at(i).line_start[0];
            p_start.y = lines->at(i).line_start[1];
            p_start.z = lines->at(i).line_start[2];
            p_end.x = lines->at(i).line_end[0];
            p_end.y = lines->at(i).line_end[1];
            p_end.z = lines->at(i).line_end[2];
      
            marker_ground.points.push_back(p_start);
            marker_ground.points.push_back(p_end);
        }
    }
    marker_array_msg->markers.push_back(marker_ground);

    line_marker_pub_->publish(std::move(marker_array_msg));
}

// Main function to run the node
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StaircaseStandaloneDetectionNode>();
    RCLCPP_INFO(node->get_logger(), "\033[1;35m Starting Staircase ROS 2 Detection Node \033[0m");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
