#include "staircase_perception/ros2/simple_clutter_segmentation_node.hpp"

SimpleClutterSegmentationNode::SimpleClutterSegmentationNode() : Node("simple_clutter_segmentation_node")
{
    // --- ROS 2 Parameter Declaration
    this->declare_parameter<std::string>("staircase_perception_params.global_frame_id", "global");

    this->get_parameter("staircase_perception_params.global_frame_id", global_frame_id_);
    this->get_parameter("use_sim_time", simulation_);

    this->declare_parameter<double>("processing_rate", 10.0);

    this->declare_parameter<std::string>("body_frame_id", "base_link");
    this->declare_parameter<std::string>("odom_frame_id", "odom");
    this->declare_parameter<std::string>("robot_name", "robot");

    this->declare_parameter<bool>("transform_detections_to_global", true);
    this->declare_parameter<std::string>("robot_topics_prefix", "robot");

    this->get_parameter("processing_rate", processing_rate_);
    this->get_parameter("body_frame_id", body_frame_id_);
    this->get_parameter("odom_frame_id", odom_frame_id_);
    this->get_parameter("robot_name", robot_name_);

    this->get_parameter("transform_detections_to_global", transform_detections_to_global_);
    this->get_parameter("robot_topics_prefix", robots_topics_prefix_);

    this->declare_parameter<std::string>("staircase_perception_topics.staircase_estimates_topic", "/staircase_estimation_robot_node/staircase_estimates");
    this->declare_parameter<std::string>("staircase_perception_topics.clutter_point_cloud_in_topic", "registered_depth_camera_points");
    this->declare_parameter<std::string>("staircase_perception_topics.odometry_topic", "odometry");
    this->declare_parameter<std::string>("staircase_perception_topics.clutter_point_cloud_out_topic", "/simple_clutter_segmentation_node/segmented_stair_surface");
    this->declare_parameter("staircase_perception_topics.global_transform_topic", "transform");

    this->get_parameter("staircase_perception_topics.staircase_estimates_topic", staircase_estimates_topic_);
    this->get_parameter("staircase_perception_topics.clutter_point_cloud_in_topic", pointcloud_topic_);
    this->get_parameter("staircase_perception_topics.odometry_topic", odom_topic_);
    this->get_parameter("staircase_perception_topics.clutter_point_cloud_out_topic", segmented_stair_surface_topic_);
    this->get_parameter("staircase_perception_topics.global_transform_topic", global_tf_topic_);

    // Segmentation parameters
    this->declare_parameter<double>("segmentation_params.max_surface_thickness", 0.05);
    this->declare_parameter<bool>("segmentation_params.combine_pointcloud_scans", true);
    
    this->get_parameter("segmentation_params.combine_pointcloud_scans", combine_scans_for_pointcloud_);
    this->get_parameter("segmentation_params.max_surface_thickness", max_surface_thickness_);

    this->declare_parameter<double>("segmentation_params.leaf_size", 0.05);
    this->declare_parameter<double>("segmentation_params.min_range_y", -4.0);
    this->declare_parameter<double>("segmentation_params.max_range_y", 4.0);
    this->declare_parameter<double>("segmentation_params.min_range_x", -4.0);
    this->declare_parameter<double>("segmentation_params.max_range_x", 4.0);
    this->declare_parameter<double>("segmentation_params.min_range_z", -2.5);
    this->declare_parameter<double>("segmentation_params.max_range_z", 1.5);

    this->get_parameter("segmentation_params.leaf_size", leaf_size_);
    this->get_parameter("segmentation_params.min_range_y", min_range_y_);
    this->get_parameter("segmentation_params.max_range_y", max_range_y_);
    this->get_parameter("segmentation_params.min_range_x", min_range_x_);
    this->get_parameter("segmentation_params.max_range_x", max_range_x_);
    this->get_parameter("segmentation_params.min_range_z", min_range_z_);
    this->get_parameter("segmentation_params.max_range_z", max_range_z_);
    
    global_to_odom_tf_stamped_.transform.rotation.w = 1.0; // Identity transform
    global_to_odom_tf_.setIdentity();

    new_stair_data_ = false;
    new_cloud_data_ = false; 
    new_odom_data_ = false;

    lasercloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    lasercloud_local_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    lasercloud_stacked_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    lasercloud_cropped_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    lasercloud_transformed_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());

    lasercloud_stair_surface_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());

    lasercloud_->reserve(300000);
    long max_points_in_voxel_grid = round(((max_range_x_ - min_range_x_) / leaf_size_) * ((max_range_y_ - min_range_y_) / leaf_size_) * ((max_range_z_ - min_range_z_) / leaf_size_));
    lasercloud_stacked_->reserve(max_points_in_voxel_grid);
    lasercloud_cropped_->reserve(max_points_in_voxel_grid);
    lasercloud_local_->reserve(max_points_in_voxel_grid);

    // --- Initialize ROS 2 Components ---
    segmented_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(segmented_stair_surface_topic_, 10);

    // Subscribers with individual callbacks for caching
    staircse_estimate_sub_ = this->create_subscription<staircase_msgs::msg::StaircaseMsg>(staircase_estimates_topic_, 10, std::bind(&SimpleClutterSegmentationNode::staircase_callback, this, std::placeholders::_1));

    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(robots_topics_prefix_ + "/" + pointcloud_topic_, 10, std::bind(&SimpleClutterSegmentationNode::pointcloud_callback, this, std::placeholders::_1));
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(robots_topics_prefix_ + "/" + odom_topic_, 10, std::bind(&SimpleClutterSegmentationNode::odometry_callback, this, std::placeholders::_1));

    global_tf_sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(robots_topics_prefix_ + "/" + global_tf_topic_, 10, std::bind(&SimpleClutterSegmentationNode::global_tf_handler, this, std::placeholders::_1));

    // Timer to trigger the main processing loop
    timer_ = this->create_wall_timer( std::chrono::duration<double>(1.0 / processing_rate_), std::bind(&SimpleClutterSegmentationNode::segment_pointclouds, this));

    RCLCPP_INFO(this->get_logger(), "SimpleClutterSegmentationNode has started. Sim: %d ", simulation_);
}

void SimpleClutterSegmentationNode::staircase_callback(const staircase_msgs::msg::StaircaseMsg::ConstSharedPtr& msg)
{
    latest_stair_msg_ = msg;
    new_stair_data_ = true;
}

void SimpleClutterSegmentationNode::pointcloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
    if (new_odom_data_)
    {
        lasercloud_->clear();
        pcl::fromROSMsg(*msg, *lasercloud_);

        if(!combine_scans_for_pointcloud_){
            lasercloud_stacked_->clear();
        }

        for(size_t i = 0; i < lasercloud_->points.size(); i++){
            lasercloud_stacked_->points.push_back(lasercloud_->points[i]);
        }
        
        box_filter_.setMin(Eigen::Vector4f(min_x_, min_y_, min_z_, -1.0));
        box_filter_.setMax(Eigen::Vector4f(max_x_, max_y_, max_z_, 1.0));
        box_filter_.setInputCloud(lasercloud_stacked_);
        box_filter_.filter(*lasercloud_cropped_);

        voxel_grid_filter_.setInputCloud(lasercloud_cropped_);
        voxel_grid_filter_.setLeafSize(leaf_size_, leaf_size_, leaf_size_);  
        voxel_grid_filter_.filter(*lasercloud_stacked_);
        
        new_cloud_data_ = true;
    }
}

void SimpleClutterSegmentationNode::odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
{
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(vehicle_roll_, vehicle_pitch_, vehicle_yaw_);

    vehicle_x_ = msg->pose.pose.position.x;
    vehicle_y_ = msg->pose.pose.position.y;
    vehicle_z_ = msg->pose.pose.position.z;

    vehicle_pose_ = msg->pose.pose;

    max_x_ = max_range_x_ + vehicle_x_;
    min_x_ = min_range_x_ + vehicle_x_;
    max_y_ = max_range_y_ + vehicle_y_;
    min_y_ = min_range_y_ + vehicle_y_;
    max_z_ = max_range_z_ + vehicle_z_;
    min_z_ = min_range_z_ + vehicle_z_;

    new_odom_data_ = true;
}

void SimpleClutterSegmentationNode::global_tf_handler(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
{
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 15000, "[Staircase Robot Node] Received and Updated Global TF");
    global_to_odom_tf_stamped_.transform = msg->transform;
    global_to_odom_tf_stamped_.header = msg->header;

    tf2::fromMsg(global_to_odom_tf_stamped_.transform, global_to_odom_tf_);
}

void SimpleClutterSegmentationNode::segment_pointclouds()
{

    if (!new_stair_data_ || !new_cloud_data_ || !new_odom_data_) {
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for new data...");
        return;
    }

    // Transform Point Cloud to Global if staircase is also in global frame
    pcl_ros::transformPointCloud(*lasercloud_stacked_, *lasercloud_transformed_, global_to_odom_tf_);
    
    // Transform Odometry to global if staircase is also in global frame
    geometry_msgs::msg::Pose vehicle_pose_global;
    tf2::doTransform(vehicle_pose_, vehicle_pose_global, global_to_odom_tf_stamped_);

    if (transform_detections_to_global_) {
        robot_pos_.frame_id = global_frame_id_;
    } else {
        robot_pos_.frame_id = odom_frame_id_;
    }
    
    robot_pos_.vehicle_pos = Eigen::Translation3d(vehicle_pose_global.position.x, vehicle_pose_global.position.y, vehicle_pose_global.position.z);
    robot_pos_.vehicle_quat = Eigen::Quaterniond(vehicle_pose_global.orientation.w, vehicle_pose_global.orientation.x, vehicle_pose_global.orientation.y, vehicle_pose_global.orientation.z);

    // Perform the segmentation
    segment_stair_surfaces(latest_stair_msg_, lasercloud_transformed_, lasercloud_stair_surface_);
    if (lasercloud_stair_surface_->points.empty()) {
        return;
    }
    
    // Convert the resulting PCL cloud back to a ROS message
    sensor_msgs::msg::PointCloud2 segmented_cloud_msg;
    pcl::toROSMsg(*lasercloud_stair_surface_, segmented_cloud_msg);
    
    if (transform_detections_to_global_) {
        segmented_cloud_msg.header.frame_id = global_frame_id_;
    } else {
        segmented_cloud_msg.header.frame_id = odom_frame_id_;
    }
    segmented_cloud_msg.header.stamp = this->now();
    segmented_cloud_pub_->publish(segmented_cloud_msg);

    // Reset flags
    new_stair_data_ = false;
}

void SimpleClutterSegmentationNode::segment_stair_surfaces(
    const staircase_msgs::msg::StaircaseMsg::ConstSharedPtr& stair_msg,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& point_cloud_in,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& segmented_stair_surfaces)
{
    segmented_stair_surfaces->points.clear();
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_pc(new pcl::PointCloud<pcl::PointXYZI>);

    double stair_depth = stair_msg->stair_depth;
    double stair_curvature = stair_msg->stair_curvature;
    double stair_width = stair_msg->stair_width;

    double surface_thickness, step_direction, curr_step_dir;

    for (size_t i = 0; i < stair_msg->stair_count; ++i)
    {
        Eigen::Vector2d corner1, start(stair_msg->steps_start_p[i].x, stair_msg->steps_start_p[i].y);
        Eigen::Vector2d corner2, end(stair_msg->steps_end_p[i].x, stair_msg->steps_end_p[i].y);

        std::vector<Eigen::Vector2d> polygon;

        double step_z = (stair_msg->steps_start_p[i].z + stair_msg->steps_end_p[i].z) / 2.0;

        float sigma1 = stair_msg->start_z_covariances[i];
        float sigma2 = stair_msg->end_z_covariances[i];

        surface_thickness = std::min(std::max(2.0 * sqrt(sigma1), 2.0 * sqrt(sigma2)), max_surface_thickness_);
        
        // Compute Step Direction - Computation can be improved in the future.

        if(i < (stair_msg->stair_count - 1)){
            Eigen::Vector2d c_0 = (start + end)/2;
            Eigen::Vector2d st_i1(stair_msg->steps_start_p[i+1].x, stair_msg->steps_start_p[i+1].y);
            Eigen::Vector2d ed_i1(stair_msg->steps_end_p[i+1].x, stair_msg->steps_end_p[i+1].y);

            Eigen::Vector2d c_1 = (st_i1 + ed_i1) / 2;

            double dir = atan2(c_1(1) - c_0(1), c_1(0) - c_0(0));
            curr_step_dir = stair_utility::wrap2PI(atan2(end.y() - start.y(), end.x() - start.x()));
            
            if(fabs(stair_utility::wrap2PI(curr_step_dir + M_PI_2 - dir)) < fabs(stair_utility::wrap2PI(curr_step_dir - M_PI_2 - dir))){
                step_direction =  stair_utility::wrap2PI(curr_step_dir + M_PI_2);
            }
            else{
                step_direction = stair_utility::wrap2PI(curr_step_dir - M_PI_2);
            }
        }
        else if(i == stair_msg->stair_count - 1){
            Eigen::Vector2d c_1 = (start + end)/2;
            Eigen::Vector2d st_i1(stair_msg->steps_start_p[i-1].x, stair_msg->steps_start_p[i-1].y);
            Eigen::Vector2d ed_i1(stair_msg->steps_end_p[i-1].x, stair_msg->steps_end_p[i-1].y);

            Eigen::Vector2d c_0 = (st_i1 + ed_i1) / 2;

            double dir = atan2(c_1(1) - c_0(1), c_1(0) - c_0(0));
            curr_step_dir = stair_utility::wrap2PI(atan2(end.y() - start.y(), end.x() - start.x()));
            
            if(fabs(stair_utility::wrap2PI(curr_step_dir + M_PI_2 - dir)) < fabs(stair_utility::wrap2PI(curr_step_dir - M_PI_2 - dir))){
                step_direction =  stair_utility::wrap2PI(curr_step_dir + M_PI_2);
            }
            else{
                step_direction = stair_utility::wrap2PI(curr_step_dir - M_PI_2);
            }
        }
        
        // Eigen::Vector2d robot_xy(robot_pos_.vehicle_pos.x(), robot_pos_.vehicle_pos.y());
        // float xy_stair_dist = stair_utility::get_line_distance(start, end, robot_xy);
        // float z_stair_dist = (step_z - robot_pos_.vehicle_pos.z());

        // if((z_stair_dist > max_z_step_distance_) || (z_stair_dist < min_z_step_distance_) || (xy_stair_dist > max_xy_step_distance_)){
        //     std::cout << "[Stair Surface Estimation] Robot Z: " << robot_pos_.vehicle_pos.z() << " StepZ: " << step_z << " Diff: " << z_stair_dist << " XY Dist: " <<  xy_stair_dist <<std::endl;  
        //     std::cout << "[Stair Surface Estimation] Step too far away from Sensor View. Skipping" << std::endl;
        //     continue;
        // }

        Eigen::Vector2d offset = {cos(step_direction), sin(step_direction)};
        
        double effective_depth_s = (stair_depth + (0.5 * stair_width * sin(stair_curvature)));
        double effective_depth_e = (stair_depth - (0.5 * stair_width * sin(stair_curvature)));
        
        Eigen::Vector2d corner4 = end + effective_depth_e * offset;
        Eigen::Vector2d corner3 = start + effective_depth_s * offset;
        corner1 = start - (0.025 * offset);
        corner2 = end - (0.025 * offset);

        polygon.push_back(corner1); polygon.push_back(corner2);
        polygon.push_back(corner4); polygon.push_back(corner3);

        temp_pc->clear();

        for (const auto& point : point_cloud_in->points) {
            if ((point.z >= step_z - surface_thickness) && (point.z <= step_z + surface_thickness)) {
                Eigen::Vector3d curr_point_3d(point.x, point.y, point.z);
                if (stair_utility::isPointInPolygon(curr_point_3d, polygon)) {
                    temp_pc->points.push_back(point);
                }
            }
        }
        
        if (temp_pc->points.size() < 10) continue;

        pcl::SampleConsensusModelParallelPlane<pcl::PointXYZI>::Ptr plane_model(new pcl::SampleConsensusModelParallelPlane<pcl::PointXYZI>(temp_pc));
        plane_model->setAxis(Eigen::Vector3f(1.0, 1.0, 0.0));
        plane_model->setEpsAngle(0.15);

        pcl::RandomSampleConsensus<pcl::PointXYZI> ransac(plane_model);
        ransac.setDistanceThreshold(0.03);
        ransac.setMaxIterations(100);
        
        if (ransac.computeModel()) {
            pcl::Indices inliers;
            ransac.getInliers(inliers);
            pcl::copyPointCloud(*temp_pc, inliers, *temp_pc);
            *segmented_stair_surfaces += *temp_pc;
        }

    }

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleClutterSegmentationNode>();
    rclcpp::spin(node); 
    rclcpp::shutdown();
    return 0;
}