#include "staircase_perception/ros2/staircase_client_node.hpp"

StaircaseClientNode::StaircaseClientNode() : Node("staircase_client_node")
{
    // Declare and get parameters with default values
    this->declare_parameter<std::string>("staircase_perception_topics.staircase_estimates_topic", "/staircase_estimation_robot_node/staircase_estimates");
    this->declare_parameter<std::string>("staircase_perception_topics.multirobot_merged_staircase_estimates_topic", "multi_robot_staircase_estimates");
    this->declare_parameter<std::string>("staircase_perception_topics.staircase_client_marker_topic", "staircase_markers");
    this->declare_parameter<std::string>("staircase_perception_topics.staircase_client_lists_topic", "staircase_lists");

    this->declare_parameter<std::string>("staircase_perception_params.global_frame_id", "global");

    this->declare_parameter<bool>("publish_updated_estimate", true);
    this->declare_parameter<std::string>("publish_marker_type", "combined");

    this->declare_parameter<double>("stair_manager.yaw_threshold", 0.5);
    this->declare_parameter<std::string>("stair_manager.filter_type", "averg");

    // Load parameters
    global_frame_id_ = this->get_parameter("staircase_perception_params.global_frame_id").as_string();
    incoming_staircase_topic_= this->get_parameter("staircase_perception_topics.staircase_estimates_topic").as_string();
    merged_staircase_topic_= this->get_parameter("staircase_perception_topics.multirobot_merged_staircase_estimates_topic").as_string();
    staircase_marker_topic_ = this->get_parameter("staircase_perception_topics.staircase_client_marker_topic").as_string();
    staircase_list_topic_ = this->get_parameter("staircase_perception_topics.staircase_client_lists_topic").as_string();

    simulation_ = this->get_parameter("use_sim_time").as_bool();

    publish_updated_estimates_ = this->get_parameter("publish_updated_estimate").as_bool();
    std::string marker_type = this->get_parameter("publish_marker_type").as_string();

    if (marker_type == "combined") {
        publish_individual_stair_markers_ = false;
        RCLCPP_INFO(this->get_logger(), "\033[1;35m Publishing markers of staircase using estimated parameters! \033[0m");
    } else if (marker_type == "seperate") {
        publish_individual_stair_markers_ = true;
        RCLCPP_INFO(this->get_logger(), "\033[1;35m Publishing markers with individual steps!\033[0m");
    } else {
        publish_individual_stair_markers_ = false;
        RCLCPP_WARN(this->get_logger(), "Invalid marker type '%s'. Defaulting to 'combined'.", marker_type.c_str());
    }
    
    stair_manager_params_.yaw_threshold = this->get_parameter("stair_manager.yaw_threshold").as_double();
    std::string filter_type = this->get_parameter("stair_manager.filter_type").as_string();

    // The client node currently only supports simple averaging for multi-robot fusion.
    stair_manager_params_.filter_type = stair_utility::StaircaseFilterType::SimpleAveraging;
    if (filter_type != "averg") {
        RCLCPP_WARN(this->get_logger(), "[Staircase Client Node] Invalid filter type '%s' specified. This node only supports 'averaging'. Defaulting to simple averaging.", filter_type.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "\033[1;35m [Staircase Client Node] Using basic averaging for merging stairs! \033[0m");
    }

    stair_manager_ = MultiRobotStairManager(stair_manager_params_);

    // Setup Subscribers
    incoming_staircase_sub_ = this->create_subscription<staircase_msgs::msg::StaircaseMsg>(
        incoming_staircase_topic_, 10, std::bind(&StaircaseClientNode::handleIncomingStaircases, this, std::placeholders::_1));

    // Setup Publishers
    if (publish_updated_estimates_) {
        merged_staircase_pub_ = this->create_publisher<staircase_msgs::msg::StaircaseMsg>(merged_staircase_topic_, 10);
    }

    staircase_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(staircase_marker_topic_, 10);
    staircase_list_pub_ = this->create_publisher<staircase_msgs::msg::StaircasesList>(staircase_list_topic_, 10);

    RCLCPP_INFO(this->get_logger(), "\033[1;35m Staircase Client Node has started.\033[0m");
}

void StaircaseClientNode::handleIncomingStaircases(const staircase_msgs::msg::StaircaseMsg::SharedPtr msg)
{
    std::string incoming_robot_name = msg->robot_id;
    if (msg->frame_id != global_frame_id_) {
        RCLCPP_WARN(this->get_logger(), "Incoming staircase estimate from '%s' is in frame '%s' but client is in '%s'. Assuming Identity transform.",
            incoming_robot_name.c_str(), msg->frame_id.c_str(), global_frame_id_.c_str());
    }

    stair_utility::StaircaseEstimate incoming_estimate;
    incoming_estimate.stair_id = msg->stair_id;
    incoming_estimate.stair_count = msg->stair_count;
    for (size_t i = 0; i < msg->steps_start_p.size(); ++i) {
        stair_utility::StairStep step;
        step.start_p = Eigen::Vector3d(msg->steps_start_p[i].x, msg->steps_start_p[i].y, msg->steps_start_p[i].z);
        step.end_p = Eigen::Vector3d(msg->steps_end_p[i].x, msg->steps_end_p[i].y, msg->steps_end_p[i].z);
        incoming_estimate.steps.push_back(step);
    }

    // Process the new staircase measurement
    stair_manager_.addNewDetectedStaircase(incoming_estimate, incoming_robot_name, updated_estimate_, updated_summary_);
    
    if (publish_updated_estimates_) {
        publishStaircaseEstimate(updated_estimate_);
    }
    publishStaircaseMarker(updated_estimate_, updated_summary_);
    publishStaircaseList();
}

void StaircaseClientNode::publishStaircaseEstimate(const stair_utility::StaircaseEstimate& stair_estimate)
{
    staircase_msgs::msg::StaircaseMsg stair_msg;

    stair_msg.frame_id = global_frame_id_;
    stair_msg.stair_id = stair_estimate.stair_id;
    stair_msg.stair_count = stair_estimate.stair_count;
    stair_msg.robot_id = "staircase_client"; // Identifier for the merged estimate

    for (const auto& step : stair_estimate.steps) {
        geometry_msgs::msg::Point start, end;
        start.x = step.start_p.x();
        start.y = step.start_p.y();
        start.z = step.start_p.z();
        end.x = step.end_p.x();
        end.y = step.end_p.y();
        end.z = step.end_p.z();
        stair_msg.steps_start_p.push_back(start);
        stair_msg.steps_end_p.push_back(end);
    }
    merged_staircase_pub_->publish(stair_msg);
}

void StaircaseClientNode::publishStaircaseMarker(const stair_utility::StaircaseEstimate& stair_estimate, const stair_utility::SingleStaircaseSummary &summary)
{
    visualization_msgs::msg::MarkerArray stair_marker_array;
    int id = stair_estimate.stair_id;

    if (publish_individual_stair_markers_)
    {
        // This mode publishes each step as a separate CUBE marker.
        // It also handles deleting old markers if the step count changes.
        if(stair_id_prev_marker_map_.find(id) == stair_id_prev_marker_map_.end()){
             stair_id_prev_marker_map_[id] = 0;
        }

        int old_stair_count = stair_id_prev_marker_map_[id];
        int current_stair_count = stair_estimate.stair_count;

        for(int i = 0; i < current_stair_count; ++i){
            visualization_msgs::msg::Marker step_marker;
            step_marker.header.frame_id = global_frame_id_;
            step_marker.header.stamp = this->get_clock()->now();
            step_marker.ns = "staircase_" + std::to_string(id);
            step_marker.id = (id * 150) + i; // Unique ID for each step
            step_marker.action = visualization_msgs::msg::Marker::ADD;
            step_marker.type = visualization_msgs::msg::Marker::CUBE;

            // Visual properties
            step_marker.color.r = 1.0; step_marker.color.g = 1.0; step_marker.color.b = 1.0; step_marker.color.a = 1.0;
            
            // Dimensions and orientation
            double height, depth, width, direction, step_orient;
            Eigen::Vector3d step_i_center = (stair_estimate.steps[i].start_p + stair_estimate.steps[i].end_p) / 2.0;

            if (i == current_stair_count - 1) {
                direction = summary.stair_end_direction;
                height = summary.stair_height;
                depth = summary.stair_depth;
            } else {
                Eigen::Vector3d step_i_1_center = (stair_estimate.steps[i + 1].start_p + stair_estimate.steps[i + 1].end_p) / 2.0;
                direction = atan2(step_i_1_center.y() - step_i_center.y(), step_i_1_center.x() - step_i_center.x());
                height = std::abs(step_i_1_center.z() - step_i_center.z());
                depth = (step_i_1_center.head<2>() - step_i_center.head<2>()).norm();
            }

            step_orient = atan2(stair_estimate.steps[i].end_p.y() - stair_estimate.steps[i].start_p.y(), stair_estimate.steps[i].end_p.x() - stair_estimate.steps[i].start_p.x());
            width = (stair_estimate.steps[i].start_p - stair_estimate.steps[i].end_p).norm();

            tf2::Quaternion quat;
            quat.setRPY(0, 0, step_orient);
            
            step_marker.pose.orientation = tf2::toMsg(quat);
            step_marker.pose.position.x = step_i_center.x() + (depth * cos(direction) / 2.0);
            step_marker.pose.position.y = step_i_center.y() + (depth * sin(direction) / 2.0);
            step_marker.pose.position.z = step_i_center.z() - height/2.0;

            step_marker.scale.x = width;
            step_marker.scale.y = depth;
            step_marker.scale.z = height;
            
            stair_marker_array.markers.push_back(step_marker);
        }

        // Delete markers for steps that no longer exist
        if (old_stair_count > current_stair_count) {
            for (int j = current_stair_count; j < old_stair_count; ++j) {
                visualization_msgs::msg::Marker delete_marker;
                delete_marker.header.frame_id = global_frame_id_;
                delete_marker.ns = "staircase_" + std::to_string(id);
                delete_marker.id = (id * 150) + j;
                delete_marker.action = visualization_msgs::msg::Marker::DELETE;
                stair_marker_array.markers.push_back(delete_marker);
            }
        }
        stair_id_prev_marker_map_[id] = current_stair_count;
    }
    else
    {
        // This mode publishes the entire staircase as a single CUBE_LIST marker.
        visualization_msgs::msg::Marker marker_msg;
        marker_msg.header.frame_id = global_frame_id_;
        marker_msg.header.stamp = this->get_clock()->now();
        marker_msg.ns = "staircase_combined" + std::to_string(id);
        marker_msg.id = id;
        marker_msg.action = visualization_msgs::msg::Marker::ADD;
        marker_msg.type = visualization_msgs::msg::Marker::CUBE_LIST;

        marker_msg.scale.x = summary.stair_depth;
        marker_msg.scale.y = summary.stair_width;
        marker_msg.scale.z = summary.stair_height;

        marker_msg.color.r = 1.0; marker_msg.color.g = 1.0; marker_msg.color.b = 1.0; marker_msg.color.a = 1.0;

        for (uint8_t i = 0; i < stair_estimate.stair_count; i++) {
            geometry_msgs::msg::Point pointN;
            pointN.x = i * summary.stair_depth;
            pointN.y = 0;
            pointN.z = i * summary.stair_height;
            marker_msg.points.push_back(pointN);
        }

        Eigen::Vector3d first_step_center = (stair_estimate.steps.front().start_p + stair_estimate.steps.front().end_p) / 2.0;
        Eigen::Vector3d last_step_center = (stair_estimate.steps.back().start_p + stair_estimate.steps.back().end_p) / 2.0;
        float staircase_yaw = atan2(last_step_center.y() - first_step_center.y(), last_step_center.x() - first_step_center.x());
        
        tf2::Quaternion quat;
        quat.setRPY(0, 0, staircase_yaw);

        marker_msg.pose.orientation = tf2::toMsg(quat);
        marker_msg.pose.position.x = first_step_center.x() + (summary.stair_depth * cos(staircase_yaw) / 2.0);
        marker_msg.pose.position.y = first_step_center.y() + (summary.stair_depth * sin(staircase_yaw) / 2.0);
        marker_msg.pose.position.z = first_step_center.z() - (summary.stair_height / 2.0);
        
        stair_marker_array.markers.push_back(marker_msg);
    }
    staircase_marker_pub_->publish(stair_marker_array);
}

void StaircaseClientNode::publishStaircaseList()
{
    stair_manager_.getStaircaseSummaries(staircase_summaries_);
    
    staircase_msgs::msg::StaircasesList list_msg;
    list_msg.header.stamp = this->get_clock()->now();
    list_msg.header.frame_id = global_frame_id_;

    for (const auto& summary : staircase_summaries_) {
        staircase_msgs::msg::SingleStaircaseDetails details;
        
        details.stair_id = summary.id;
        
        summary.robot_list;
        
        for(std::string robot : summary.robot_list){
            details.contributing_robots.push_back(robot);
        }

        details.stair_width = summary.stair_width;
        details.stair_depth = summary.stair_depth;
        details.stair_height = summary.stair_height;
        details.start_yaw = summary.stair_start_direction;
        details.end_yaw = summary.stair_end_direction;
        
        list_msg.staircases.push_back(details);
    }
    staircase_list_pub_->publish(list_msg);
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;35m Starting Staircase ROS 2 Node for Clients \033[0m");
    auto node = std::make_shared<StaircaseClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
