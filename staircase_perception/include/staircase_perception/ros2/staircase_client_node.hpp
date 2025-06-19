#ifndef _STAIRCASE_CLIENT_NODE_H_
#define _STAIRCASE_CLIENT_NODE_H_

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "staircase_msgs/msg/staircase_msg.hpp"
#include "staircase_msgs/msg/staircases_list.hpp"
#include "staircase_msgs/msg/single_staircase_details.hpp"

#include "staircase_perception/core/multi_stair_manager.hpp"
#include "staircase_perception/utils/stair_utilities.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <string>
#include <vector>
#include <unordered_map>

/**
 * @class StaircaseClientNode
 * @brief A ROS2 node that subscribes to staircase estimates from multiple robots,
 * merges them into a global estimate, and publishes the merged result and visualizations.
 */
class StaircaseClientNode : public rclcpp::Node
{
    public:
    
        StaircaseClientNode();

    private:
        /**
         * @brief Callback function to handle incoming staircase estimates.
         * @param msg The received StaircaseMsg.
         */
        void handleIncomingStaircases(const staircase_msgs::msg::StaircaseMsg::SharedPtr msg);

        /**
         * @brief Publishes the merged staircase estimate.
         * @param stair_estimate The staircase estimate to publish.
         */
        void publishStaircaseEstimate(const stair_utility::StaircaseEstimate& stair_estimate);

        /**
         * @brief Publishes visualization markers for the staircase.
         * @param stair_estimate The staircase estimate for visualization.
         * @param summary The summary containing dimensional properties.
         */
        void publishStaircaseMarker(const stair_utility::StaircaseEstimate& stair_estimate, const stair_utility::SingleStaircaseSummary &summary);
        
        /**
         * @brief Publishes a list of all known staircases for UI consumption.
         */
        void publishStaircaseList();

        // Core logic for managing and fusing staircase data from multiple robots.
        MultiRobotStairManager stair_manager_;

        // Parameters for the stair manager.
        stair_utility::StairManagerParams stair_manager_params_;

        // ROS 2 Publishers and Subscribers
        rclcpp::Subscription<staircase_msgs::msg::StaircaseMsg>::SharedPtr incoming_staircase_sub_;
        rclcpp::Publisher<staircase_msgs::msg::StaircaseMsg>::SharedPtr merged_staircase_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr staircase_marker_pub_;
        rclcpp::Publisher<staircase_msgs::msg::StaircasesList>::SharedPtr staircase_list_pub_;

        // Node Parameters
        std::string global_frame_id_;
        bool publish_updated_estimates_;
        bool publish_individual_stair_markers_;
        bool simulation_;

        std::string incoming_staircase_topic_, merged_staircase_topic_, staircase_marker_topic_, staircase_list_topic_;

        // State variables
        stair_utility::StaircaseEstimate updated_estimate_;
        stair_utility::SingleStaircaseSummary updated_summary_;
        std::vector<stair_utility::SingleStaircaseSummary> staircase_summaries_;

        // Map to keep track of markers for deletion.
        std::unordered_map<int, int> stair_id_prev_marker_map_;
};

#endif // _STAIRCASE_CLIENT_NODE_H_
