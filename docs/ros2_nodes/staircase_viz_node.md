# `staircase_viz_node`, Optional

A node that publishes markers for measurements and estimates on the robot for debugging. Launched if `launch_marker_publisher` arg is set to `true` in the `staircase_robot_nodes.launch.py` launch file. 

#### Inputs
* **Filtered Staircase Estimate**
    * **Topic**: `/staircase_estimation_robot_node/staircase_estimates`
    * **Type**: `staircase_msgs/msg/StaircaseMsg`
    * **Description**: Receives the staircase estimate from the staircase robot node. This estimate is combined with point cloud to segment out clutter.

* **Raw Staircase Measurements (Optional)**
    * **Topic**: `staircase_estimation_robot_node/staircase_measurements`. Can be changed in the `config/unified_estimation_config.yaml` under `staircase_perception_topics.staircase_measurements_topic`.
    * **Type**: `staircase_msgs/msg/StaircaseMeasurement`
    * **Description**: Receives the raw, unfiltered detections from the robot node. The parameter `publish_measurements` needs to be set to `true`.
    
#### Outputs
* **Debug Visualization Markers**
    * **Topic**: `/staircase_viz_node/stair_estimate_markers` and `/staircase_viz_node/stair_measurements_markers`.
    * **Type**: `visualization_msgs/msg/MarkerArray`
    * **Description**: Publishes markers based on publishers from the robot node, which can be visualized in RViz. This comes from the `staircase_viz_node` which is launched if the launch arg `launch_marker_publisher` is set to `true`.
