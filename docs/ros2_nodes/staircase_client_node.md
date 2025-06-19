# `staircase_client_node`

This node typically runs off-robot and acts as a central aggregator and monitoring station. It fuses data from all robots to create a global world model of all staircases. You can launch this node as,

## Inputs

* **Staircase Estimates from Robots**
    * **Topic**: Subscribes to the shared `/staircase_estimation_robot_node/staircase_estimates` topic.
    * **Type**: `staircase_msgs/msg/StaircaseMsg`
    * **Description**: Receives the processed staircase estimates from all `staircase_estimation_robot_node` instances.

## Outputs

* **Staircase List for UI**
    * **Topic**: `/staircase_lists` (configurable).
    * **Type**: `staircase_msgs/msg/StaircasesList`
    * **Description**: The primary output for high-level applications or user interfaces. It publishes a structured list of all known staircases and their fused properties.

* **Global Visualization Markers**
    * **Topic**: `/staircase_markers` (configurable).
    * **Type**: `visualization_msgs/msg/MarkerArray`
    * **Description**: Publishes a unified visualization of all staircases detected by all robots in a common frame (default: `global`). This is the main marker topic to view in RViz.

* **Merged Staircase Model (Optional)**
    * **Topic**: `/multi_robot_staircase_estimates` (configurable).
    * **Type**: `staircase_msgs/msg/StaircaseMsg`
    * **Description**: Can publish a geometric representation of the fused (averaged) staircase models. 

---

## Parameters:

### Global Parameters

These parameters from `config/unified_estimation_config.yaml` apply globally to all nodes launched with this configuration file. These params are shared among all nodes launched using `launch/staircase_robot_nodes.launch.py`

-   `use_sim_time`: (boolean) Set to `true` if you are running the ROS ecosystem with a simulated clock (e.g., using Gazebo or playing back a rosbag). Set to `false` for live operation with a real robot.
-   `global_frame_id`: (string) The name of the fixed, global coordinate frame in your TF tree (e.g., `"map"` or `"odom"`). This frame serves as a stable reference for transforming and merging detections. The provided default is `"global"`.

#### ROS Topics (`staircase_perception_topics`)

This section defines the topic names used for communication between the nodes in this package and the wider ROS system. For more details on topic types and assumptions for all nodes, please look at `docs/ros2_nodes/`

-   `staircase_estimates_topic`: The output topic where the filtered and stable staircase estimates are published. Note the leading `/` makes this a global topic, allowing a single client node to subscribe to estimates from multiple robots.
-   `multirobot_merged_staircase_estimates_topic`: The output topic from the `staircase_client_node` containing the merged estimates from all participating robots.
-   `staircase_client_marker_topic`: The topic for publishing visualization markers (`visualization_msgs/MarkerArray`) for Rviz.
-   `staircase_client_lists_topic`: The topic where the `staircase_client_node` publishes the consolidated lists of all detected staircases.

### Staircase Client Node specific parameters (`staircase_client_node.**`)

-   `publish_updated_estimate`: (boolean) If `true`, the client node will publish the merged estimates on the `multirobot_merged_staircase_estimates_topic`.
-   `publish_marker_type`: (string) Controls how visualization markers are published.
    -   `"combined"`: Publishes a single `MarkerArray` for all staircases.
    -   `"seperate"`: Publishes markers on separate topics.
-   **`stair_manager`**:
    -   `yaw_threshold`: (radians) Functions similarly to the one in `staircase_estimation_robot_node`, used to associate incoming estimates from different robots to the same global staircase object.
    -   `filter_type`: (string) The filtering strategy for merging estimates. Currently, only simple averaging (`"averg"`) is supported in the client node.
