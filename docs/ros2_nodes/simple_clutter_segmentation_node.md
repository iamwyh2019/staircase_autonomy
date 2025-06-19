# `simple_clutter_segmentation_node`

A node that can subscribe to point cloud, odometry and staircase estimates and publish out the segmented clutter-free staircase point cloud. Allows for a different point cloud source such as a stereo camera if needed.

## Inputs

* **Point Cloud**
    * **Topic**: Subscribes to the topic specified in `config/unified_estimation_config.yaml` under `staircase_perception_topics.clutter_point_cloud_in_topic` (default: `/namespace/velodyne_cloud_registered`).
    * **Type**: `sensor_msgs/msg/PointCloud2`
    * **Description**: The main 3D sensor data used for staircase detection. Point clouds must be registered to the odometry (Inertial) frame. The namespace is set by `robot_namespace` launch argument. Can either be set to same topic as detection point cloud input or different dense source such as stereo if available. We assume that the inertial frame is aligned with gravity, as the roll and pitch are not conisdered when transforming to local frame for processing.

* **Odometry**
    * **Topic**: Subscribes to the topic specified in the config file under `staircase_perception_topics.odometry_topic` (default: `/namespace/odom_to_base_link`). 
    * **Type**: `nav_msgs/msg/Odometry`
    * **Description**: Provides the robot's pose. The namespace is set by `robot_namespace` launch argument. We assume that the inertial frame is aligned with gravity, as the roll and pitch are not conisdered when transforming to local frame for processing.

* **Global to Inertial Transform**
    * **Topic**: Subscribes to the topic specified in the config file under `staircase_perception_topics.global_transform_topic` (default: `/namespace/transform`).
    * **Type**: `geometry_msgs/msg/TransformStamped`
    * **Description**: Transform between robot's inertial frame and the overall world frame. Used when running with multiple robots. Staircase estimates are transformed to the global frame if the parameter `transform_detections_to_global` is set to `true`. Should reflect same setting as robot node to ensure consistency.

* **Filtered Staircase Estimate**
    * **Topic**: `/staircase_estimation_robot_node/staircase_estimates`
    * **Type**: `staircase_msgs/msg/StaircaseMsg`
    * **Description**: Receives the staircase estimate from the staircase robot node. This estimate is combined with point cloud to segment out clutter.

## Outputs
* **Segmented Staircase PointCloud**
    * **Topic**: `/simple_clutter_segmentation_node/segmented_stair_surface`
    * **Type**: `sensor_msgs/msg/PointCloud2`
    * **Description**: Published pointcloud of the staircase after segmenting out clutter based on staircase location, and assuming flat steps. 

---

## Parameters: 

### Global Parameters

These parameters from `config/unified_estimation_config.yaml` apply globally to all nodes launched with this configuration file. These params are shared among all nodes launched using `launch/staircase_robot_nodes.launch.py`

-   `use_sim_time`: (boolean) Set to `true` if you are running the ROS ecosystem with a simulated clock (e.g., using Gazebo or playing back a rosbag). Set to `false` for live operation with a real robot.
-   `global_frame_id`: (string) The name of the fixed, global coordinate frame in your TF tree (e.g., `"map"` or `"odom"`). This frame serves as a stable reference for transforming and merging detections. The provided default is `"global"`.

#### ROS Topics (`staircase_perception_topics`)

This section defines the topic names used for communication between the nodes in this package and the wider ROS system. For more details on topic types and assumptions for all nodes, please look at `docs/ros2_nodes/`

-   `odometry_topic`: The topic providing the robot's odometry, typically as a `nav_msgs/Odometry` message.
-   `global_transform_topic`: The topic for the transform from the odometry frame to the global frame.
-   `staircase_estimates_topic`: The output topic where the filtered and stable staircase estimates are published. Note the leading `/` makes this a global topic, allowing a single client node to subscribe to estimates from multiple robots.
-   `clutter_point_cloud_in_topic`: The input topic for the point cloud to be used by the `simple_clutter_segmentation_node`. This is kept separate to allow the use of a denser sensor (like a depth camera) for clutter segmentation.
-   `clutter_point_cloud_out_topic`: The output topic from the `simple_clutter_segmentation_node`, which contains the segmented point cloud of the stair surfaces, free of clutter.


### Clutter Segmentation Node specific params (`simple_clutter_segmentation_node.**`)

This node is designed to segment and remove clutter (e.g., objects, debris) from the surfaces of a detected staircase.

-   `processing_rate`: (double) The operating frequency (in Hz) of the node.
-   `robot_name`, `odom_frame_id`, `body_frame_id`, `transform_detections_to_global`: These function identically to their counterparts in the `staircase_estimation_robot_node`.
-   **`segmentation_params`**:
    -   `max_surface_thickness`: (meters) The maximum thickness used when modeling the planar surfaces of the stairs for segmentation.
    -   `combine_pointcloud_scans`: (boolean) If `true`, the node will accumulate multiple point cloud scans to build a denser map before performing segmentation.
    -   `leaf_size`, `min/max_range_x/y/z`: These function identically to their counterparts in the `staircase_estimation_robot_node`, but are applied to the `clutter_point_cloud_in_topic`.

---