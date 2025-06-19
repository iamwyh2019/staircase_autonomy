# `staircase_estimation_robot_node`

This is the primary node that runs on each robot and performs the core perception tasks. It processes raw sensor data to find, track, and filter individual staircase estimates.

## Inputs

* **Point Cloud**
    * **Topic**: Subscribes to the topic specified in `config/unified_estimation_config.yaml` under `staircase_perception_topics.point_cloud_topic` (default: `/namespace/velodyne_cloud_registered`).
    * **Type**: `sensor_msgs/msg/PointCloud2`
    * **Description**: The main 3D sensor data used for staircase detection. Point clouds must be registered to the odometry (Inertial) frame. The namespace is set by `robot_namespace` launch argument. **We assume that the inertial frame is aligned with gravity**, as the roll and pitch are not conisdered when transforming to local frame for processing.

* **Odometry**
    * **Topic**: Subscribes to the topic specified in the config file under `staircase_perception_topics.odometry_topic` (default: `/namespace/odom_to_base_link`). 
    * **Type**: `nav_msgs/msg/Odometry`
    * **Description**: Provides the robot's pose, which is used by the internal EKF for motion updates. Must be with respect to inertial frame. The namespace is set by `robot_namespace` launch argument. **We assume that the inertial frame is aligned with gravity**, as the roll and pitch are not conisdered when transforming to local frame for processing.

* **Global to Inertial Transform**
    * **Topic**: Subscribes to the topic specified in the config file under `staircase_perception_topics.global_transform_topic` (default: `/namespace/transform`).
    * **Type**: `geometry_msgs/msg/TransformStamped`
    * **Description**: Transform between robot's inertial frame and the overall world frame. Used when running with multiple robots. Staircase estimates are transformed to the global frame if the parameter `transform_detections_to_global` is set to `true`. Necessary for running smoothly with `staircase_client node`. The namespace is set by `robot_namespace` launch argument.

* **Staircase Detection Enable/Disable**
    * **Topic**: Subscribes to topic specified in `config/unified_estimation_config.yaml` under `staircase_perception_topics.staircase_enable_topic` (default: `staircase_estimation_robot_node_enable`)
    * **Type**: `std_msgs/msg/Bool`
    * **Description**: Can be used to enable/disable detection node during runtime by publishing `false` to disable and `true` to enable.

## Outputs

* **Filtered Staircase Estimate**
    * **Topic**: `/staircase_estimation_robot_node/staircase_estimates` (This is a shared, global topic by default).
    * **Type**: `staircase_msgs/msg/StaircaseMsg`
    * **Description**: The primary output of the node. It publishes a filtered estimate of a single staircase's geometry after processing it with the EKF.

* **Raw Staircase Measurements (Optional)**
    * **Topic**: `staircase_estimation_robot_node/staircase_measurements`. Can be changed in the `config/unified_estimation_config.yaml` under `staircase_perception_topics.staircase_measurements_topic`.
    * **Type**: `staircase_msgs/msg/StaircaseMeasurement`
    * **Description**: Publishes the raw, unfiltered detections before they enter the EKF. The parameter `publish_measurements` needs to be set to `true`. Useful for debugging.

* **Staircase Detection Node Status**
    * **Topic**: Publishes to topic specified in `config/unified_estimation_config.yaml` under `staircase_perception_topics.staircase_status_topic` (`staircase_estimation_robot_node_status`)
    * **Type**: `std_msgs/msg/Bool`
    * **Description**: Publishes a heartbeat every two seconds to signal if the node and processing loop are running correctly.

--- 

## Parameters: 

### Global Parameters

These parameters apply globally to all nodes launched with this configuration file. These params are shared among all nodes launched using `launch/staircase_robot_nodes.launch.py`

-   `use_sim_time`: (boolean) Set to `true` if you are running the ROS ecosystem with a simulated clock (e.g., using Gazebo or playing back a rosbag). Set to `false` for live operation with a real robot.
-   `global_frame_id`: (string) The name of the fixed, global coordinate frame in your TF tree (e.g., `"map"` or `"odom"`). This frame serves as a stable reference for transforming and merging detections. The provided default is `"global"`.

#### ROS Topics (`staircase_perception_topics`)

This section defines the topic names used for communication between the nodes in this package and the wider ROS system. For more details on topic types and assumptions for all nodes, please look at `docs/ros2_nodes/`

-   `point_cloud_topic`: The input topic for the primary 3D point cloud data (e.g., from a LiDAR).
-   `odometry_topic`: The topic providing the robot's odometry, typically as a `nav_msgs/Odometry` message.
-   `global_transform_topic`: The topic for the transform from the odometry frame to the global frame.
-   `staircase_status_topic`: A topic to publish the current status of the `staircase_estimation_robot_node`.
-   `staircase_enable_topic`: A topic to enable or disable the `staircase_estimation_robot_node`'s processing.
-   `staircase_estimates_topic`: The output topic where the filtered and stable staircase estimates are published. Note the leading `/` makes this a global topic, allowing a single client node to subscribe to estimates from multiple robots.
-   `staircase_measurements_topic`: The output topic for raw, unfiltered staircase detections. Useful for debugging.


### Staircase Estimation Robot Node specific parameters

Parameters under each node are classified based on what they are used for. 

### Node Options(`staircase_estimation_robot_node.**`)

-   `ros_rate`: (double) The operating frequency (in Hz) for the detection node's main processing loop.
-   `debug`: (boolean) If `true`, the node will publish additional debug information and visualizations.
-   `publish_measurements`: (boolean) If `true`, the node publishes raw, unfiltered detections on the `staircase_measurements_topic`.
-   `robot_name`: (string) A unique identifier for the robot, used for multi-robot coordination.
-   `odom_frame_id`: (string) The name of the robot's odometry frame.
-   `body_frame_id`: (string) The name of the robot's base/body frame.
-   `transform_detections_to_global`: (boolean) If `true`, all detections will be transformed into the `global_frame_id` before being processed and published.

### Point Cloud Parameters (`staircase_estimation_robot_node.stair_pointcloud.**`)

These parameters control the pre-processing of the input point cloud.

-   `leaf_size`: (double, meters) The voxel grid leaf size for downsampling the point cloud. Increasing this value improves performance but reduces detail.
-   `min/max_range_x/y/z`: (double, meters) These define a 3D bounding box to crop the point cloud. This is crucial for focusing the algorithm's search area, reducing computational load, and eliminating irrelevant data. **Note:** The detection range is limited by this box. If you want to detect stairs farther away, you must increase these values *and* the `stair_line_extractor.max_range` parameter.

### Stair Detector Parameters (`staircase_estimation_robot_node.stair_detector.**`)

These parameters define the geometric constraints for what qualifies as a staircase. **These are the most important parameters to tune for your specific environment.**

-   `min_stair_count`: The minimum number of consecutive steps required to be considered a valid staircase.
-   `stair_slope_min/max`: (radians) The valid range for the overall slope of the staircase.
-   `min_stair_width`: (meters) The minimum width of a valid staircase.
-   `min/max_stair_height`: (meters) The valid range for the height (rise) of a single step.
-   `min/max_stair_depth`: (meters) The valid range for the depth (run) of a single step.
-   `max_stair_curvature`: (radians) The maximum allowable curvature for detecting spiral or curved staircases.
-   `angle_resolution`: (degrees) The resolution used when projecting the point cloud into a front-view for line detection.
-   `robot_height`: (meters) The height of the sensor/robot from the ground. This is used to help segment the ground plane. **IMPORTANT**
-   `initialization_range`: (meters) The vertical range above the detected ground plane to search for the first two steps of a staircase.
-   `ground_height_buffer`: (meters) A buffer zone around the `robot_height` used to classify ground lines more robustly.
-   `use_ramp_detection`: (boolean) If `true`, the algorithm includes logic to identify and ignore lines on ramps, reducing false positives. Setting to `false` might increase detections but also increases the risk of misidentifying ramps as stairs.

### Line Extractor Parameters (`staircase_estimation_robot_node.stair_line_extractor.**`)

This section configures the algorithm used to extract straight lines from the 2D projected point cloud, which correspond to the edges of the steps.

-   **Weighted Line Fitting**:
    -   `bearing_variance`, `range_variance`, `z_variance`: These variances are part of a weighted least squares fitting model, accounting for sensor noise characteristics.
    -   `least_sq_angle_thresh`, `least_sq_radius_thresh`: Thresholds for the least squares fitting process.
-   **Split and Merge Parameters**: This well-known algorithm is used for line segment detection.
    -   `max_line_gap`: (meters) The maximum gap between two line segments before they are considered separate lines.
    -   `min_range`, `max_range`: (meters) The minimum and maximum range from the sensor at which to consider points for line extraction. `max_range` should be consistent with the `stair_pointcloud` crop box for best results.
    -   `min_split_distance`: (meters) When performing "split" step of split and merge, a split between two points results when the two points are at least this far apart.
    -   `min_line_points`: The minimum number of points required to form a valid line segment.
    -   `outlier_distance`: (meters) Points who are at least this distance from all their neighbours are considered outliers.

### Stair Manager (`staircase_estimation_robot_node.stair_manager.**`)

Manages the list of detected staircases, filtering and associating new detections with existing ones.

-   `yaw_threshold`: (radians) The maximum difference in yaw (orientation) between a new detection and an existing staircase estimate to be considered the same staircase.
-   `filter_type`: (string) The filtering strategy to use for smoothing staircase estimates over time.
    -   `"averg"`: Simple averaging of parameters.
    -   `"l_ekf"`: An Extended Kalman Filter (EKF) operating in the local frame. Provides robust and smooth estimates.
    -   `"maxim"`: A Maximizing filter. Present only as a reference for the paper's baseline

### EKF Parameters (`staircase_estimation_robot_node.stair_ekf_params.**`)

These parameters configure the Extended Kalman Filter if `filter_type` is set to `"l_ekf"`.

-   `initial_measurement_sigmas`: The initial uncertainty standard deviations for the robot measurment component of detected staircase. `[simga_r, sigma_theta, sigma_zstart, sigma_zend]`
-   `initial_pose_sigmas`: The initial uncertainty standard deviations for the robot pose component of detected staircase. `[sigma_x, sigma_y, sigma_z, sigma_yaw]`
-   `measurement_sigmas`: The uncertainty standard deviation of subsequent measurements. `[simga_r, sigma_theta, sigma_zstart, sigma_zend]`
-   `pose_sigmas`: The uncertainty standard deviation of the robot's pose at detection location. `[sigma_x, sigma_y, sigma_z, sigma_yaw]`
-   `model_noise_sigmas`: The process noise sigmas (Q), representing the uncertainty in the filter's prediction model. This accounts for small changes in the staircase's perceived geometry over time. The values correspond to the noise in `[height, depth, width, initial_angle, deviation, end_angle]`.

---