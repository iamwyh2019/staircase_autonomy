# `staircase_standalone_detection_node`


The `staircase_standalone_detection_node` is a ROS 2 node designed for detecting staircases (both ascending and descending) from an incoming point cloud stream. It operates independently, processing each point cloud to identify potential stair structures. Upon detection, it publishes detailed staircase measurements and visualization markers for RViz.

This node is suitable for scenarios where a robot does not have external odometry or mapping systems. It only supports detections, without estimation or merging pupline for this specific detection task.

## Inputs

* **Point Cloud**
    * **Topic**: Subscribes to the topic specified in `config/standalone_detection_config.yaml` under `staircase_detection_topics.point_cloud_topic` (default: `/namespace/velodyne_cloud_registered`).
    * **Type**: `sensor_msgs/msg/PointCloud2`
    * **Description**: The main 3D sensor data used for staircase detection. Point clouds are in local camera frame. The namespace is set by `robot_namespace` launch argument. **We assume that the camera frame is aligned with gravity, i.e., z-axis is pointing up parallel to gravity**.  If the roll and pitch are not zero, detections might not occur.

## Outputs

* **Detected Staircases**
    * **Topic**: `staircase_standalone_detection_node/detections`. Can be changed in the `config/standalone_detection_config.yaml` under `staircase_detection_topics.staircase_detections_topic`.
    * **Type**: `staircase_msgs/msg/StaircaseMeasurement`
    * **Description**: Publishes the staircase detections. 

* **Detected Staircase Markers**
    * **Topic**: `staircase_standalone_detection_node/detections` Can be changed in `config/standalone_detection_config.yaml` under `staircase_detection_topics.staircase_markers_topic` 
    * **Type**: `visualization_msgs/msg/MarkerArray`
    * **Description**: Publishes a heartbeat every two seconds to signal if the node and processing loop are running correctly.

--- 

## Parameters: 

### Staircase Standalone Detection Node specific parameters

Parameters under each node are classified based on what they are used for. 

### Node Options(`staircase_standalone_detection_node.**`)

-   `ros_rate`: (double) The operating frequency (in Hz) for the detection node's main processing loop.
-   `debug`: (boolean) If `true`, the node will publish additional debug information and visualizations.
-   `camera_frame_id`: (string) The name of the camera frame to publish markers.

### ROS Topics (`staircase_standalone_detection_node.staircase_detection_topics`)

- `point_cloud_topic` :  The input topic for the primary 3D point cloud data (e.g., from a Intel Realsense/ZED Camera).
- `staircase_detections_topic` : A topic to publish the detected staircases in the pointcloud
- `staircase_markers_topic` :A topic to publish the markers for the detected staircases to be visualized

### Point Cloud Parameters (`staircase_standalone_detection_node.stair_pointcloud.**`)

These parameters control the pre-processing of the input point cloud.

-   `leaf_size`: (double, meters) The voxel grid leaf size for downsampling the point cloud. Increasing this value improves performance but reduces detail.
-   `min/max_range_x/y/z`: (double, meters) These define a 3D bounding box to crop the point cloud. This is crucial for focusing the algorithm's search area, reducing computational load, and eliminating irrelevant data. **Note:** The detection range is limited by this box. If you want to detect stairs farther away, you must increase these values *and* the `stair_line_extractor.max_range` parameter.

### Stair Detector Parameters (`staircase_standalone_detection_node.stair_detector.**`)

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

### Line Extractor Parameters (`staircase_standalone_detection_node.stair_line_extractor.**`)

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

---
