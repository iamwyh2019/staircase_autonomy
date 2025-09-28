# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS 2 package for staircase perception and modeling, implementing algorithms from two research papers for autonomous robot navigation on stairs. The system performs 3D point cloud-based staircase detection, multi-robot merging, and Bayesian estimation.

## Architecture

The codebase consists of two main ROS 2 packages:

### staircase_msgs
Contains custom ROS 2 message definitions:
- `StaircaseMsg.msg` - World frame staircase estimates
- `StaircaseMeasurement.msg` - Individual detections with robot odometry
- `SingleStaircaseDetails.msg` and `StaircasesList.msg` - Simplified staircase data for visualization

### staircase_perception
Core perception package with modular architecture:
- **Core Components** (`src/core/`): C++ classes for perception pipeline
  - `StairDetector` - Point cloud processing and staircase detection
  - `StaircaseModel` - Bayesian modeling and Extended Kalman Filter
  - `MultiStairManager` - Single robot staircase tracking
  - `MultiRobotManager` - Multi-robot data fusion
- **ROS 2 Nodes** (`src/ros2/`): ROS wrappers exposing core functionality
- **Utilities** (`src/utils/`): Line extraction algorithms (modified from laser_line_extraction)

## Build Commands

Build the packages:
```bash
colcon build --symlink-install --packages-up-to staircase_perception
source install/setup.bash
```

## Launch Commands

### Single Robot Estimation
Main deployment for robots with SLAM capability:
```bash
ros2 launch staircase_perception staircase_estimation_robot_nodes.launch.py robot_namespace:=/robot_A launch_marker_publisher:=true
```

Key arguments:
- `robot_namespace`: Sensor input topic namespace (default: "")
- `launch_marker_publisher`: Launch visualization node (default: false)
- `config_file`: Parameter file path (default: config/unified_estimation_config.yaml)
- `simulation`: Use simulation time (default: false)

### Multi-Robot Setup
Central aggregator for multi-robot deployments:
```bash
ros2 launch staircase_perception staircase_client_nodes.launch.py
```

### Standalone Detection
For robots without SLAM using raw stereo camera data:
```bash
ros2 launch staircase_perception staircase_standalone_detection_node.launch.py
```

Note: Requires camera aligned with gravity (z-axis up, 0 roll/pitch).

## Data Flow

The system expects:
- **Input**: Registered point clouds from SLAM + robot odometry (main nodes)
- **Input**: Raw point clouds from stereo cameras (standalone node)
- **Output**: Staircase estimates in world frame with uncertainty
- **Visualization**: RViz markers for detected/estimated staircases

## Key Dependencies

- ROS 2 Humble (tested platform)
- PCL (Point Cloud Library)
- Eigen3 (linear algebra)
- tf2 (coordinate transformations)

## Configuration

Main config files:
- `config/unified_estimation_config.yaml` - Full estimation pipeline parameters
- `config/standalone_detection_config.yaml` - Detection-only parameters

## Testing

No specific test commands found. Check for test files or ask maintainer for testing procedures.