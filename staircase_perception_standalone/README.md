# Staircase Perception Standalone

This is a standalone version of the staircase perception system, converted from the original ROS2 implementation for incremental testing and development.

## Dependencies

- **PCL (Point Cloud Library)** - For point cloud processing
- **Eigen3** - For linear algebra operations
- **CMake 3.10+** - Build system
- **C++17** compatible compiler

## Installation

### Install Dependencies (Ubuntu/Debian)
```bash
sudo apt-get install libpcl-dev libeigen3-dev cmake
```

### Install Dependencies (macOS with Homebrew)
```bash
brew install pcl eigen cmake
```

## Building

```bash
mkdir build
cd build
cmake ..
make
```

## Usage

### Phase 1: StairDetector Testing

Test the core staircase detection algorithm:

```bash
# Run with synthetic data
./test_detector_only

# Run with your own PCD file
./test_detector_only /path/to/your/pointcloud.pcd
```

The test program will:
1. Load a point cloud (or generate synthetic staircase data)
2. Run the detection algorithm
3. Print detailed results about detected staircases
4. Save processed point clouds for visualization

### Expected Output

```
=== Staircase Detector Standalone Test (Phase 1) ===
Initializing StairDetector...
Generating synthetic staircase point cloud...
Generated 12500 points
After voxel filtering: 8234 points

Running staircase detection...
[Staircase Detector] Staircase Detected Going Up with 5 steps!
Detection completed in 45ms

=== DETECTION RESULTS ===
Staircase detected going UP only.

=== ASCENDING Staircase Detected ===
Step count: 5
Step 1:
  Start: (1.25, -0.5, 0.15)
  End:   (1.25, 0.5, 0.15)
  Width: 1.0m
  Polar: r=1.34, θ=0.38
...
```

## Project Structure

```
staircase_perception_standalone/
├── CMakeLists.txt              # Build configuration
├── README.md                   # This file
├── include/
│   └── staircase_perception/
│       ├── core/
│       │   └── stair_detector.hpp
│       └── utils/
│           ├── stair_utilities.hpp
│           └── line_extraction/
├── src/
│   ├── core/
│   │   └── stair_detector.cpp
│   └── utils/
│       └── line_extraction/
├── tests/
│   └── test_detector_only.cpp  # Phase 1 test
└── data/                       # Test data directory
```

## Testing with Your Data

To test with your own point cloud data:

1. Ensure your PCD file contains `PointXYZI` data
2. Point cloud should be in robot-local coordinates (robot at origin)
3. Z-axis should be vertical (up positive)
4. Adjust parameters in `test_detector_only.cpp` as needed

## Parameters

Key detection parameters (in `createDefaultDetectorParams()`):

- `robot_height`: Height of robot sensor above ground (default: 0.3m)
- `min_stair_count`: Minimum steps to detect staircase (default: 3)
- `stair_slope_min/max`: Valid stair slope range (default: 0.2-1.2 rad)
- `min/max_stair_height`: Step height range (default: 0.1-0.25m)
- `min/max_stair_depth`: Step depth range (default: 0.2-0.4m)

## Development Phases

This standalone conversion is organized in phases:

- **Phase 1** (Current): Core `StairDetector` testing
- **Phase 2**: Bayesian filtering (`StaircaseModel` + `MultiStairManager`)
- **Phase 3**: Complete pipeline integration

## Troubleshooting

### Build Issues
- Ensure PCL and Eigen3 are properly installed
- Check CMake can find the dependencies: `cmake .. -DCMAKE_VERBOSE_MAKEFILE=ON`

### Detection Issues
- Verify point cloud is in correct coordinate frame
- Adjust robot height parameter to match your setup
- Check point cloud bounds parameters match your data range