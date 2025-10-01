# RealSense Utils

Python utilities for Intel RealSense cameras using pyrealsense2.

## Files

- `rs_motion_quaternion.py` - **Quaternion-based motion tracking (NO GIMBAL LOCK!)**
- `rs_motion_d455.py` - D455-specific motion tracking with optional visualization (Euler angles)
- `test_imu.py` - Simple IMU functionality test
- `requirements.txt` - Python dependencies

## rs_motion_quaternion.py (RECOMMENDED)

**This is the superior implementation** that uses quaternions for rotation representation, completely eliminating gimbal lock issues that occur when pitch approaches ±90° or yaw approaches ±90°.

### Key Advantages:
- **No Gimbal Lock**: Uses quaternion representation throughout
- **More Accurate**: Direct quaternion integration of gyroscope data
- **Stable**: Robust complementary filter using quaternion corrections
- **Mathematically Sound**: Proper 3D rotation handling

### Usage:
```bash
# Console output only
python rs_motion_quaternion.py

# With 3D visualization
python rs_motion_quaternion.py --visualize

# Visualization only (no console spam)
python rs_motion_quaternion.py --visualize --no-console
```

## rs_motion_d455.py (Legacy)

Original Euler angle-based implementation. **Has gimbal lock issues** when rotations approach singularities.

### Features

- **D455-optimized**: Multiple configuration attempts for reliable IMU streaming
- **Real-time processing**: Complementary filter for sensor fusion
- **Optional visualization**: 3D (matplotlib) and 2D (OpenCV) rotation display
- **Thread-safe**: Concurrent rotation estimation
- **Flexible output**: Console and/or visual feedback

### Usage

**Basic console output:**
```bash
python rs_motion_d455.py
```

**With 3D visualization:**
```bash
python rs_motion_d455.py --visualize 3d
```

**With 2D visualization:**
```bash
python rs_motion_d455.py --visualize 2d
```

**Visualization only (no console spam):**
```bash
python rs_motion_d455.py --visualize 3d --no-console
```

### Requirements

- Intel RealSense camera with IMU (e.g., D435i, D455, T265)
- Python 3.6+
- pyrealsense2
- numpy

## test_imu.py

Simple test script to verify IMU functionality without complex processing. Good for troubleshooting connectivity issues.

```bash
python test_imu.py
```

### Visualization Options

1. **3D Visualization (matplotlib)**: Shows rotating coordinate axes in real-time
2. **2D Visualization (OpenCV)**: Shows angle values and circular indicators for each axis

### Output

The script continuously displays rotation angles in degrees (matching C++ rs-motion convention):
- **Pitch** (X-axis rotation) - Red
- **Yaw** (Y-axis rotation) - Green
- **Roll** (Z-axis rotation) - Blue

Press Ctrl+C to stop the program, or ESC/'q' in the 2D visualization window.

## Installation

```bash
pip install -r requirements.txt
```

Note: Make sure you have the Intel RealSense SDK installed and your camera is properly connected.