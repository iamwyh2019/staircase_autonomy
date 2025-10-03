# Python Bindings for Staircase Detector

This directory contains Python bindings for the staircase perception standalone detector.

## Architecture Note

Since PCL is only available as x86_64 on macOS (no arm64 version), the Python module must be built and run under x86_64 architecture using Rosetta 2.

## Files

- `bindings.cpp` - pybind11 C++ bindings source code
- `CMakeLists.txt` - CMake configuration for building the Python module
- `build.sh` - Build script for compiling the Python module
- `test_detector.py` - Python test script equivalent to test_detector_only.cpp
- `run.sh` - Convenience wrapper for running Python scripts with correct architecture
- `stair_detector_py.cpython-311-darwin.so` - Compiled Python module (x86_64)

## Requirements

- Python 3.11 (x86_64) at `/Library/Frameworks/Python.framework/Versions/3.11/`
- NumPy (x86_64 version)
- PCL 1.15+ (x86_64)
- Eigen3
- CMake 3.10+

## Building

To rebuild the Python module:

```bash
cd python
./build.sh
```

This will:
1. Configure CMake for x86_64 architecture
2. Build the stair_detector library and dependencies
3. Compile the pybind11 module for Python 3.11
4. Output: `stair_detector_py.cpython-311-darwin.so`

## Usage

### Using the convenience wrapper

```bash
cd python
./run.sh test_detector.py ../data/ascending1.pcd
```

### Direct usage

```bash
arch -x86_64 env PYTHONPATH="" /Library/Frameworks/Python.framework/Versions/3.11/bin/python3 test_detector.py <pcd_file> [-v] [-c config.yaml]
```

### From Python code

```python
import stair_detector_py as sd

# Load configuration
config = sd.ConfigParser("../config/standalone_detection_config.yaml")
detector_params = config.get_detector_params()
line_params = config.get_line_extractor_params()

# Create detector
detector = sd.StairDetector(detector_params, line_params)

# Load point cloud from PCD file
pcd_data = sd.load_pcd_file("path/to/pointcloud.pcd")
xyz = pcd_data['xyz']  # numpy array (N, 3)
intensity = pcd_data['intensity']  # numpy array (N,)

# Set point cloud
detector.set_point_cloud(xyz, intensity)

# Run detection
result, stair_up, stair_down = detector.detect_staircase()

# Check results
if result == sd.StairsDetectedUp:
    print(f"Detected {stair_up.stair_count} steps going up")
    for i, step in enumerate(stair_up.steps):
        print(f"Step {i+1}: start={step.start_p}, end={step.end_p}")
```

## API Reference

### Classes

- **StairDetector**: Main detector class
  - `__init__(detector_params, line_params)`: Constructor
  - `set_point_cloud(xyz, intensity)`: Set input point cloud from numpy arrays
  - `detect_staircase()`: Run detection, returns (result, stair_up, stair_down)

- **ConfigParser**: Configuration file parser
  - `__init__(config_file)`: Load config from YAML file
  - `get_detector_params()`: Get detector parameters
  - `get_line_extractor_params()`: Get line extractor parameters

- **StaircaseDetectorParams**: Detection parameters (see config file)
- **LineExtractorParams**: Line extraction parameters (see config file)
- **StaircaseMeasurement**: Detection result containing steps
- **StairStep**: Individual step information

### Enums

- **StaircaseDetectorResult**:
  - `NoStairsDetected`
  - `StairsDetectedUp`
  - `StairsDetectedDown`
  - `StairsDetectedBoth`

### Functions

- `load_pcd_file(filename)`: Load PCD file, returns dict with 'xyz', 'intensity', 'size'

## Troubleshooting

### Architecture mismatch errors

If you see errors about "Bad CPU type" or "incompatible architecture", make sure you're running under x86_64:

```bash
arch -x86_64 /Library/Frameworks/Python.framework/Versions/3.11/bin/python3
```

### NumPy architecture mismatch

If NumPy fails to load, reinstall it for x86_64:

```bash
arch -x86_64 /Library/Frameworks/Python.framework/Versions/3.11/bin/pip3 install --force-reinstall numpy
```

### Module not found

Make sure you're in the `python/` directory or add it to your PYTHONPATH:

```bash
export PYTHONPATH=/path/to/staircase_perception_standalone/python:$PYTHONPATH
```
