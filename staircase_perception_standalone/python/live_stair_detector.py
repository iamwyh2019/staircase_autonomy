"""
Live staircase detection with Intel RealSense camera, using an OpenCV-based preview
similar to realsense_utils/capture_ply.py.

This version has no dependency on open3d and uses the correct OrientationTracker API.
"""
import sys
import os
import time
import numpy as np
import pyrealsense2 as rs
import cv2
import argparse
import datetime

# --- Path setup for imports ---
try:
    script_dir = os.path.dirname(os.path.realpath(__file__))
    project_root = os.path.abspath(os.path.join(script_dir, '..', '..'))
    realsense_utils_path = os.path.join(project_root, 'realsense_utils')
    sys.path.insert(0, realsense_utils_path)
    from orientation_tracker import OrientationTracker
except (NameError, ImportError) as e:
    print(f"Could not import OrientationTracker: {e}")
    sys.exit(1)

# Setup DLL search paths for Windows (Python 3.8+)
if sys.platform == 'win32' and sys.version_info >= (3, 8):
    path_entries = os.environ.get('PATH', '').split(os.pathsep)
    vcpkg_bins = [p for p in path_entries if 'vcpkg' in p.lower() and 'bin' in p.lower()]
    if vcpkg_bins:
        for vcpkg_bin in vcpkg_bins:
            if os.path.exists(vcpkg_bin):
                os.add_dll_directory(vcpkg_bin)
    else:
        print("WARNING: vcpkg bin directory not found in PATH.", file=sys.stderr)

import stair_detector as sd

# --- Helper Functions (from capture_ply.py) ---
def save_ply(points, colors, filename):
    # (Implementation is correct and unchanged)
    valid_mask = np.isfinite(points).all(axis=1) & np.isfinite(colors).all(axis=1)
    points = points[valid_mask]
    colors = colors[valid_mask]
    if len(points) == 0: return False
    print(f"Saving {len(points)} points to {filename}")
    with open(filename, 'w') as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n")
        for i in range(len(points)):
            x, y, z = points[i]
            r, g, b = colors[i].astype(int)
            f.write(f"{x:.6f} {y:.6f} {z:.6f} {r} {g} {b}\n")
    return True

def transform_coordinates(points_rs):
    """
    Transforms RealSense coordinates (X=right, Y=down, Z=forward) to the
    target coordinate system (X=forward, Y=left, Z=up).
    """
    points_transformed = np.zeros_like(points_rs)
    points_transformed[:, 0] = points_rs[:, 2]  # New X is old Z
    points_transformed[:, 1] = points_rs[:, 0] # New Y is old X
    points_transformed[:, 2] = -points_rs[:, 1]  # New Z is old -Y
    return points_transformed

def print_staircase_results(stair_measurement, direction):
    # (Full implementation restored)
    print(f"\n=== {direction} Staircase Detected ===")
    print(f"Step count: {stair_measurement.stair_count}")
    steps = stair_measurement.steps
    if not steps: return
    if len(steps) > 1:
        total_height, total_depth, total_width = 0.0, 0.0, 0.0
        k = len(steps)
        for i in range(k - 1):
            step_i, step_i1 = steps[i], steps[i+1]
            total_height += abs(step_i1.start_p[2] - step_i.start_p[2]) + abs(step_i1.end_p[2] - step_i.end_p[2])
            depth_start = np.sqrt((step_i1.start_p[0] - step_i.start_p[0])**2 + (step_i1.start_p[1] - step_i.start_p[1])**2)
            depth_end = np.sqrt((step_i1.end_p[0] - step_i.end_p[0])**2 + (step_i1.end_p[1] - step_i.end_p[1])**2)
            total_depth += depth_start + depth_end
        avg_height = total_height / (2.0 * (k - 1))
        avg_depth = total_depth / (2.0 * (k - 1))
        avg_width = sum(s.step_width for s in steps) / k
        print(f"Average riser height: {avg_height:.3f}m")
        print(f"Average tread depth: {avg_depth:.3f}m")
        print(f"Average step width: {avg_width:.3f}m")
    for i, step in enumerate(steps):
        print(f"Step {i + 1}: Start: ({step.start_p[0]:.3f}, {step.start_p[1]:.3f}, {step.start_p[2]:.3f}), End: ({step.end_p[0]:.3f}, {step.end_p[1]:.3f}, {step.end_p[2]:.3f}), Width: {step.step_width:.3f}m")

# --- Main Application ---
def main():
    parser = argparse.ArgumentParser(description='Live staircase detection with Intel RealSense.')
    parser.add_argument('-c', '--config', default='../config/standalone_detection_config.yaml', help='Path to config file')
    parser.add_argument('-v', '--verbose', action='store_true', help='Enable verbose output for stair detector')
    parser.add_argument('--max-distance', type=float, default=4.0, help='Maximum capture distance in meters')
    args = parser.parse_args()

    # --- Initialize Stair Detector ---
    print("Initializing StairDetector...")
    try:
        config_parser = sd.ConfigParser(args.config)
        detector_params = config_parser.get_detector_params()
        line_params = config_parser.get_line_extractor_params()
        detector_params.verbose = args.verbose
        line_params.min_line_length = detector_params.min_stair_width
        detector = sd.StairDetector(detector_params, line_params)
    except Exception as e:
        print(f"Error loading config or initializing detector: {e}")
        return 1

    # --- Initialize RealSense for Depth/Color ---
    print("Initializing Intel RealSense camera for Depth/Color...")
    pipeline = rs.pipeline()
    config = rs.config()
    width, height = 424, 240
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 30)
    profile = pipeline.start(config)
    align = rs.align(rs.stream.color)
    depth_intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()

    # --- Initialize Orientation Tracker ---
    tracker = OrientationTracker()
    if not tracker.start():
        print("Failed to start OrientationTracker. Exiting.")
        pipeline.stop()
        return 1

    print("\nLive preview is active. Press [Space] to capture and detect. Press [Q] to quit.")

    try:
        while True:
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not depth_frame or not color_frame: continue

            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            images = np.vstack((color_image, depth_colormap))

            cv2.imshow('Live Stair Detector (Color | Depth)', images)
            key = cv2.waitKey(1)

            if key & 0xFF == ord('q'): break
            if key & 0xFF != ord(' '): continue

            print("\n--- Capturing and Detecting Stairs ---")
            
            # Point cloud generation logic from capture_ply.py
            u, v = np.meshgrid(np.arange(width), np.arange(height))
            u, v = u.flatten(), v.flatten()
            depth_values = np.asanyarray(depth_frame.get_data()).flatten()

            valid_mask = (depth_values > 0) & (depth_values < args.max_distance * 1000)
            u, v, depth_values = u[valid_mask], v[valid_mask], depth_values[valid_mask]
            depth_values_m = depth_values / 1000.0

            x = (u - depth_intrinsics.ppx) * depth_values_m / depth_intrinsics.fx
            y = (v - depth_intrinsics.ppy) * depth_values_m / depth_intrinsics.fy
            points_rs = np.column_stack((x, y, depth_values_m))
            colors_bgr = np.asanyarray(color_frame.get_data())[v, u]
            colors_rgb = colors_bgr[:, [2, 1, 0]]

            points_transformed = transform_coordinates(points_rs)
            rotation_matrix = tracker.get_rotation_matrix()
            points_corrected = points_transformed @ rotation_matrix.T

            output_dir = "realsense_captures"
            os.makedirs(output_dir, exist_ok=True)
            filename = f"{output_dir}/capture_{datetime.datetime.now():%Y%m%d_%H%M%S}.ply"
            if not save_ply(points_corrected, colors_rgb, filename):
                print("Failed to save PLY file.")
                continue

            # --- Feed to Stair Detector ---
            xyz = points_corrected
            intensity = np.zeros(xyz.shape[0], dtype=np.float32)
            print(f"Processing {xyz.shape[0]} points...")

            start_time = time.time()
            filtered_data = sd.voxel_grid_filter(xyz, intensity, detector_params.leaf_size)
            detector.set_point_cloud(filtered_data['xyz'], filtered_data['intensity'])
            result, stair_up, stair_down = detector.detect_staircase()
            duration_ms = (time.time() - start_time) * 1000
            print(f"Detection process completed in {duration_ms:.0f}ms")

            if result == sd.StaircaseDetectorResult.NoStairsDetected:
                print("\n\033[1;31mNo staircases detected.\033[0m")
            if result in [sd.StaircaseDetectorResult.StairsDetectedUp, sd.StaircaseDetectorResult.StairsDetectedBoth]:
                print_staircase_results(stair_up, "ASCENDING")
            if result in [sd.StaircaseDetectorResult.StairsDetectedDown, sd.StaircaseDetectorResult.StairsDetectedBoth]:
                print_staircase_results(stair_down, "DESCENDING")
            print("--- Detection Complete ---\n")

    finally:
        print("Stopping...")
        pipeline.stop()
        tracker.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
