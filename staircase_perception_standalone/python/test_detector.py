#!/usr/bin/env python3
"""
Python version of test_detector_only.cpp
Tests the StairDetector with PCD files or generates synthetic data
"""

import sys
import time
import numpy as np
import stair_detector_py as sd

def print_staircase_results(stair_measurement, direction):
    """Print detailed staircase detection results"""
    print(f"\n=== {direction} Staircase Detected ===")
    print(f"Step count: {stair_measurement.stair_count}")

    steps = stair_measurement.steps
    if len(steps) > 1:
        total_height = 0.0
        total_depth = 0.0
        total_width = 0.0
        k = len(steps)

        # Calculate averages (matching C++ implementation)
        for i in range(len(steps) - 1):
            step_i = steps[i]
            step_i1 = steps[i + 1]

            # Height: ||p_s^{i+1} - p_s^i||_z + ||p_e^{i+1} - p_e^i||_z
            height_start = abs(step_i1.start_p[2] - step_i.start_p[2])
            height_end = abs(step_i1.end_p[2] - step_i.end_p[2])
            total_height += (height_start + height_end)

            # Tread depth: ||p_s^{i+1} - p_s^i||_xy + ||p_e^{i+1} - p_e^i||_xy
            depth_start = np.sqrt((step_i1.start_p[0] - step_i.start_p[0])**2 +
                                 (step_i1.start_p[1] - step_i.start_p[1])**2)
            depth_end = np.sqrt((step_i1.end_p[0] - step_i.end_p[0])**2 +
                               (step_i1.end_p[1] - step_i.end_p[1])**2)
            total_depth += (depth_start + depth_end)

        # Average height and depth
        avg_height = total_height / (2.0 * k)
        avg_depth = total_depth / (2.0 * k)

        # Average width
        for step in steps:
            total_width += step.step_width
        avg_width = total_width / k

        print(f"Average riser height: {avg_height:.3f}m")
        print(f"Average tread depth: {avg_depth:.3f}m")
        print(f"Average step width: {avg_width:.3f}m")

    # Print individual steps
    for i, step in enumerate(steps):
        print(f"Step {i + 1}:")
        print(f"  Start: ({step.start_p[0]:.3f}, {step.start_p[1]:.3f}, {step.start_p[2]:.3f})")
        print(f"  End:   ({step.end_p[0]:.3f}, {step.end_p[1]:.3f}, {step.end_p[2]:.3f})")
        print(f"  Width: {step.step_width:.3f}m")
        print(f"  Polar: r={step.line_polar_form[0]:.2f}, θ={step.line_polar_form[1]:.2f}")


def main():
    """Main function"""
    print("=== Staircase Detector Python Test ===\n")

    # Parse arguments
    import argparse
    parser = argparse.ArgumentParser(description='Test staircase detector with PCD files')
    parser.add_argument('pcd_file', nargs='?', help='Path to PCD file')
    parser.add_argument('-v', '--verbose', action='store_true', help='Enable verbose output')
    parser.add_argument('-c', '--config', default='../config/standalone_detection_config.yaml',
                       help='Path to config file')

    args = parser.parse_args()

    print(f"Loading config from: {args.config}")
    if args.verbose:
        print("Verbose mode: ENABLED")

    # Load configuration
    try:
        config = sd.ConfigParser(args.config)
        detector_params = config.get_detector_params()
        line_params = config.get_line_extractor_params()
    except Exception as e:
        print(f"Error loading config: {e}")
        return 1

    # Set verbose flag
    detector_params.verbose = args.verbose

    # Match line extractor min_line_length to detector min_stair_width
    line_params.min_line_length = detector_params.min_stair_width
    print(f"Using min_line_length = min_stair_width = {line_params.min_line_length}m")

    # Initialize detector
    print("Initializing StairDetector...")
    detector = sd.StairDetector(detector_params, line_params)

    # Load point cloud
    if args.pcd_file:
        print(f"Loading point cloud from: {args.pcd_file}")
        try:
            pcd_data = sd.load_pcd_file(args.pcd_file)
            xyz = pcd_data['xyz']
            intensity = pcd_data['intensity']
            print(f"Loaded {pcd_data['size']} points")
        except Exception as e:
            print(f"Error loading PCD file: {e}")
            return 1
    else:
        print("No PCD file provided.")
        print(f"Usage: {sys.argv[0]} [PCD_FILE] [-v|--verbose] [-c|--config CONFIG_FILE]")
        return 1

    # Apply voxel filtering using numpy
    print(f"Applying voxel grid filter (leaf size: {detector_params.leaf_size}m)...")
    # Simple downsampling - for production use, you'd want proper voxel grid filtering
    # For now, just pass through to match the C++ test
    filtered_xyz = xyz
    filtered_intensity = intensity
    print(f"After filtering: {len(filtered_xyz)} points")

    # Set point cloud
    detector.set_point_cloud(filtered_xyz, filtered_intensity)

    # Run detection
    print("\nRunning staircase detection...")
    start_time = time.time()
    result, stair_up, stair_down = detector.detect_staircase()
    end_time = time.time()

    duration_ms = (end_time - start_time) * 1000
    print(f"Detection completed in {duration_ms:.0f}ms")

    # Print results
    print("\n=== DETECTION RESULTS ===")

    if result == sd.NoStairsDetected:
        print("No staircases detected.")
    elif result == sd.StairsDetectedUp:
        print("Staircase detected going UP only.")
        print(f"\033[1;32m[Stair Detector] Staircase Detected Going Up with {stair_up.stair_count} steps! \033[0m")
        if len(stair_up.steps) > 0:
            first_step = stair_up.steps[0]
            distance_forward = np.sqrt(first_step.start_p[0]**2 + first_step.start_p[1]**2)
            height_above_ground = first_step.start_p[2] - (-detector_params.robot_height)
            print(f"\033[1;32m[Stair Detector] First step is {distance_forward:.2f}m in front of you and "
                  f"{height_above_ground:.2f}m above the ground\033[0m")
        print_staircase_results(stair_up, "ASCENDING")
    elif result == sd.StairsDetectedDown:
        print("Staircase detected going DOWN only.")
        print(f"\033[1;32m[Stair Detector] Staircase Detected Going Down {stair_down.stair_count} steps! \033[0m")
        if len(stair_down.steps) > 0:
            first_step = stair_down.steps[0]
            distance_forward = np.sqrt(first_step.start_p[0]**2 + first_step.start_p[1]**2)
            height_below_ground = (-detector_params.robot_height) - first_step.start_p[2]
            print(f"\033[1;32m[Stair Detector] First step is {distance_forward:.2f}m in front of you and "
                  f"{height_below_ground:.2f}m below the ground\033[0m")
        print_staircase_results(stair_down, "DESCENDING")
    elif result == sd.StairsDetectedBoth:
        print("Staircases detected in BOTH directions.")
        print(f"\033[1;32m[Stair Detector] Staircase Detected Going Up with {stair_up.stair_count} steps! \033[0m")
        if len(stair_up.steps) > 0:
            first_step = stair_up.steps[0]
            distance_forward = np.sqrt(first_step.start_p[0]**2 + first_step.start_p[1]**2)
            height_above_ground = first_step.start_p[2] - (-detector_params.robot_height)
            print(f"\033[1;32m[Stair Detector] First step is {distance_forward:.2f}m in front of you and "
                  f"{height_above_ground:.2f}m above the ground\033[0m")
        print(f"\033[1;32m[Stair Detector] Staircase Detected Going Down {stair_down.stair_count} steps! \033[0m")
        if len(stair_down.steps) > 0:
            first_step = stair_down.steps[0]
            distance_forward = np.sqrt(first_step.start_p[0]**2 + first_step.start_p[1]**2)
            height_below_ground = (-detector_params.robot_height) - first_step.start_p[2]
            print(f"\033[1;32m[Stair Detector] First step is {distance_forward:.2f}m in front of you and "
                  f"{height_below_ground:.2f}m below the ground\033[0m")
        print_staircase_results(stair_up, "ASCENDING")
        print_staircase_results(stair_down, "DESCENDING")

    print("\n=== TEST COMPLETED ===")
    return 0


if __name__ == "__main__":
    sys.exit(main())
