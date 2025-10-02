#!/usr/bin/env python3
"""
RealSense D455 PLY Point Cloud Capture
Captures RGB-D data and saves as PLY files with custom coordinate system:
X = forward, Y = left, Z = up
"""

import pyrealsense2 as rs
import numpy as np
import cv2
import time
import os
import argparse
from datetime import datetime


def save_ply(points, colors, filename):
    """
    Save point cloud as PLY file with RGB colors

    Args:
        points: Nx3 array of (x, y, z) coordinates
        colors: Nx3 array of (r, g, b) colors [0-255]
        filename: output PLY filename
    """
    # Remove invalid points (where z=0 or any coordinate is inf/nan)
    valid_mask = (points[:, 2] != 0) & np.isfinite(points).all(axis=1) & np.isfinite(colors).all(axis=1)
    points = points[valid_mask]
    colors = colors[valid_mask]

    if len(points) == 0:
        print(f"Warning: No valid points to save in {filename}")
        return False

    print(f"Saving {len(points)} points to {filename}")

    # Write PLY header
    with open(filename, 'w') as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("property uchar red\n")
        f.write("property uchar green\n")
        f.write("property uchar blue\n")
        f.write("end_header\n")

        # Write vertex data
        for i in range(len(points)):
            x, y, z = points[i]
            r, g, b = colors[i].astype(int)
            f.write(f"{x:.6f} {y:.6f} {z:.6f} {r} {g} {b}\n")

    return True


def transform_coordinates(points):
    """
    Transform from RealSense coordinate system to target coordinate system

    RealSense: X=right, Y=down, Z=forward
    Target:    X=forward, Y=left, Z=up

    Transformation matrix:
    [X_new]   [0  0  1] [X_rs]   [Z_rs]     (forward = RealSense Z)
    [Y_new] = [1  0  0] [Y_rs] = [X_rs]     (left = RealSense X)
    [Z_new]   [0 -1  0] [Z_rs]   [-Y_rs]    (up = -RealSense Y)
    """
    transformed = np.zeros_like(points)
    transformed[:, 0] = points[:, 2]   # X_new = Z_rs (forward)
    transformed[:, 1] = points[:, 0]   # Y_new = X_rs (left)
    transformed[:, 2] = -points[:, 1]  # Z_new = -Y_rs (up)

    return transformed


def setup_camera(visual_preset=4):
    """Setup RealSense camera with optimal settings for point cloud capture"""
    pipeline = rs.pipeline()
    config = rs.config()

    # Configure streams - 640x480 optimal for 1 point per 2.5cm at 5m range
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    print("Starting RealSense pipeline...")
    profile = pipeline.start(config)

    # Get device and enable depth sensor improvements
    device = profile.get_device()
    depth_sensor = device.first_depth_sensor()

    # Apply settings to improve depth quality
    if depth_sensor.supports(rs.option.visual_preset):
        depth_sensor.set_option(rs.option.visual_preset, visual_preset)

    if depth_sensor.supports(rs.option.laser_power):
        depth_sensor.set_option(rs.option.laser_power, 360)  # Max laser power

    # Create alignment object
    align_to = rs.stream.color
    align = rs.align(align_to)

    # Get camera intrinsics for point cloud generation
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()

    depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics

    print(f"Camera resolution: {depth_intrinsics.width}x{depth_intrinsics.height}")
    print(f"Camera intrinsics: fx={depth_intrinsics.fx:.1f}, fy={depth_intrinsics.fy:.1f}")

    return pipeline, align, depth_intrinsics


def capture_point_cloud(pipeline, align, depth_intrinsics, max_distance=3.0):
    """
    Capture a single point cloud frame

    Args:
        pipeline: RealSense pipeline
        align: Alignment object
        depth_intrinsics: Camera intrinsics
        max_distance: Maximum depth in meters

    Returns:
        points: Nx3 array of 3D points
        colors: Nx3 array of RGB colors
    """
    # Capture frames
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)

    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()

    if not depth_frame or not color_frame:
        return None, None

    # Convert to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    # Get image dimensions
    height, width = depth_image.shape

    # Create coordinate arrays
    u, v = np.meshgrid(np.arange(width), np.arange(height))
    u = u.flatten()
    v = v.flatten()
    depth_values = depth_image.flatten()

    # Remove invalid depth values
    valid_mask = (depth_values > 0) & (depth_values < max_distance * 1000)  # Convert to mm
    u = u[valid_mask]
    v = v[valid_mask]
    depth_values = depth_values[valid_mask]

    # Convert depth values to meters
    depth_values_m = depth_values / 1000.0

    # Convert pixel coordinates to 3D points using camera intrinsics
    x = (u - depth_intrinsics.ppx) * depth_values_m / depth_intrinsics.fx
    y = (v - depth_intrinsics.ppy) * depth_values_m / depth_intrinsics.fy
    z = depth_values_m

    # Combine into point array (RealSense coordinate system)
    points_rs = np.column_stack((x, y, z))

    # Transform to target coordinate system (X=forward, Y=left, Z=up)
    points = transform_coordinates(points_rs)

    # Get corresponding RGB colors
    colors = color_image[v, u]  # BGR format
    colors = colors[:, [2, 1, 0]]  # Convert BGR to RGB

    return points, colors


def live_preview(pipeline, align, depth_intrinsics, max_distance=5.0):
    """Live preview with capture capability"""
    print("\nLive Preview Mode")
    print("Controls:")
    print("  SPACE - Capture point cloud")
    print("  's'   - Save current frame as image")
    print("  'q'   - Quit")
    print("-" * 40)

    capture_count = 0

    try:
        while True:
            # Get frames
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)

            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            # Convert to numpy arrays for display
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Create depth colormap
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03),
                cv2.COLORMAP_JET
            )

            # Remove background beyond max distance
            max_depth_mm = max_distance * 1000
            mask = (depth_image > max_depth_mm) | (depth_image <= 0)
            depth_colormap[mask] = [64, 64, 64]

            # Show images vertically stacked
            images = np.vstack((color_image, depth_colormap))

            # Add capture count overlay
            cv2.putText(images, f"Captured: {capture_count}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            cv2.imshow('RealSense Live Preview (Color | Depth)', images)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord(' '):  # Spacebar to capture
                print("Capturing point cloud...")
                points, colors = capture_point_cloud(pipeline, align, depth_intrinsics, max_distance)

                if points is not None:
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    os.makedirs("pointclouds", exist_ok=True)
                    filename = f"pointclouds/pointcloud_{timestamp}.ply"

                    if save_ply(points, colors, filename):
                        capture_count += 1
                        print(f"✓ Saved {filename}")
                    else:
                        print("✗ Failed to save point cloud")
                else:
                    print("✗ Failed to capture point cloud")

            elif key == ord('s'):  # Save current frame
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                cv2.imwrite(f"frame_{timestamp}.png", images)
                print(f"✓ Saved frame_{timestamp}.png")

    except KeyboardInterrupt:
        print("\nCapture interrupted by user")

    finally:
        cv2.destroyAllWindows()


def capture_single(pipeline, align, depth_intrinsics, filename=None):
    """Capture a single point cloud and save it"""
    if filename is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        os.makedirs("pointclouds", exist_ok=True)
        filename = f"pointclouds/pointcloud_{timestamp}.ply"

    print("Capturing single point cloud...")

    # Wait a moment for camera to stabilize
    for i in range(10):
        pipeline.wait_for_frames()

    points, colors = capture_point_cloud(pipeline, align, depth_intrinsics)

    if points is not None:
        if save_ply(points, colors, filename):
            print(f"✓ Successfully saved {filename}")
            print(f"  Points: {len(points)}")
            print(f"  Coordinate system: X=forward, Y=left, Z=up")
            return True
        else:
            print("✗ Failed to save point cloud")
            return False
    else:
        print("✗ Failed to capture point cloud")
        return False


def main():
    parser = argparse.ArgumentParser(description='RealSense PLY Point Cloud Capture')
    parser.add_argument('--output', '-o', type=str,
                       help='Output PLY filename (default: timestamped)')
    parser.add_argument('--single', '-s', action='store_true',
                       help='Capture single point cloud and exit')
    parser.add_argument('--max-distance', '-d', type=float, default=4.0,
                       help='Maximum capture distance in meters (default: 4.0)')
    parser.add_argument('--visual-preset', '-p', type=int, default=4,
                       help='Visual preset: 0=Custom, 1=Default, 2=Hand, 3=High Accuracy, 4=High Density, 5=Medium Density (default: 4)')

    args = parser.parse_args()

    print("RealSense PLY Point Cloud Capture")
    print("=================================")
    print("Coordinate system: X=forward, Y=left, Z=up")
    print(f"Max distance: {args.max_distance}m")
    print(f"Visual preset: {args.visual_preset}")
    print()

    try:
        # Setup camera
        pipeline, align, depth_intrinsics = setup_camera(args.visual_preset)

        if args.single:
            # Single capture mode
            success = capture_single(pipeline, align, depth_intrinsics, args.output)
            return 0 if success else 1
        else:
            # Live preview mode
            live_preview(pipeline, align, depth_intrinsics, args.max_distance)
            return 0

    except Exception as e:
        print(f"Error: {e}")
        return 1

    finally:
        try:
            pipeline.stop()
        except:
            pass


if __name__ == "__main__":
    import sys
    sys.exit(main())