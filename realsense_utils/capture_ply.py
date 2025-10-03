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
import math
from datetime import datetime
from orientation_tracker import OrientationTracker


def save_ply(points, colors, filename):
    """
    Save point cloud as PLY file with RGB colors

    Args:
        points: Nx3 array of (x, y, z) coordinates
        colors: Nx3 array of (r, g, b) colors [0-255]
        filename: output PLY filename
    """
    # Remove invalid points (inf/nan coordinates or colors)
    valid_mask = np.isfinite(points).all(axis=1) & np.isfinite(colors).all(axis=1)
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


def estimate_camera_height(points):
    """
    Estimate camera height above ground using point cloud data

    Args:
        points: Nx3 array of 3D points in target coordinate system (X=forward, Y=left, Z=up)

    Returns:
        float: Estimated height in meters (None if estimation fails)
    """
    if len(points) < 100:
        return None

    # Method: Use percentile of Z values (up axis)
    # The ground should be represented by the lowest Z values
    z_values = points[:, 2]
    valid_z = z_values[np.isfinite(z_values)]

    if len(valid_z) < 50:
        return None

    # Use 5th percentile as ground level estimate (robust against noise)
    ground_level = np.percentile(valid_z, 5)

    # Camera height = distance from camera (origin) to ground level
    # Since points are in camera coordinate system, camera is at (0,0,0)
    # Ground level is at Z = ground_level (negative if below camera)
    camera_height = -ground_level  # Negative because ground is below camera

    return max(0, camera_height)  # Ensure positive height


def estimate_camera_height_robust(points):
    """
    More robust height estimation using plane fitting

    Args:
        points: Nx3 array of 3D points in target coordinate system

    Returns:
        tuple: (height, confidence) where confidence is 0-1
    """
    if len(points) < 200:
        return None, 0.0

    try:
        # Sample points for plane fitting (use bottom 30% of Z values)
        z_values = points[:, 2]
        valid_mask = np.isfinite(z_values)
        valid_points = points[valid_mask]

        if len(valid_points) < 100:
            return None, 0.0

        # Sort by Z and take lowest 30% as ground candidates
        sorted_indices = np.argsort(valid_points[:, 2])
        ground_candidates = valid_points[sorted_indices[:len(sorted_indices)//3]]

        # Remove outliers using IQR
        z_ground = ground_candidates[:, 2]
        q25, q75 = np.percentile(z_ground, [25, 75])
        iqr = q75 - q25
        outlier_mask = (z_ground >= q25 - 1.5*iqr) & (z_ground <= q75 + 1.5*iqr)

        clean_ground_points = ground_candidates[outlier_mask]

        if len(clean_ground_points) < 50:
            return None, 0.0

        # Fit plane using least squares: z = ax + by + d
        A = np.c_[clean_ground_points[:, 0], clean_ground_points[:, 1], np.ones(len(clean_ground_points))]
        B = clean_ground_points[:, 2]

        # Solve normal equations
        coeffs = np.linalg.lstsq(A, B, rcond=None)[0]
        a, b, d = coeffs

        # Distance from origin (camera at 0,0,0) to plane z = ax + by + d
        # At camera position (0,0): ground_z = a*0 + b*0 + d = d
        camera_height = -d  # Negative because ground is below camera

        # Calculate confidence based on how well points fit the plane
        predicted_z = clean_ground_points[:, 0] * a + clean_ground_points[:, 1] * b + d
        residuals = clean_ground_points[:, 2] - predicted_z
        rmse = np.sqrt(np.mean(residuals**2))

        # Confidence decreases with RMSE (good fit = high confidence)
        confidence = max(0, min(1, 1 - rmse/0.1))  # 10cm RMSE = 0 confidence

        return max(0, camera_height), confidence

    except:
        return None, 0.0


def setup_camera(visual_preset=4, width=424, height=240):
    """Setup RealSense camera with optimal settings for point cloud capture"""
    pipeline = rs.pipeline()
    config = rs.config()

    # Configure streams - 424x240 optimal for 1 point per 2.5cm at 5m range
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 30)

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


def capture_point_cloud(pipeline, align, depth_intrinsics, orientation_tracker=None, max_distance=3.0):
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

    # Apply gravity alignment if available
    if orientation_tracker and orientation_tracker.is_ready():
        R = orientation_tracker.get_rotation_matrix()
        points = points @ R.T
        print("Applied gravity alignment")

    # Estimate camera height
    height_simple = estimate_camera_height(points)
    height_robust, confidence = estimate_camera_height_robust(points)

    if height_robust is not None:
        print(f"Camera height: {height_robust:.2f}m (confidence: {confidence:.2f})")
    elif height_simple is not None:
        print(f"Camera height: {height_simple:.2f}m (simple estimate)")

    # Get corresponding RGB colors
    colors = color_image[v, u]  # BGR format
    colors = colors[:, [2, 1, 0]]  # Convert BGR to RGB

    return points, colors


def live_preview(pipeline, align, depth_intrinsics, orientation_tracker=None, max_distance=5.0):
    """Live preview with capture capability"""
    print("\nLive Preview Mode")
    print("Controls:")
    print("  SPACE - Capture point cloud")
    print("  's'   - Save current frame as image")
    print("  'q'   - Quit")
    if orientation_tracker:
        print("  Gravity alignment: ENABLED")
    else:
        print("  Gravity alignment: DISABLED")
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

            # Add status overlay
            status_text = f"Captured: {capture_count}"
            if orientation_tracker and orientation_tracker.is_ready():
                pitch, roll, yaw = orientation_tracker.get_orientation()
                status_text += f" | P:{pitch:.1f}° R:{roll:.1f}°"
            elif orientation_tracker:
                status_text += " | IMU: Initializing"

            cv2.putText(images, status_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            cv2.imshow('RealSense Live Preview (Color | Depth)', images)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord(' '):  # Spacebar to capture
                print("Capturing point cloud...")
                points, colors = capture_point_cloud(pipeline, align, depth_intrinsics, orientation_tracker, max_distance)

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


def capture_single(pipeline, align, depth_intrinsics, orientation_tracker=None, filename=None):
    """Capture a single point cloud and save it"""
    if filename is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        os.makedirs("pointclouds", exist_ok=True)
        filename = f"pointclouds/pointcloud_{timestamp}.ply"

    print("Capturing single point cloud...")

    # Wait a moment for camera to stabilize
    for i in range(10):
        pipeline.wait_for_frames()

    points, colors = capture_point_cloud(pipeline, align, depth_intrinsics, orientation_tracker)

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
    parser.add_argument('--resolution', '-r', type=str, default='424x240',
                       help='Camera resolution: 424x240, 640x480, 848x480, 1280x720 (default: 424x240)')
    parser.add_argument('--no-gravity-align', action='store_true',
                       help='Disable gravity alignment correction')

    args = parser.parse_args()

    # Parse resolution
    try:
        width, height = map(int, args.resolution.split('x'))
    except ValueError:
        print(f"Error: Invalid resolution format '{args.resolution}'. Use format like '424x240'")
        return 1

    print("RealSense PLY Point Cloud Capture")
    print("=================================")
    print("Coordinate system: X=forward, Y=left, Z=up")
    print(f"Resolution: {width}x{height}")
    print(f"Max distance: {args.max_distance}m")
    print(f"Visual preset: {args.visual_preset}")
    print(f"Gravity alignment: {'Disabled' if args.no_gravity_align else 'Enabled'}")

    # Calculate and show point spacing
    fov_h = 86  # degrees
    fov_v = 57  # degrees
    spacing_h = (2 * args.max_distance * math.tan(math.radians(fov_h/2))) / width * 100
    spacing_v = (2 * args.max_distance * math.tan(math.radians(fov_v/2))) / height * 100
    print(f"Point spacing at {args.max_distance}m: {spacing_h:.1f}cm x {spacing_v:.1f}cm")
    print()

    # Start orientation tracker
    orientation_tracker = None
    if not args.no_gravity_align:
        print("Starting orientation tracker...")
        orientation_tracker = OrientationTracker()
        if not orientation_tracker.start():
            print("Warning: Failed to start orientation tracker. Continuing without gravity alignment.")
            orientation_tracker = None

    try:
        # Setup camera
        pipeline, align, depth_intrinsics = setup_camera(args.visual_preset, width, height)

        if args.single:
            # Single capture mode
            if orientation_tracker:
                time.sleep(1.0)  # Allow tracker to initialize
            success = capture_single(pipeline, align, depth_intrinsics, orientation_tracker, args.output)
            return 0 if success else 1
        else:
            # Live preview mode
            live_preview(pipeline, align, depth_intrinsics, orientation_tracker, args.max_distance)
            return 0

    except Exception as e:
        print(f"Error: {e}")
        return 1

    finally:
        if orientation_tracker:
            orientation_tracker.stop()
        try:
            pipeline.stop()
        except:
            pass


if __name__ == "__main__":
    import sys
    sys.exit(main())