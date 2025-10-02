#!/usr/bin/env python3
"""
RealSense D455 - EXACT C++ Algorithm Replication
Direct line-by-line copy of rs-motion.cpp algorithm
"""

import pyrealsense2 as rs
import numpy as np
import threading
import time
import math
import sys
import argparse

# Optional visualization imports
try:
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False


class Vector3:
    """Exact copy of float3 from C++"""
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

    def add(self, dx, dy, dz):
        """Exact copy of C++ add function"""
        self.x += dx
        self.y += dy
        self.z += dz


class RotationEstimatorCpp:
    """
    EXACT copy of C++ rotation_estimator class
    """

    def __init__(self):
        # Exact C++ variables
        self.theta = Vector3()
        self.theta_mtx = threading.Lock()
        self.alpha = 0.98  # Exact same alpha as C++
        self.firstGyro = True
        self.firstAccel = True
        self.last_ts_gyro = 0

    def process_gyro(self, gyro_data, ts):
        """EXACT copy of C++ process_gyro function"""
        if self.firstGyro:  # On the first iteration, use only data from accelerometer to set the camera's initial position
            self.firstGyro = False
            self.last_ts_gyro = ts
            return

        # Holds the change in angle, as calculated from gyro
        gyro_angle = Vector3()

        # Initialize gyro_angle with data from gyro
        gyro_angle.x = gyro_data.x  # Pitch
        gyro_angle.y = gyro_data.y  # Yaw
        gyro_angle.z = gyro_data.z  # Roll

        # Compute the difference between arrival times of previous and current gyro frames
        dt_gyro = (ts - self.last_ts_gyro) / 1000.0
        self.last_ts_gyro = ts

        # Change in angle equals gyro measures * time passed since last measurement
        gyro_angle.x *= dt_gyro
        gyro_angle.y *= dt_gyro
        gyro_angle.z *= dt_gyro

        # Apply the calculated change of angle to the current angle (theta)
        with self.theta_mtx:
            self.theta.add(-gyro_angle.z, -gyro_angle.y, gyro_angle.x)

    def process_accel(self, accel_data):
        """EXACT copy of C++ process_accel function"""
        # Holds the angle as calculated from accelerometer data
        accel_angle = Vector3()

        # Calculate rotation angle from accelerometer data
        accel_angle.z = math.atan2(accel_data.y, accel_data.z)
        accel_angle.x = math.atan2(accel_data.x, math.sqrt(accel_data.y * accel_data.y + accel_data.z * accel_data.z))

        # If it is the first iteration, set initial pose of camera according to accelerometer data (note the different handling for Y axis)
        with self.theta_mtx:
            if self.firstAccel:
                self.firstAccel = False
                self.theta = accel_angle
                # Since we can't infer the angle around Y axis using accelerometer data, we'll use PI as a convetion for the initial pose
                self.theta.y = math.pi
            else:
                # Apply Complementary Filter:
                #     - high-pass filter = theta * alpha:  allows short-duration signals to pass through while filtering out signals
                #       that are steady over time, is used to cancel out drift.
                #     - low-pass filter = accel * (1- alpha): lets through long term changes, filtering out short term fluctuations
                self.theta.x = self.theta.x * self.alpha + accel_angle.x * (1 - self.alpha)
                self.theta.z = self.theta.z * self.alpha + accel_angle.z * (1 - self.alpha)

    def get_theta(self):
        """Returns the current rotation angle"""
        with self.theta_mtx:
            return Vector3(self.theta.x, self.theta.y, self.theta.z)


class CppVisualizer:
    """Simple visualization showing EXACT C++ pitch behavior"""

    def __init__(self):
        if not MATPLOTLIB_AVAILABLE:
            raise ImportError("Matplotlib not available")

        self.fig = plt.figure(figsize=(18, 6))

        # Create three subplots: 3D view and two line charts
        self.ax3d = self.fig.add_subplot(131, projection='3d')
        self.ax_pitch = self.fig.add_subplot(132)
        self.ax_roll = self.fig.add_subplot(133)

        # 3D plot setup
        self.ax3d.set_xlim([-2, 2])
        self.ax3d.set_ylim([-2, 2])
        self.ax3d.set_zlim([-2, 2])
        self.ax3d.set_xlabel('X')
        self.ax3d.set_ylabel('Y')
        self.ax3d.set_zlabel('Z')
        self.ax3d.set_title('Camera Orientation (Exact C++)')

        # Pitch plot setup
        self.ax_pitch.set_xlim([0, 60])  # 60 seconds of data
        self.ax_pitch.set_ylim([-180, 180])
        self.ax_pitch.set_xlabel('Time (s)')
        self.ax_pitch.set_ylabel('Pitch (degrees)')
        self.ax_pitch.set_title('Pitch (X-axis)')
        self.ax_pitch.grid(True)

        # Roll plot setup
        self.ax_roll.set_xlim([0, 60])  # 60 seconds of data
        self.ax_roll.set_ylim([-180, 180])
        self.ax_roll.set_xlabel('Time (s)')
        self.ax_roll.set_ylabel('Roll (degrees)')
        self.ax_roll.set_title('Roll (Z-axis)')
        self.ax_roll.grid(True)

        # Data for plots
        self.time_data = []
        self.pitch_data = []
        self.roll_data = []
        self.start_time = time.time()

        plt.tight_layout()
        plt.ion()
        plt.show()

    def update(self, theta_vec):
        """Update both 3D and 2D visualizations"""
        # Clear 3D plot
        self.ax3d.clear()
        self.ax3d.set_xlim([-2, 2])
        self.ax3d.set_ylim([-2, 2])
        self.ax3d.set_zlim([-2, 2])
        self.ax3d.set_xlabel('X')
        self.ax3d.set_ylabel('Y')
        self.ax3d.set_zlabel('Z')
        self.ax3d.grid(True)

        # Apply EXACT same rotations as C++ OpenGL code
        # glRotatef(theta.x * 180 / PI_FL, 0, 0, -1);
        # glRotatef(theta.y * 180 / PI_FL, 0, -1, 0);
        # glRotatef((theta.z - PI_FL / 2) * 180 / PI_FL, -1, 0, 0);

        cos_x, sin_x = math.cos(theta_vec.x), math.sin(theta_vec.x)
        cos_y, sin_y = math.cos(theta_vec.y), math.sin(theta_vec.y)
        cos_z, sin_z = math.cos(theta_vec.z - math.pi/2), math.sin(theta_vec.z - math.pi/2)

        # Rotation matrices (matching OpenGL exactly)
        R_x = np.array([[cos_x, sin_x, 0],
                        [-sin_x, cos_x, 0],
                        [0, 0, 1]])

        R_y = np.array([[cos_y, 0, -sin_y],
                        [0, 1, 0],
                        [sin_y, 0, cos_y]])

        R_z = np.array([[1, 0, 0],
                        [0, cos_z, sin_z],
                        [0, -sin_z, cos_z]])

        R_total = R_x @ R_y @ R_z

        # Draw coordinate axes (matching C++ draw_axes)
        axis_length = 1.5
        axes_end = R_total @ np.array([[-axis_length, 0, 0],        # X to -X (red)
                                      [0, -axis_length, 0],         # Y to -Y (green)
                                      [0, 0, axis_length]]).T       # Z to +Z (blue)

        # Draw axes
        self.ax3d.plot([0, axes_end[0,0]], [0, axes_end[1,0]], [0, axes_end[2,0]], 'r-', linewidth=4, label='X')
        self.ax3d.plot([0, axes_end[0,1]], [0, axes_end[1,1]], [0, axes_end[2,1]], 'g-', linewidth=4, label='Y')
        self.ax3d.plot([0, axes_end[0,2]], [0, axes_end[1,2]], [0, axes_end[2,2]], 'b-', linewidth=4, label='Z')

        # Draw simple camera box
        camera_size = 0.3
        camera_vertices = np.array([
            [-camera_size, -camera_size/2, camera_size/3],
            [camera_size, -camera_size/2, camera_size/3],
            [camera_size, camera_size/2, camera_size/3],
            [-camera_size, camera_size/2, camera_size/3],
            [-camera_size, -camera_size/2, -camera_size/3],
            [camera_size, -camera_size/2, -camera_size/3],
            [camera_size, camera_size/2, -camera_size/3],
            [-camera_size, camera_size/2, -camera_size/3],
        ])

        rotated_camera = (R_total @ camera_vertices.T).T

        # Draw camera edges
        edges = [
            [0,1], [1,2], [2,3], [3,0],  # front face
            [4,5], [5,6], [6,7], [7,4],  # back face
            [0,4], [1,5], [2,6], [3,7]   # connecting edges
        ]

        for edge in edges:
            points = rotated_camera[edge]
            self.ax3d.plot3D(*points.T, 'k-', alpha=0.7)

        # Update title with angles - CORRECTED: roll is actually the vertical movement
        pitch_deg = math.degrees(theta_vec.x)
        yaw_deg = math.degrees(theta_vec.y)
        roll_deg = math.degrees(theta_vec.z)  # This is actually the vertical/pitch movement!

        self.ax3d.set_title(f'Exact C++ Algorithm\nX: {pitch_deg:.1f}° Y: {yaw_deg:.1f}° Z(Vertical): {roll_deg:.1f}°')
        self.ax3d.legend()

        # Update both pitch and roll plots
        current_time = time.time() - self.start_time
        self.time_data.append(current_time)
        self.pitch_data.append(pitch_deg)  # X-axis (traditional pitch)
        self.roll_data.append(roll_deg)    # Z-axis (the robust vertical one)

        # Keep only last 60 seconds
        while len(self.time_data) > 0 and self.time_data[0] < current_time - 60:
            self.time_data.pop(0)
            self.pitch_data.pop(0)
            self.roll_data.pop(0)

        # Update pitch plot
        self.ax_pitch.clear()
        self.ax_pitch.plot(self.time_data, self.pitch_data, 'b-', linewidth=2)
        self.ax_pitch.set_xlim([max(0, current_time-60), current_time])
        self.ax_pitch.set_ylim([-180, 180])
        self.ax_pitch.set_xlabel('Time (s)')
        self.ax_pitch.set_ylabel('Pitch (degrees)')
        self.ax_pitch.set_title('Pitch (X-axis) - Traditional pitch')
        self.ax_pitch.grid(True)
        self.ax_pitch.axhline(y=0, color='r', linestyle='--', alpha=0.5, label='Horizontal')
        self.ax_pitch.axhline(y=90, color='g', linestyle='--', alpha=0.5, label='Up')
        self.ax_pitch.axhline(y=-90, color='g', linestyle='--', alpha=0.5, label='Down')
        self.ax_pitch.legend()

        # Update roll plot
        self.ax_roll.clear()
        self.ax_roll.plot(self.time_data, self.roll_data, 'r-', linewidth=2)
        self.ax_roll.set_xlim([max(0, current_time-60), current_time])
        self.ax_roll.set_ylim([-180, 180])
        self.ax_roll.set_xlabel('Time (s)')
        self.ax_roll.set_ylabel('Roll (degrees)')
        self.ax_roll.set_title('Roll (Z-axis) - Robust vertical')
        self.ax_roll.grid(True)
        self.ax_roll.axhline(y=0, color='r', linestyle='--', alpha=0.5, label='Horizontal')
        self.ax_roll.axhline(y=90, color='g', linestyle='--', alpha=0.5, label='Up')
        self.ax_roll.axhline(y=-90, color='g', linestyle='--', alpha=0.5, label='Down')
        self.ax_roll.legend()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def start_imu_streaming():
    """Start IMU streaming with multiple fallback configurations"""
    ctx = rs.context()
    pipeline = rs.pipeline(ctx)

    # Try the same configurations that worked before
    configurations = [
        # Config 1: With frame rates
        {'accel': {'format': rs.format.motion_xyz32f, 'fps': 63},
         'gyro': {'format': rs.format.motion_xyz32f, 'fps': 200}},
        # Config 2: Different frame rates
        {'accel': {'format': rs.format.motion_xyz32f, 'fps': 100},
         'gyro': {'format': rs.format.motion_xyz32f, 'fps': 200}},
        # Config 3: No frame rates
        {'accel': {'format': rs.format.motion_xyz32f, 'fps': None},
         'gyro': {'format': rs.format.motion_xyz32f, 'fps': None}}
    ]

    for i, cfg in enumerate(configurations):
        try:
            config = rs.config()
            print(f"Trying configuration {i+1}...")

            # Enable accelerometer
            if cfg['accel']['fps']:
                config.enable_stream(rs.stream.accel, cfg['accel']['format'], cfg['accel']['fps'])
            else:
                config.enable_stream(rs.stream.accel, cfg['accel']['format'])

            # Enable gyroscope
            if cfg['gyro']['fps']:
                config.enable_stream(rs.stream.gyro, cfg['gyro']['format'], cfg['gyro']['fps'])
            else:
                config.enable_stream(rs.stream.gyro, cfg['gyro']['format'])

            # Try to start
            profile = pipeline.start(config)
            print(f"✓ Successfully started with configuration {i+1}")
            return pipeline, profile

        except Exception as e:
            print(f"✗ Configuration {i+1} failed: {e}")
            try:
                pipeline.stop()
            except:
                pass

    print("ERROR: All configurations failed!")
    return None, None


def main():
    """Main function"""
    parser = argparse.ArgumentParser(description='EXACT C++ rs-motion Algorithm Replication')
    parser.add_argument('--visualize', '-v', action='store_true', help='Enable visualization')

    args = parser.parse_args()

    print("RealSense EXACT C++ Algorithm Replication")
    print("=========================================")

    # Initialize visualizer
    visualizer = None
    if args.visualize:
        if not MATPLOTLIB_AVAILABLE:
            print("Error: matplotlib not available")
            return -1
        visualizer = CppVisualizer()
        print("✓ Visualization enabled")

    # Start IMU streaming
    pipeline, profile = start_imu_streaming()
    if pipeline is None:
        return -1

    # Create EXACT C++ estimator
    estimator = RotationEstimatorCpp()

    try:
        print("\nIMU streaming started. Press Ctrl+C to stop.")
        print("Watch the pitch values - they should be stable like C++!")
        print("-" * 60)

        start_time = time.time()
        last_print_time = start_time
        last_viz_time = start_time

        while True:
            try:
                frames = pipeline.wait_for_frames(timeout_ms=5000)

                for i in range(frames.size()):
                    frame = frames[i]

                    if frame.is_motion_frame():
                        motion_frame = frame.as_motion_frame()

                        if motion_frame.get_profile().stream_type() == rs.stream.gyro:
                            ts = motion_frame.get_timestamp()
                            gyro_data = motion_frame.get_motion_data()
                            estimator.process_gyro(gyro_data, ts)

                        elif motion_frame.get_profile().stream_type() == rs.stream.accel:
                            accel_data = motion_frame.get_motion_data()
                            estimator.process_accel(accel_data)

                current_time = time.time()

                # Update visualization at 30 Hz
                if visualizer and (current_time - last_viz_time >= 1/30.0):
                    theta = estimator.get_theta()
                    visualizer.update(theta)
                    last_viz_time = current_time

                # Print at 5 Hz
                if current_time - last_print_time >= 0.2:
                    theta = estimator.get_theta()
                    x_deg = math.degrees(theta.x)
                    y_deg = math.degrees(theta.y)
                    z_deg = math.degrees(theta.z)  # This is the robust vertical movement!

                    elapsed = current_time - start_time
                    print(f"\rTime: {elapsed:6.1f}s | "
                          f"X: {x_deg:7.2f}° | "
                          f"Y: {y_deg:7.2f}° | "
                          f"Z(Vertical): {z_deg:7.2f}°", end="", flush=True)

                    last_print_time = current_time

            except RuntimeError as e:
                if "timeout" in str(e).lower():
                    print(f"\rTimeout...", end="", flush=True)
                    continue
                else:
                    raise

    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        try:
            pipeline.stop()
        except:
            pass

    return 0


if __name__ == "__main__":
    sys.exit(main())