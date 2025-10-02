#!/usr/bin/env python3
"""
RealSense Orientation Tracker API
Provides gravity-aligned orientation tracking for point cloud correction
"""

import pyrealsense2 as rs
import numpy as np
import threading
import time
import math


class Vector3:
    """Simple 3D vector class"""
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

    def add(self, dx, dy, dz):
        self.x += dx
        self.y += dy
        self.z += dz


class OrientationTracker:
    """
    RealSense camera orientation tracker for gravity alignment

    Usage:
        tracker = OrientationTracker()
        tracker.start()

        # Get current orientation
        pitch, roll, yaw = tracker.get_orientation()

        # Get rotation matrix for point cloud correction
        R = tracker.get_rotation_matrix()

        tracker.stop()
    """

    def __init__(self):
        # Orientation state
        self.theta = Vector3()
        self.theta_mtx = threading.Lock()
        self.alpha = 0.98  # Complementary filter weight

        # Initialization flags
        self.firstGyro = True
        self.firstAccel = True
        self.last_ts_gyro = 0

        # IMU pipeline
        self.pipeline = None
        self.running = False
        self.thread = None

    def start(self):
        """Start orientation tracking in background thread"""
        if self.running:
            print("Orientation tracker already running")
            return False

        # Setup IMU pipeline
        if not self._setup_imu():
            return False

        # Start background thread
        self.running = True
        self.thread = threading.Thread(target=self._tracking_loop, daemon=True)
        self.thread.start()

        print("Orientation tracker started. Initializing...")
        time.sleep(1.0)  # Allow time for initialization
        print("Orientation tracker ready.")
        return True

    def stop(self):
        """Stop orientation tracking"""
        if not self.running:
            return

        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)

        if self.pipeline:
            try:
                self.pipeline.stop()
            except:
                pass

        print("Orientation tracker stopped.")

    def get_orientation(self):
        """
        Get current camera orientation in degrees

        Returns:
            tuple: (pitch, roll, yaw) in degrees
                   pitch: rotation around Y-axis (up/down tilt) - VERTICAL movement
                   roll: rotation around Z-axis (left/right tilt)
                   yaw: rotation around X-axis (left/right turn)
        """
        with self.theta_mtx:
            # Fix the axis mapping:
            # theta.z is the robust VERTICAL axis = should be PITCH (up/down)
            # theta.x should be ROLL (left/right tilt)
            pitch = math.degrees(self.theta.z)  # Robust vertical = pitch
            roll = math.degrees(self.theta.x)   # Left/right tilt = roll
            yaw = math.degrees(self.theta.y)
            return pitch, roll, yaw

    def get_rotation_matrix(self):
        """
        Get rotation matrix to transform points to gravity-aligned frame
        Coordinate system: X=forward, Y=left, Z=up

        Returns:
            np.ndarray: 3x3 rotation matrix that corrects for camera tilt
                       Use: corrected_points = points @ R.T
        """
        with self.theta_mtx:
            # RealSense IMU gives orientation in RealSense coordinate system
            # Points have already been transformed to target system (X=forward, Y=left, Z=up)

            # In our target coordinate system:
            # - theta.x (IMU pitch) affects rotation around Y-axis (left/right tilt correction)
            # - theta.z (IMU roll) affects rotation around X-axis (forward/back tilt correction)

            # Fix axis mapping to match corrected get_orientation():
            # theta.z = pitch (up/down, vertical) = robust vertical axis
            # theta.x = roll (left/right tilt)
            raw_pitch = self.theta.z  # Vertical movement (robust)
            raw_roll = self.theta.x   # Left/right tilt

            # When P=-90, R=0 is perfectly aligned (no correction needed)
            # So we need to apply correction relative to this reference
            corrected_pitch = raw_pitch - math.radians(-90)  # P=-90 is the target
            corrected_roll = raw_roll - 0                    # R=0 is the target

            # For gravity alignment in target coordinate system (X=forward, Y=left, Z=up):
            # - Pitch correction: rotate around Y-axis to make table parallel to X-Y plane
            # - Roll correction: rotate around X-axis to level left/right tilt

            # Rotation around Y-axis (left) to correct pitch (up/down tilt)
            cos_y, sin_y = math.cos(-corrected_pitch), math.sin(-corrected_pitch)
            R_y = np.array([
                [cos_y, 0, sin_y],
                [0, 1, 0],
                [-sin_y, 0, cos_y]
            ])

            # Rotation around X-axis (forward) to correct roll (left/right tilt)
            # Fix: apply correction in opposite direction
            cos_x, sin_x = math.cos(corrected_roll), math.sin(corrected_roll)  # Removed negative sign
            R_x = np.array([
                [1, 0, 0],
                [0, cos_x, -sin_x],
                [0, sin_x, cos_x]
            ])

            # Debug output
            if abs(corrected_roll) > 0.1 or abs(corrected_pitch) > 0.1:  # Only print when corrections are significant
                print(f"Applying corrections: Pitch={math.degrees(corrected_pitch):.1f}°, Roll={math.degrees(corrected_roll):.1f}°")

            return R_y @ R_x

    def get_gravity_aligned_transform(self):
        """
        Get transformation matrix for gravity-aligned point clouds

        Returns:
            np.ndarray: 4x4 transformation matrix for homogeneous coordinates
        """
        R = self.get_rotation_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        return T

    def is_ready(self):
        """Check if tracker is initialized and ready"""
        return self.running and not self.firstAccel

    def _setup_imu(self):
        """Setup IMU pipeline with fallback configurations"""
        ctx = rs.context()
        self.pipeline = rs.pipeline(ctx)

        configurations = [
            {'accel': {'format': rs.format.motion_xyz32f, 'fps': 63},
             'gyro': {'format': rs.format.motion_xyz32f, 'fps': 200}},
            {'accel': {'format': rs.format.motion_xyz32f, 'fps': 100},
             'gyro': {'format': rs.format.motion_xyz32f, 'fps': 200}},
            {'accel': {'format': rs.format.motion_xyz32f, 'fps': None},
             'gyro': {'format': rs.format.motion_xyz32f, 'fps': None}}
        ]

        for i, cfg in enumerate(configurations):
            try:
                config = rs.config()

                if cfg['accel']['fps']:
                    config.enable_stream(rs.stream.accel, cfg['accel']['format'], cfg['accel']['fps'])
                else:
                    config.enable_stream(rs.stream.accel, cfg['accel']['format'])

                if cfg['gyro']['fps']:
                    config.enable_stream(rs.stream.gyro, cfg['gyro']['format'], cfg['gyro']['fps'])
                else:
                    config.enable_stream(rs.stream.gyro, cfg['gyro']['format'])

                profile = self.pipeline.start(config)
                return True

            except Exception as e:
                print(f"IMU configuration {i+1} failed: {e}")
                try:
                    self.pipeline.stop()
                except:
                    pass

        print("ERROR: All IMU configurations failed!")
        return False

    def _tracking_loop(self):
        """Main tracking loop running in background thread"""
        while self.running:
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=1000)

                for i in range(frames.size()):
                    frame = frames[i]

                    if frame.is_motion_frame():
                        motion_frame = frame.as_motion_frame()

                        if motion_frame.get_profile().stream_type() == rs.stream.gyro:
                            ts = motion_frame.get_timestamp()
                            gyro_data = motion_frame.get_motion_data()
                            self._process_gyro(gyro_data, ts)

                        elif motion_frame.get_profile().stream_type() == rs.stream.accel:
                            accel_data = motion_frame.get_motion_data()
                            self._process_accel(accel_data)

            except RuntimeError as e:
                if "timeout" in str(e).lower():
                    continue
                else:
                    print(f"IMU tracking error: {e}")
                    break
            except Exception as e:
                print(f"Unexpected error: {e}")
                break

    def _process_gyro(self, gyro_data, ts):
        """Process gyroscope data (exact C++ algorithm)"""
        if self.firstGyro:
            self.firstGyro = False
            self.last_ts_gyro = ts
            return

        gyro_angle = Vector3()
        gyro_angle.x = gyro_data.x
        gyro_angle.y = gyro_data.y
        gyro_angle.z = gyro_data.z

        dt_gyro = (ts - self.last_ts_gyro) / 1000.0
        self.last_ts_gyro = ts

        gyro_angle.x *= dt_gyro
        gyro_angle.y *= dt_gyro
        gyro_angle.z *= dt_gyro

        with self.theta_mtx:
            self.theta.add(-gyro_angle.z, -gyro_angle.y, gyro_angle.x)

    def _process_accel(self, accel_data):
        """Process accelerometer data (exact C++ algorithm)"""
        accel_angle = Vector3()

        accel_angle.z = math.atan2(accel_data.y, accel_data.z)
        accel_angle.x = math.atan2(accel_data.x, math.sqrt(accel_data.y * accel_data.y + accel_data.z * accel_data.z))

        with self.theta_mtx:
            if self.firstAccel:
                self.firstAccel = False
                self.theta = accel_angle
                self.theta.y = math.pi
            else:
                # Complementary filter
                self.theta.x = self.theta.x * self.alpha + accel_angle.x * (1 - self.alpha)
                self.theta.z = self.theta.z * self.alpha + accel_angle.z * (1 - self.alpha)


def main():
    """Demo usage"""
    tracker = OrientationTracker()

    if not tracker.start():
        print("Failed to start orientation tracker")
        return -1

    try:
        print("\nOrientation Tracker Demo")
        print("========================")
        print("Move the camera around to see orientation changes")
        print("Press Ctrl+C to stop\n")

        while True:
            if tracker.is_ready():
                pitch, roll, yaw = tracker.get_orientation()
                R = tracker.get_rotation_matrix()

                print(f"\rPitch: {pitch:7.2f}° Roll: {roll:7.2f}° Yaw: {yaw:7.2f}° | "
                      f"Matrix norm: {np.linalg.norm(R):.3f}", end="", flush=True)
            else:
                print("\rInitializing...", end="", flush=True)

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        tracker.stop()

    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())