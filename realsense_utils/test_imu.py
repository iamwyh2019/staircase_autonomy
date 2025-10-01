#!/usr/bin/env python3
"""
Simple IMU test for RealSense D455
Tests basic IMU functionality without complex processing
"""

import pyrealsense2 as rs
import time
import sys


def main():
    print("RealSense IMU Test")
    print("==================")

    # Create pipeline
    pipeline = rs.pipeline()
    config = rs.config()

    # Try the simplest IMU configuration
    try:
        print("Enabling IMU streams...")
        config.enable_stream(rs.stream.accel)
        config.enable_stream(rs.stream.gyro)

        print("Starting pipeline...")
        pipeline.start(config)

        print("✓ Pipeline started successfully!")
        print("\nReceiving IMU data (press Ctrl+C to stop):")
        print("Format: [timestamp] accel(x,y,z) gyro(x,y,z)")
        print("-" * 60)

        frame_count = 0
        start_time = time.time()

        while True:
            frames = pipeline.wait_for_frames(timeout_ms=1000)

            for frame in frames:
                if frame.is_motion_frame():
                    motion = frame.as_motion_frame()
                    data = motion.get_motion_data()
                    timestamp = motion.get_timestamp()
                    stream_type = motion.get_profile().stream_type()

                    frame_count += 1

                    if frame_count % 50 == 0:  # Print every 50th frame
                        if stream_type == rs.stream.accel:
                            print(f"[{timestamp:10.0f}] ACCEL: ({data.x:6.3f}, {data.y:6.3f}, {data.z:6.3f})")
                        elif stream_type == rs.stream.gyro:
                            print(f"[{timestamp:10.0f}] GYRO:  ({data.x:6.3f}, {data.y:6.3f}, {data.z:6.3f})")

            # Print statistics every 5 seconds
            elapsed = time.time() - start_time
            if elapsed >= 5.0:
                fps = frame_count / elapsed
                print(f"\nStats: {frame_count} frames in {elapsed:.1f}s = {fps:.1f} FPS")
                frame_count = 0
                start_time = time.time()

    except KeyboardInterrupt:
        print("\n\nStopping...")

    except Exception as e:
        print(f"\nError: {e}")
        print("\nThis could mean:")
        print("- IMU is not supported on this device")
        print("- Device is already in use")
        print("- pyrealsense2 version issue")
        return -1

    finally:
        try:
            pipeline.stop()
            print("Pipeline stopped.")
        except:
            pass

    return 0


if __name__ == "__main__":
    sys.exit(main())