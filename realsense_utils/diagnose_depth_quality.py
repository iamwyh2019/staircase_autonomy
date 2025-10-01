#!/usr/bin/env python3
"""
RealSense D455 Depth Quality Diagnostics
Investigates why depth quality might be poor
"""

import pyrealsense2 as rs
import numpy as np
import cv2
import time


def check_infrared_projector():
    """Check if IR projector is working properly"""
    print("IR PROJECTOR DIAGNOSTICS:")
    print("-" * 30)

    pipeline = rs.pipeline()
    config = rs.config()

    # Enable IR streams to see projector pattern
    config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)  # Left IR
    config.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 30)  # Right IR
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    try:
        profile = pipeline.start(config)
        device = profile.get_device()
        depth_sensor = device.first_depth_sensor()

        # Set laser to maximum
        if depth_sensor.supports(rs.option.laser_power):
            depth_sensor.set_option(rs.option.laser_power, 360)
            print(f"✓ Laser power set to: {depth_sensor.get_option(rs.option.laser_power)}")

        print("\nCapturing IR images to check projector pattern...")
        print("Look for dot pattern in IR Left image - this indicates projector is working")

        for i in range(10):  # Wait for frames to stabilize
            pipeline.wait_for_frames()

        frames = pipeline.wait_for_frames()
        ir_left = frames.get_infrared_frame(1)
        ir_right = frames.get_infrared_frame(2)
        depth_frame = frames.get_depth_frame()

        if ir_left and ir_right and depth_frame:
            ir_left_image = np.asanyarray(ir_left.get_data())
            ir_right_image = np.asanyarray(ir_right.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            # Analyze IR pattern strength
            ir_left_std = np.std(ir_left_image)
            ir_right_std = np.std(ir_right_image)

            print(f"\nIR Analysis:")
            print(f"  Left IR std deviation: {ir_left_std:.1f} (should be >10 with projector)")
            print(f"  Right IR std deviation: {ir_right_std:.1f}")
            print(f"  Depth valid pixels: {np.sum(depth_image > 0)} / {depth_image.size}")
            print(f"  Depth fill rate: {np.sum(depth_image > 0) / depth_image.size * 100:.1f}%")

            # Check for projector pattern
            if ir_left_std > 15:
                print("  ✓ IR projector appears to be working (good texture variation)")
            else:
                print("  ⚠ IR projector may be weak or blocked (low texture variation)")

            # Save IR images for inspection
            cv2.imwrite('ir_left_diagnostic.png', ir_left_image)
            cv2.imwrite('ir_right_diagnostic.png', ir_right_image)
            cv2.imwrite('depth_diagnostic.png', cv2.convertScaleAbs(depth_image, alpha=0.03))

            print(f"\n✓ Saved diagnostic images:")
            print(f"  - ir_left_diagnostic.png (look for dot pattern)")
            print(f"  - ir_right_diagnostic.png")
            print(f"  - depth_diagnostic.png")

            return ir_left_std > 15, np.sum(depth_image > 0) / depth_image.size

    except Exception as e:
        print(f"Error in IR diagnostics: {e}")
        return False, 0

    finally:
        pipeline.stop()


def check_environmental_factors():
    """Check environmental factors that affect depth quality"""
    print("\nENVIRONMENTAL FACTORS CHECK:")
    print("-" * 30)

    print("Please check the following:")
    print("1. ⚠ LIGHTING: Too much sunlight/IR light interferes with depth")
    print("   - Try in indoor lighting, avoid direct sunlight")
    print("   - Fluorescent lights can sometimes interfere")

    print("\n2. ⚠ SURFACE MATERIALS: Some materials don't work well")
    print("   - Shiny/reflective surfaces (mirrors, glass, metal)")
    print("   - Very dark surfaces (absorb IR light)")
    print("   - Transparent materials (glass, clear plastic)")
    print("   - Fine textures (fabric, hair) may be noisy")

    print("\n3. ⚠ DISTANCE: D455 has optimal range")
    print("   - Minimum distance: ~20cm")
    print("   - Optimal range: 0.3m - 3m")
    print("   - Maximum range: ~9m (but quality degrades)")

    print("\n4. ⚠ CAMERA POSITIONING:")
    print("   - Keep camera level/stable")
    print("   - Avoid rapid movements during capture")
    print("   - Clean lens (especially IR emitter/receivers)")


def test_different_presets():
    """Test different depth presets to find best quality"""
    print("\nTESTING DEPTH PRESETS:")
    print("-" * 30)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Available presets (these may vary by device)
    presets = [
        (rs.option.visual_preset, 0, "Custom"),
        (rs.option.visual_preset, 1, "Default"),
        (rs.option.visual_preset, 2, "Hand"),
        (rs.option.visual_preset, 3, "High Accuracy"),
        (rs.option.visual_preset, 4, "High Density"),
        (rs.option.visual_preset, 5, "Medium Density"),
    ]

    try:
        profile = pipeline.start(config)
        device = profile.get_device()
        depth_sensor = device.first_depth_sensor()

        if not depth_sensor.supports(rs.option.visual_preset):
            print("Visual presets not supported on this device")
            return

        align = rs.align(rs.stream.color)

        for preset_option, preset_value, preset_name in presets:
            try:
                print(f"\nTesting preset: {preset_name}")
                depth_sensor.set_option(preset_option, preset_value)

                # Let camera adjust
                for i in range(10):
                    pipeline.wait_for_frames()

                # Capture and analyze
                frames = pipeline.wait_for_frames()
                aligned_frames = align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()

                if depth_frame:
                    depth_image = np.asanyarray(depth_frame.get_data())
                    valid_pixels = np.sum(depth_image > 0)
                    fill_rate = valid_pixels / depth_image.size * 100

                    # Calculate noise (std dev of valid pixels)
                    valid_depths = depth_image[depth_image > 0]
                    if len(valid_depths) > 0:
                        depth_noise = np.std(valid_depths)
                        print(f"  Fill rate: {fill_rate:.1f}%, Noise: {depth_noise:.1f}mm")

                        # Save sample image
                        depth_vis = cv2.convertScaleAbs(depth_image, alpha=0.03)
                        cv2.imwrite(f'depth_preset_{preset_name.replace(" ", "_").lower()}.png', depth_vis)
                    else:
                        print(f"  No valid depth data")

            except Exception as e:
                print(f"  Failed to test {preset_name}: {e}")

    except Exception as e:
        print(f"Error testing presets: {e}")

    finally:
        pipeline.stop()


def check_firmware_and_calibration():
    """Check firmware version and calibration status"""
    print("\nFIRMWARE & CALIBRATION CHECK:")
    print("-" * 30)

    ctx = rs.context()
    devices = ctx.query_devices()

    if len(devices) == 0:
        print("No devices found")
        return

    device = devices[0]

    # Check firmware
    fw_version = device.get_info(rs.camera_info.firmware_version)
    print(f"Firmware version: {fw_version}")

    # Latest firmware for D455 as of 2024 is around 5.16.x
    major, minor = map(int, fw_version.split('.')[:2])
    if major < 5 or (major == 5 and minor < 15):
        print("  ⚠ Firmware may be outdated. Consider updating to latest version.")
        print("  📥 Download from: https://www.intelrealsense.com/developers/")
    else:
        print("  ✓ Firmware appears up to date")

    # Check calibration (get camera intrinsics)
    try:
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        profile = pipeline.start(config)
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()

        intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics

        print(f"\nCamera intrinsics:")
        print(f"  Resolution: {intrinsics.width}x{intrinsics.height}")
        print(f"  Focal length: fx={intrinsics.fx:.1f}, fy={intrinsics.fy:.1f}")
        print(f"  Principal point: cx={intrinsics.ppx:.1f}, cy={intrinsics.ppy:.1f}")

        # Check if values seem reasonable
        if abs(intrinsics.fx - intrinsics.fy) > 10:
            print("  ⚠ Focal lengths significantly different - may indicate calibration issue")
        else:
            print("  ✓ Focal lengths look reasonable")

        pipeline.stop()

    except Exception as e:
        print(f"Error checking calibration: {e}")


def main():
    print("RealSense D455 Depth Quality Diagnostics")
    print("=" * 50)
    print("This will help identify why depth quality might be poor")
    print()

    # Run all diagnostics
    projector_ok, fill_rate = check_infrared_projector()

    check_environmental_factors()

    check_firmware_and_calibration()

    test_different_presets()

    print("\n" + "=" * 50)
    print("SUMMARY & RECOMMENDATIONS:")
    print("=" * 50)

    if not projector_ok:
        print("🔴 IR PROJECTOR ISSUE DETECTED")
        print("   - Check if projector lens is clean")
        print("   - Check if projector is physically blocked")
        print("   - May need RMA if hardware failure")

    if fill_rate < 0.3:
        print("🔴 LOW DEPTH FILL RATE")
        print("   - Check environmental factors (lighting, surfaces)")
        print("   - Try different distance from objects")
        print("   - Clean camera lenses")

    print("\n📋 NEXT STEPS:")
    print("1. Review saved diagnostic images")
    print("2. Try depth capture in different lighting conditions")
    print("3. Test with different surface materials")
    print("4. If hardware issue suspected, contact Intel support")

    print(f"\n📁 Files saved in current directory:")
    print("   - ir_left_diagnostic.png")
    print("   - ir_right_diagnostic.png")
    print("   - depth_diagnostic.png")
    print("   - depth_preset_*.png (multiple preset comparisons)")


if __name__ == "__main__":
    main()