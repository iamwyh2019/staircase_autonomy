#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <chrono>

#include "staircase_perception_standalone/core/stair_detector.hpp"
#include "staircase_perception_standalone/utils/stair_utilities.hpp"
#include "staircase_perception_standalone/utils/line_extraction/line_extractor.hpp"

// Function to create default parameters for testing
stair_utility::StaircaseDetectorParams createDefaultDetectorParams() {
    stair_utility::StaircaseDetectorParams params;

    // Detection parameters
    params.use_ramp_detection = true;
    params.angle_resolution = 2.0;  // degrees
    params.leaf_size = 0.05;        // 5cm voxel size

    // Robot parameters - adjusted for coordinate system where robot is at bottom
    params.robot_height = 0.0;      // Robot at z=0 (bottom of scene)
    params.initialization_range = 2.0;  // 2m initialization range
    params.ground_height_buffer = 0.2;  // 20cm ground buffer (more lenient)

    // Stair parameters
    params.min_stair_count = 3;
    params.stair_slope_min = 0.2;   // ~11 degrees
    params.stair_slope_max = 1.2;   // ~50 degrees

    params.min_stair_width = 0.3;   // 30cm minimum width
    params.min_stair_height = 0.1;  // 10cm minimum height
    params.max_stair_height = 0.25; // 25cm maximum height
    params.min_stair_depth = 0.2;   // 20cm minimum depth
    params.max_stair_depth = 0.4;   // 40cm maximum depth
    params.max_stair_curvature = 0.3; // Maximum curvature

    // Point cloud bounds (robot-centered) - adjusted for your data
    params.x_max = 5.0;   // 5m forward
    params.x_min = -2.0;  // 2m backward (your data goes to -1.5)
    params.y_max = 3.0;   // 3m left
    params.y_min = -3.0;  // 3m right
    params.z_max = 4.0;   // 4m up (your data goes to 3.6)
    params.z_min = -1.0;  // 1m down

    return params;
}

stair_utility::LineExtractorParams createDefaultLineParams() {
    stair_utility::LineExtractorParams params;

    params.bearing_var = 0.05;
    params.range_var = 0.02;
    params.z_var = 0.01;
    params.least_sq_angle_thresh = 0.1;
    params.least_sq_radius_thresh = 0.05;
    params.max_line_gap = 0.1;
    params.min_line_length = 0.3;
    params.min_range = 0.2;
    params.max_range = 10.0;
    params.min_split_dist = 0.05;
    params.outlier_dist = 0.05;
    params.min_line_points = 5;

    return params;
}

void printStaircaseResults(const stair_utility::StaircaseMeasurement& stair_measurement, const std::string& direction) {
    std::cout << "\n=== " << direction << " Staircase Detected ===\n";
    std::cout << "Step count: " << static_cast<int>(stair_measurement.stair_count) << "\n";

    for (size_t i = 0; i < stair_measurement.steps.size(); ++i) {
        const auto& step = stair_measurement.steps[i];
        std::cout << "Step " << i + 1 << ":\n";
        std::cout << "  Start: (" << step.start_p(0) << ", " << step.start_p(1) << ", " << step.start_p(2) << ")\n";
        std::cout << "  End:   (" << step.end_p(0) << ", " << step.end_p(1) << ", " << step.end_p(2) << ")\n";
        std::cout << "  Width: " << step.step_width << "m\n";
        std::cout << "  Polar: r=" << step.line_polar_form(0) << ", θ=" << step.line_polar_form(1) << "\n";
    }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr generateSampleStaircase() {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    std::cout << "Generating synthetic staircase point cloud...\n";

    // Create a simple 5-step staircase going up
    int num_steps = 5;
    float step_height = 0.15;  // 15cm
    float step_depth = 0.3;    // 30cm
    float step_width = 1.0;    // 1m

    for (int step = 0; step < num_steps; ++step) {
        float z = step * step_height;
        float x_start = 1.0 + step * step_depth;
        float x_end = x_start + step_depth;

        // Add points for the step surface (top)
        for (float x = x_start; x <= x_end; x += 0.02) {
            for (float y = -step_width/2; y <= step_width/2; y += 0.02) {
                pcl::PointXYZI point;
                point.x = x;
                point.y = y;
                point.z = z;
                point.intensity = 100;
                cloud->points.push_back(point);
            }
        }

        // Add points for the step riser (vertical part)
        if (step > 0) {
            for (float z_riser = (step-1) * step_height; z_riser <= z; z_riser += 0.02) {
                for (float y = -step_width/2; y <= step_width/2; y += 0.02) {
                    pcl::PointXYZI point;
                    point.x = x_start;
                    point.y = y;
                    point.z = z_riser;
                    point.intensity = 100;
                    cloud->points.push_back(point);
                }
            }
        }
    }

    // Add some ground points
    for (float x = 0.5; x <= 1.0; x += 0.02) {
        for (float y = -step_width/2; y <= step_width/2; y += 0.02) {
            pcl::PointXYZI point;
            point.x = x;
            point.y = y;
            point.z = 0.0;
            point.intensity = 50;
            cloud->points.push_back(point);
        }
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;

    std::cout << "Generated " << cloud->points.size() << " points\n";
    return cloud;
}

int main(int argc, char** argv) {
    std::cout << "=== Staircase Detector Standalone Test (Phase 1) ===\n";

    // Create parameters
    auto detector_params = createDefaultDetectorParams();
    auto line_params = createDefaultLineParams();

    // Initialize detector
    std::cout << "Initializing StairDetector...\n";
    StairDetector detector(detector_params, line_params);

    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud;

    // Try to load PCD file if provided, otherwise generate synthetic data
    if (argc > 1) {
        std::string pcd_file = argv[1];
        std::cout << "Loading point cloud from: " << pcd_file << "\n";

        input_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_file, *input_cloud) == -1) {
            std::cerr << "Error: Could not read file " << pcd_file << "\n";
            std::cout << "Falling back to synthetic data...\n";
            input_cloud = generateSampleStaircase();
        } else {
            std::cout << "Loaded " << input_cloud->points.size() << " points\n";
        }
    } else {
        std::cout << "No PCD file provided. Using synthetic staircase data.\n";
        input_cloud = generateSampleStaircase();
    }

    // Apply basic filtering
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    voxel_filter.setInputCloud(input_cloud);
    voxel_filter.setLeafSize(detector_params.leaf_size, detector_params.leaf_size, detector_params.leaf_size);

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    voxel_filter.filter(*filtered_cloud);

    std::cout << "After voxel filtering: " << filtered_cloud->points.size() << " points\n";


    // Set input cloud
    detector.setPointCloudAndOdometry(filtered_cloud);

    // Run detection
    std::cout << "\nRunning staircase detection...\n";

    stair_utility::StaircaseMeasurement stair_up, stair_down;

    auto start_time = std::chrono::high_resolution_clock::now();
    stair_utility::StaircaseDetectorResult result = detector.detectStaircase(stair_up, stair_down);
    auto end_time = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "Detection completed in " << duration.count() << "ms\n";

    // Print results
    std::cout << "\n=== DETECTION RESULTS ===\n";

    switch (result) {
        case stair_utility::StaircaseDetectorResult::NoStairsDetected:
            std::cout << "No staircases detected.\n";
            break;
        case stair_utility::StaircaseDetectorResult::StairsDetectedUp:
            std::cout << "Staircase detected going UP only.\n";
            printStaircaseResults(stair_up, "ASCENDING");
            break;
        case stair_utility::StaircaseDetectorResult::StairsDetectedDown:
            std::cout << "Staircase detected going DOWN only.\n";
            printStaircaseResults(stair_down, "DESCENDING");
            break;
        case stair_utility::StaircaseDetectorResult::StairsDetectedBoth:
            std::cout << "Staircases detected in BOTH directions.\n";
            printStaircaseResults(stair_up, "ASCENDING");
            printStaircaseResults(stair_down, "DESCENDING");
            break;
    }

    // Save processed clouds and test line extraction
    pcl::PointCloud<pcl::PointXYZI>::Ptr processed_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    std::cout << "\nSaving processed point clouds and testing line extraction...\n";

    // Save top-down projection
    detector.getProcessedCloud(processed_cloud, 1);
    if (!processed_cloud->empty()) {
        pcl::io::savePCDFileASCII("topdown_projection.pcd", *processed_cloud);
        std::cout << "Saved top-down projection to: topdown_projection.pcd\n";

        // Test line extraction on detector's internal projection
        int non_zero = 0;
        for (const auto& point : processed_cloud->points) {
            if (point.x != 0 || point.y != 0 || point.z != 0) non_zero++;
        }
        std::cout << "Detector's top-down: " << processed_cloud->size() << " total, " << non_zero << " non-zero points\n";

        if (non_zero > 0) {
            // Quick line extraction test
            std::vector<float> ranges, xs, ys, zs, bearings, cos_bearings, sin_bearings;
            std::vector<unsigned int> indices;

            for (size_t i = 0; i < processed_cloud->points.size(); ++i) {
                const auto& point = processed_cloud->points[i];
                if (point.x == 0 && point.y == 0 && point.z == 0) continue;

                float range = sqrt(point.x*point.x + point.y*point.y);
                float bearing = atan2(point.y, point.x);

                ranges.push_back(range);
                xs.push_back(point.x);
                ys.push_back(point.y);
                zs.push_back(point.z);
                bearings.push_back(bearing);
                cos_bearings.push_back(cos(bearing));
                sin_bearings.push_back(sin(bearing));
                indices.push_back(indices.size());
            }

            if (ranges.size() >= 3) {
                LineExtractor extractor;
                extractor.setPrecomputedCache(bearings, cos_bearings, sin_bearings, indices);
                extractor.setRangeData(ranges, xs, ys, zs);
                extractor.setBearingVariance(0.05);
                extractor.setRangeVariance(0.02);
                extractor.setZVariance(0.01);
                extractor.setLeastSqAngleThresh(0.1);
                extractor.setLeastSqRadiusThresh(0.05);
                extractor.setMaxLineGap(0.1);
                extractor.setMinLineLength(0.3);
                extractor.setMinRange(0.2);
                extractor.setMaxRange(10.0);
                extractor.setMinSplitDist(0.05);
                extractor.setOutlierDist(0.05);
                extractor.setMinLinePoints(5);

                std::vector<Line> lines;
                extractor.extractLines(lines);
                std::cout << "Lines extracted from detector's projection: " << lines.size() << "\n";
            }
        }
    }

    // Save cylindrical projection
    detector.getProcessedCloud(processed_cloud, 2);
    if (!processed_cloud->empty()) {
        pcl::io::savePCDFileASCII("cylindrical_projection.pcd", *processed_cloud);
        std::cout << "Saved cylindrical projection to: cylindrical_projection.pcd\n";
    }

    std::cout << "\n=== TEST COMPLETED ===\n";
    return 0;
}