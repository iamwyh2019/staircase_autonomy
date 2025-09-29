#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <chrono>

#include "staircase_perception_standalone/core/stair_detector.hpp"
#include "staircase_perception_standalone/utils/stair_utilities.hpp"
#include "staircase_perception_standalone/utils/line_extraction/line_extractor.hpp"
#include "staircase_perception_standalone/utils/config_parser.hpp"


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

    // Load configuration
    std::string config_file = "../config/standalone_detection_config.yaml";
    if (argc > 2) {
        config_file = argv[2];
    }

    std::cout << "Loading config from: " << config_file << "\n";
    stair_utility::ConfigParser config(config_file);

    // Create parameters from config
    auto detector_params = config.getDetectorParams();
    auto line_params = config.getLineExtractorParams();

    // Set line extractor min_line_length equal to stair param min_stair_width
    line_params.min_line_length = detector_params.min_stair_width;

    std::cout << "Using min_line_length = min_stair_width = " << line_params.min_line_length << "m\n";

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