#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <chrono>
#include <numeric>

#include "staircase_perception_standalone/core/stair_detector.hpp"
#include "staircase_perception_standalone/utils/stair_utilities.hpp"
#include "staircase_perception_standalone/utils/line_extraction/line_extractor.hpp"
#include "staircase_perception_standalone/utils/config_parser.hpp"


void saveLineVisualization(const std::vector<Line>& lines, const std::string& filename) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr line_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    std::cout << "Saving " << lines.size() << " lines to " << filename << std::endl;

    for (size_t i = 0; i < lines.size(); ++i) {
        const Line& line = lines[i];

        // Get line endpoints
        const auto& start = line.getStart();
        const auto& end = line.getEnd();

        std::cout << "Line " << i << ": start(" << start[0] << "," << start[1] << "," << start[2]
                  << ") end(" << end[0] << "," << end[1] << "," << end[2]
                  << ") length=" << line.length() << std::endl;

        // Create points along the line for visualization
        int num_viz_points = 50;
        for (int j = 0; j <= num_viz_points; ++j) {
            float t = static_cast<float>(j) / num_viz_points;

            pcl::PointXYZRGB point;
            point.x = start[0] + t * (end[0] - start[0]);
            point.y = start[1] + t * (end[1] - start[1]);
            point.z = start[2] + t * (end[2] - start[2]);

            // Different color for each line
            switch (i % 6) {
                case 0: point.r = 255; point.g = 0; point.b = 0; break;     // Red
                case 1: point.r = 0; point.g = 255; point.b = 0; break;     // Green
                case 2: point.r = 0; point.g = 0; point.b = 255; break;     // Blue
                case 3: point.r = 255; point.g = 255; point.b = 0; break;   // Yellow
                case 4: point.r = 255; point.g = 0; point.b = 255; break;   // Magenta
                case 5: point.r = 0; point.g = 255; point.b = 255; break;   // Cyan
            }

            line_cloud->points.push_back(point);
        }
    }

    line_cloud->width = line_cloud->points.size();
    line_cloud->height = 1;
    line_cloud->is_dense = true;

    pcl::io::savePLYFile(filename, *line_cloud, true);
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

        // Get actual lines detected by the detector
        std::vector<Line> all_detector_lines;

        // Collect lines from all slices: above, ground, and below
        auto lines_above = detector.getDetectedLinesAbove();
        auto lines_ground = detector.getDetectedLinesGround();
        auto lines_below = detector.getDetectedLinesBelow();

        // Convert detector lines to Line objects for visualization
        for (const auto& slice : lines_above) {
            for (const auto& detected_line : *slice) {
                Line line(detected_line.line_theta, detected_line.line_radius, detected_line.line_covariance,
                         detected_line.line_start, detected_line.line_end, std::vector<unsigned int>());
                all_detector_lines.push_back(line);
            }
        }

        for (const auto& slice : lines_ground) {
            for (const auto& detected_line : *slice) {
                Line line(detected_line.line_theta, detected_line.line_radius, detected_line.line_covariance,
                         detected_line.line_start, detected_line.line_end, std::vector<unsigned int>());
                all_detector_lines.push_back(line);
            }
        }

        for (const auto& slice : lines_below) {
            for (const auto& detected_line : *slice) {
                Line line(detected_line.line_theta, detected_line.line_radius, detected_line.line_covariance,
                         detected_line.line_start, detected_line.line_end, std::vector<unsigned int>());
                all_detector_lines.push_back(line);
            }
        }

        std::cout << "Actual lines detected by detector: " << all_detector_lines.size() << "\n";

        // Count lines by length for debugging
        int short_lines = 0, long_lines = 0;
        for (const auto& line : all_detector_lines) {
            if (line.length() < line_params.min_line_length) {
                short_lines++;
            } else {
                long_lines++;
            }
        }
        std::cout << "Lines shorter than " << line_params.min_line_length << "m: " << short_lines << std::endl;
        std::cout << "Lines longer than " << line_params.min_line_length << "m: " << long_lines << std::endl;

        // Save line visualization of actual detector lines
        if (all_detector_lines.size() > 0) {
            saveLineVisualization(all_detector_lines, "actual_detector_lines.ply");
            std::cout << "Saved actual detector lines to: actual_detector_lines.ply" << std::endl;
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