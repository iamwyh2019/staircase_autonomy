#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

#include "staircase_perception_standalone/core/stair_detector.hpp"
#include "staircase_perception_standalone/utils/stair_utilities.hpp"
#include "staircase_perception_standalone/utils/line_extraction/line_extractor.hpp"

stair_utility::LineExtractorParams createDefaultLineParams() {
    stair_utility::LineExtractorParams params;
    params.bearing_var = 0.0001;
    params.range_var = 0.001;
    params.z_var = 0.0004;
    params.least_sq_angle_thresh = 0.05;
    params.least_sq_radius_thresh = 0.075;
    params.max_line_gap = 0.2;
    params.min_line_length = 0.75;
    params.min_range = 0.1;
    params.max_range = 5.0;
    params.min_split_dist = 0.2;
    params.outlier_dist = 0.2;
    params.min_line_points = 7;
    return params;
}

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
                  << ") length=" << line.length() << " points=" << line.numPoints() << std::endl;

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

void debugProjectionLineExtraction(pcl::PointCloud<pcl::PointXYZI>::Ptr projection,
                                   const std::string& projection_name,
                                   const stair_utility::LineExtractorParams& line_params) {

    std::cout << "\n=== " << projection_name << " Line Extraction Debug ===\n";

    if (projection->empty()) {
        std::cout << "Projection is empty!" << std::endl;
        return;
    }

    // Count non-zero points
    int non_zero = 0;
    for (const auto& point : projection->points) {
        if (point.x != 0 || point.y != 0 || point.z != 0) non_zero++;
    }
    std::cout << "Projection: " << projection->size() << " total, " << non_zero << " non-zero points\n";

    if (non_zero == 0) {
        std::cout << "No non-zero points for line extraction!" << std::endl;
        return;
    }

    std::cout << "Found " << non_zero << " valid points - proceeding with line extraction\n";

    // Prepare data for line extraction
    std::vector<float> ranges, xs, ys, zs, bearings, cos_bearings, sin_bearings;
    std::vector<unsigned int> indices;

    for (size_t i = 0; i < projection->points.size(); ++i) {
        const auto& point = projection->points[i];

        // Skip zero points
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

    std::cout << "Prepared " << ranges.size() << " points for line extraction\n";

    if (ranges.size() < 3) {
        std::cout << "Not enough points for line extraction!" << std::endl;
        return;
    }

    // Run line extraction
    LineExtractor extractor;
    extractor.setPrecomputedCache(bearings, cos_bearings, sin_bearings, indices);
    extractor.setRangeData(ranges, xs, ys, zs);

    // Set parameters
    extractor.setBearingVariance(line_params.bearing_var);
    extractor.setRangeVariance(line_params.range_var);
    extractor.setZVariance(line_params.z_var);
    extractor.setLeastSqAngleThresh(line_params.least_sq_angle_thresh);
    extractor.setLeastSqRadiusThresh(line_params.least_sq_radius_thresh);
    extractor.setMaxLineGap(line_params.max_line_gap);
    extractor.setMinLineLength(line_params.min_line_length);
    extractor.setMinRange(line_params.min_range);
    extractor.setMaxRange(line_params.max_range);
    extractor.setMinSplitDist(line_params.min_split_dist);
    extractor.setOutlierDist(line_params.outlier_dist);
    extractor.setMinLinePoints(line_params.min_line_points);

    std::vector<Line> lines;
    extractor.extractLines(lines);

    std::cout << "Extracted " << lines.size() << " lines\n";

    if (lines.size() > 0) {
        // Save visualization
        std::string filename = projection_name + "_lines.ply";
        saveLineVisualization(lines, filename);
        std::cout << "Saved line visualization to: " << filename << std::endl;
    } else {
        std::cout << "No lines extracted - this is likely the problem!" << std::endl;
    }
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cout << "Usage: " << argv[0] << " <projection.pcd>" << std::endl;
        std::cout << "Example: " << argv[0] << " topdown_projection.pcd" << std::endl;
        return -1;
    }

    std::string projection_file = argv[1];
    std::cout << "=== Line Extraction Debug Tool ===\n";

    // Load the projection file directly
    pcl::PointCloud<pcl::PointXYZI>::Ptr projection(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(projection_file, *projection) == -1) {
        std::cerr << "Error: Could not read file " << projection_file << std::endl;
        return -1;
    }

    std::cout << "Loaded projection: " << projection->size() << " points\n";

    // Create line extraction parameters
    auto line_params = createDefaultLineParams();

    // Debug line extraction on the loaded projection
    std::string projection_name = projection_file.substr(projection_file.find_last_of("/") + 1);
    debugProjectionLineExtraction(projection, projection_name, line_params);

    return 0;
}