#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <algorithm>
#include <cmath>
#include <limits>

double estimateGroundDistance(const std::string& filename, double tolerance = 0.1) {
    std::cout << "Estimating ground distance from: " << filename << std::endl;

    // Determine file type by extension
    std::string ext = filename.substr(filename.find_last_of(".") + 1);
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    // Load file based on extension
    if (ext == "pcd") {
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(filename, *cloud) == -1) {
            std::cerr << "Error: Could not read PCD file " << filename << std::endl;
            return NAN;
        }
    } else if (ext == "ply") {
        if (pcl::io::loadPLYFile<pcl::PointXYZI>(filename, *cloud) == -1) {
            std::cerr << "Error: Could not read PLY file " << filename << std::endl;
            return NAN;
        }
    } else {
        std::cerr << "Error: Unsupported file format. Use .pcd or .ply files." << std::endl;
        return NAN;
    }

    if (cloud->empty()) {
        std::cerr << "Error: Point cloud is empty" << std::endl;
        return NAN;
    }

    std::cout << "Loaded " << cloud->size() << " points" << std::endl;

    // Find points near origin (x=0, y=0) within tolerance
    std::vector<double> z_values;
    for (const auto& point : cloud->points) {
        if (fabs(point.x) <= tolerance && fabs(point.y) <= tolerance) {
            z_values.push_back(point.z);
        }
    }

    if (z_values.empty()) {
        std::cout << "No points found near origin (x=0, y=0) within tolerance " << tolerance << "m" << std::endl;
        std::cout << "Expanding search..." << std::endl;

        // Expand search area if no points found at origin
        tolerance *= 5.0;
        for (const auto& point : cloud->points) {
            if (fabs(point.x) <= tolerance && fabs(point.y) <= tolerance) {
                z_values.push_back(point.z);
            }
        }

        if (z_values.empty()) {
            std::cout << "No points found near origin even with expanded tolerance " << tolerance << "m" << std::endl;
            std::cout << "Fallback: Using lowest Z coordinate in entire cloud..." << std::endl;

            // Find the lowest Z coordinate in the entire point cloud
            double min_z = std::numeric_limits<double>::max();
            for (const auto& point : cloud->points) {
                if (point.z < min_z) {
                    min_z = point.z;
                }
            }

            std::cout << "Estimated ground distance (fallback): " << min_z << "m" << std::endl;
            return min_z;
        }
    }

    // Calculate average z-coordinate
    double sum_z = 0.0;
    for (double z : z_values) {
        sum_z += z;
    }
    double avg_z = sum_z / z_values.size();

    std::cout << "Found " << z_values.size() << " points near origin (tolerance: " << tolerance << "m)" << std::endl;
    std::cout << "Estimated ground distance: " << avg_z << "m" << std::endl;

    return avg_z;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <pointcloud.pcd|pointcloud.ply> [tolerance]" << std::endl;
        std::cout << "Example: " << argv[0] << " scan.pcd 0.2" << std::endl;
        return -1;
    }

    std::string filename = argv[1];
    double tolerance = 0.1; // Default tolerance

    if (argc >= 3) {
        tolerance = std::atof(argv[2]);
        if (tolerance <= 0) {
            std::cerr << "Error: Tolerance must be positive" << std::endl;
            return -1;
        }
    }

    double ground_distance = estimateGroundDistance(filename, tolerance);

    if (std::isnan(ground_distance)) {
        std::cerr << "Failed to estimate ground distance" << std::endl;
        return -1;
    }

    return 0;
}