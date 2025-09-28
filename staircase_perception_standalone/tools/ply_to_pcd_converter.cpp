#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <string>

int main(int argc, char** argv) {
    if (argc < 3 || argc > 4) {
        std::cout << "Usage: " << argv[0] << " <input.ply> <output.pcd> [--arkit-transform]" << std::endl;
        std::cout << "  --arkit-transform: Apply coordinate system transformation for ARKit data" << std::endl;
        std::cout << "                     (negates Z, adjusts for robot-relative coordinates)" << std::endl;
        return -1;
    }

    std::string input_file = argv[1];
    std::string output_file = argv[2];
    bool apply_arkit_transform = false;

    if (argc == 4 && std::string(argv[3]) == "--arkit-transform") {
        apply_arkit_transform = true;
        std::cout << "ARKit coordinate transformation will be applied" << std::endl;
    }

    // Load PLY file with RGB data
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(input_file, *cloud_rgb) == -1) {
        std::cerr << "Error: Could not read PLY file " << input_file << std::endl;
        return -1;
    }

    std::cout << "Loaded " << cloud_rgb->size() << " points from " << input_file << std::endl;

    // Convert to XYZI format for stair detection
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyzi(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_xyzi->width = cloud_rgb->width;
    cloud_xyzi->height = cloud_rgb->height;
    cloud_xyzi->is_dense = cloud_rgb->is_dense;
    cloud_xyzi->points.resize(cloud_rgb->points.size());

    // Convert RGB to intensity and apply coordinate transformations
    float min_z = std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();

    // First pass: find z bounds for transformation
    if (apply_arkit_transform) {
        for (const auto& point : cloud_rgb->points) {
            min_z = std::min(min_z, point.z);
            max_z = std::max(max_z, point.z);
        }
        std::cout << "Original ARKit z range: [" << min_z << ", " << max_z << "]" << std::endl;
    }

    for (size_t i = 0; i < cloud_rgb->points.size(); ++i) {
        const auto& rgb_point = cloud_rgb->points[i];
        auto& xyzi_point = cloud_xyzi->points[i];

        if (apply_arkit_transform) {
            // ARKit transformation for stair detection:
            // 1. Your code uses -z (camera looking down at stairs)
            // 2. Need to transform to robot-centric coordinates
            // 3. Set ground level (max_z) to z=0, robot at z=0.3

            xyzi_point.x = rgb_point.x;
            xyzi_point.y = rgb_point.y;
            xyzi_point.z = -rgb_point.z + max_z;  // Negate Z and shift so max becomes 0
        } else {
            // No transformation
            xyzi_point.x = rgb_point.x;
            xyzi_point.y = rgb_point.y;
            xyzi_point.z = rgb_point.z;
        }

        // Convert RGB to intensity (0-255 range)
        xyzi_point.intensity = 0.299f * rgb_point.r + 0.587f * rgb_point.g + 0.114f * rgb_point.b;
    }

    if (apply_arkit_transform) {
        // Find new z bounds after transformation
        float new_min_z = std::numeric_limits<float>::max();
        float new_max_z = -std::numeric_limits<float>::max();
        for (const auto& point : cloud_xyzi->points) {
            new_min_z = std::min(new_min_z, point.z);
            new_max_z = std::max(new_max_z, point.z);
        }
        std::cout << "Transformed z range: [" << new_min_z << ", " << new_max_z << "]" << std::endl;
        std::cout << "Ground level (z=0) should be at highest points" << std::endl;
        std::cout << "Robot reference at z=0.3 (above ground)" << std::endl;
    }

    // Save as PCD
    if (pcl::io::savePCDFileASCII(output_file, *cloud_xyzi) == -1) {
        std::cerr << "Error: Could not save PCD file " << output_file << std::endl;
        return -1;
    }

    std::cout << "Converted and saved " << cloud_xyzi->size() << " points to " << output_file << std::endl;
    std::cout << "Intensity range: [0-255] computed from RGB values" << std::endl;

    return 0;
}