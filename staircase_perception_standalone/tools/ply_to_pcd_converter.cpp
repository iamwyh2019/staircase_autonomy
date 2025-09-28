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

    for (size_t i = 0; i < cloud_rgb->points.size(); ++i) {
        const auto& rgb_point = cloud_rgb->points[i];
        auto& xyzi_point = cloud_xyzi->points[i];

        if (apply_arkit_transform) {
            // ARKit → Robot coordinate system transformation:
            // ARKit: X=right, Y=up, Z=forward (toward camera)
            // Robot: X=forward, Y=left, Z=upward

            // Transform:
            xyzi_point.x = -rgb_point.z;  // ARKit forward (-Z) → Robot forward (+X)
            xyzi_point.y = -rgb_point.x;  // ARKit right (+X) → Robot left (+Y)
            xyzi_point.z = rgb_point.y;   // ARKit up (+Y) → Robot up (+Z)
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
        std::cout << "Applied ARKit → Robot coordinate transformation" << std::endl;
        std::cout << "  ARKit Z (forward) → Robot X (forward)" << std::endl;
        std::cout << "  ARKit X (right) → Robot Y (left)" << std::endl;
        std::cout << "  ARKit Y (up) → Robot Z (up)" << std::endl;
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