#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cout << "Usage: " << argv[0] << " <input.pcd> <output.ply>" << std::endl;
        return -1;
    }

    std::string input_file = argv[1];
    std::string output_file = argv[2];

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZI>(input_file, *cloud) == -1) {
        std::cerr << "Error: Could not read PCD file " << input_file << std::endl;
        return -1;
    }

    std::cout << "Loaded " << cloud->size() << " points from " << input_file << std::endl;

    if (cloud->empty()) {
        std::cout << "WARNING: Point cloud is empty!" << std::endl;
    }

    // Convert XYZI to RGB for better visualization
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_rgb->width = cloud->width;
    cloud_rgb->height = cloud->height;
    cloud_rgb->is_dense = cloud->is_dense;
    cloud_rgb->points.resize(cloud->points.size());

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        const auto& xyzi_point = cloud->points[i];
        auto& rgb_point = cloud_rgb->points[i];

        rgb_point.x = xyzi_point.x;
        rgb_point.y = xyzi_point.y;
        rgb_point.z = xyzi_point.z;

        // Convert intensity to grayscale RGB
        uint8_t gray = static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, xyzi_point.intensity)));
        rgb_point.r = gray;
        rgb_point.g = gray;
        rgb_point.b = gray;
    }

    if (pcl::io::savePLYFileASCII(output_file, *cloud_rgb) == -1) {
        std::cerr << "Error: Could not save PLY file " << output_file << std::endl;
        return -1;
    }

    std::cout << "Converted and saved " << cloud_rgb->size() << " points to " << output_file << std::endl;
    return 0;
}