#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cout << "Usage: " << argv[0] << " <file.pcd>" << std::endl;
        return -1;
    }

    std::string file = argv[1];
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZI>(file, *cloud) == -1) {
        std::cerr << "Error: Could not read PCD file " << file << std::endl;
        return -1;
    }

    std::cout << "=== PCD File Info: " << file << " ===\n";
    std::cout << "Points: " << cloud->size() << std::endl;
    std::cout << "Width: " << cloud->width << ", Height: " << cloud->height << std::endl;
    std::cout << "Is dense: " << (cloud->is_dense ? "true" : "false") << std::endl;

    if (cloud->empty()) {
        std::cout << "WARNING: Point cloud is EMPTY!" << std::endl;
        return 0;
    }

    // Calculate bounds
    float min_x = cloud->points[0].x, max_x = cloud->points[0].x;
    float min_y = cloud->points[0].y, max_y = cloud->points[0].y;
    float min_z = cloud->points[0].z, max_z = cloud->points[0].z;
    float min_i = cloud->points[0].intensity, max_i = cloud->points[0].intensity;

    for (const auto& point : cloud->points) {
        min_x = std::min(min_x, point.x); max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y); max_y = std::max(max_y, point.y);
        min_z = std::min(min_z, point.z); max_z = std::max(max_z, point.z);
        min_i = std::min(min_i, point.intensity); max_i = std::max(max_i, point.intensity);
    }

    std::cout << "Bounds:\n";
    std::cout << "  X: [" << min_x << ", " << max_x << "] (range: " << (max_x - min_x) << ")\n";
    std::cout << "  Y: [" << min_y << ", " << max_y << "] (range: " << (max_y - min_y) << ")\n";
    std::cout << "  Z: [" << min_z << ", " << max_z << "] (range: " << (max_z - min_z) << ")\n";
    std::cout << "  I: [" << min_i << ", " << max_i << "] (range: " << (max_i - min_i) << ")\n";

    // Show first few points
    std::cout << "\nFirst 5 points:\n";
    for (size_t i = 0; i < std::min(size_t(5), cloud->size()); ++i) {
        const auto& p = cloud->points[i];
        std::cout << "  [" << i << "] x=" << p.x << " y=" << p.y << " z=" << p.z << " i=" << p.intensity << std::endl;
    }

    return 0;
}