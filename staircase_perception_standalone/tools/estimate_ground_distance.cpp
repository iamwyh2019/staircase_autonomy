#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <algorithm>
#include <cmath>
#include <limits>

struct PlaneInfo {
    Eigen::Vector4f coefficients;  // ax + by + cz + d = 0
    double xy_distance;             // Distance from origin in X-Y plane
    int inlier_count;
};

double estimateGroundDistance(const std::string& filename) {
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

    // Convert to XYZ for RANSAC
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& point : cloud->points) {
        cloud_xyz->push_back(pcl::PointXYZ(point.x, point.y, point.z));
    }

    // Find multiple horizontal planes using RANSAC
    std::vector<PlaneInfo> planes;
    pcl::PointCloud<pcl::PointXYZ>::Ptr remaining_cloud = cloud_xyz;
    const double distance_threshold = 0.05;  // 5cm tolerance for plane fitting
    const double horizontal_threshold = 0.85; // Normal must be mostly vertical: |z_component| > 0.85 (angle < 31.8 degrees from vertical)
    const int max_planes = 10;

    std::cout << "\nSearching for horizontal planes (parallel to ground)..." << std::endl;
    std::cout << "Constraint: plane normal Z-component must be > " << horizontal_threshold
              << " (angle from vertical < " << (acos(horizontal_threshold) * 180.0 / M_PI) << " degrees)" << std::endl;

    int rejected_count = 0;
    for (int i = 0; i < max_planes && remaining_cloud->size() > 100; ++i) {
        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model(
            new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(remaining_cloud));

        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
        ransac.setDistanceThreshold(distance_threshold);
        ransac.computeModel();

        Eigen::VectorXf coeffs;
        ransac.getModelCoefficients(coeffs);

        // Check if plane is approximately horizontal (normal close to Z-axis)
        // For a horizontal plane, the normal vector should point mostly in Z direction
        double nx = coeffs[0], ny = coeffs[1], nz = coeffs[2];
        double norm = sqrt(nx*nx + ny*ny + nz*nz);
        double nz_normalized = fabs(nz / norm);  // Normalized Z component (0 to 1)

        std::vector<int> inliers;
        ransac.getInliers(inliers);

        std::cout << "  Candidate " << i+1 << ": " << inliers.size() << " inliers, "
                  << "normal=(" << nx/norm << ", " << ny/norm << ", " << nz/norm << "), "
                  << "|nz|=" << nz_normalized;

        if (nz_normalized > horizontal_threshold) {
            std::cout << " \033[1;32m✓ ACCEPTED\033[0m" << std::endl;

            // Calculate X-Y distance from origin to plane
            // For plane ax + by + cz + d = 0, distance from point (px, py, pz) to plane is:
            // |a*px + b*py + c*pz + d| / sqrt(a^2 + b^2 + c^2)
            // For origin (0,0,0): |d| / sqrt(a^2 + b^2 + c^2)
            // But we want X-Y distance, so we project onto X-Y plane

            double a = coeffs[0], b = coeffs[1], c = coeffs[2], d = coeffs[3];

            // Find the point on the plane closest to the origin in X-Y
            // We need to find (x,y,z) on plane that minimizes x^2 + y^2
            // Subject to: ax + by + cz + d = 0
            // Setting z=0 (approximately, for X-Y distance): ax + by + d = 0
            // This gives us the X-Y distance from origin to the plane's X-Y projection

            double xy_dist;
            if (fabs(a) > 1e-6 || fabs(b) > 1e-6) {
                xy_dist = fabs(d) / sqrt(a*a + b*b);
            } else {
                xy_dist = 0.0;  // Vertical plane at origin
            }

            // Calculate vertical height from origin to plane
            // For a plane ax + by + cz + d = 0, the vertical distance from origin is:
            // At x=0, y=0: cz + d = 0, so z = -d/c
            // This gives the Z coordinate where the plane intersects the Z-axis
            double height = (fabs(c) > 1e-6) ? -d / c : NAN;

            // Find actual Z range and closest X-Y point in this plane
            double min_z = std::numeric_limits<double>::max();
            double max_z = -std::numeric_limits<double>::max();
            double min_xy_dist = std::numeric_limits<double>::max();

            for (int idx : inliers) {
                const auto& pt = remaining_cloud->points[idx];
                double z = pt.z;
                min_z = std::min(min_z, z);
                max_z = std::max(max_z, z);

                // Calculate X-Y distance from origin to this point
                double xy_dist_point = sqrt(pt.x * pt.x + pt.y * pt.y);
                min_xy_dist = std::min(min_xy_dist, xy_dist_point);
            }

            PlaneInfo plane_info;
            plane_info.coefficients = Eigen::Vector4f(a, b, c, d);
            plane_info.xy_distance = xy_dist;
            plane_info.inlier_count = inliers.size();
            planes.push_back(plane_info);

            std::cout << "         \033[1;32mHeight from sensor: " << height << "m\033[0m" << std::endl;
            std::cout << "         \033[1;32mActual Z range: [" << min_z << ", " << max_z << "]\033[0m" << std::endl;
            std::cout << "         \033[1;32mClosest X-Y distance (any point on plane): " << min_xy_dist << "m\033[0m" << std::endl;

            // Remove inliers for next iteration
            pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            for (size_t j = 0; j < remaining_cloud->size(); ++j) {
                if (std::find(inliers.begin(), inliers.end(), j) == inliers.end()) {
                    new_cloud->push_back(remaining_cloud->points[j]);
                }
            }
            remaining_cloud = new_cloud;
        } else {
            std::cout << " ✗ REJECTED (not horizontal)" << std::endl;
            rejected_count++;
            // Don't break - keep searching for horizontal planes
            // But remove these inliers so we don't keep finding the same plane
            pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            for (size_t j = 0; j < remaining_cloud->size(); ++j) {
                if (std::find(inliers.begin(), inliers.end(), j) == inliers.end()) {
                    new_cloud->push_back(remaining_cloud->points[j]);
                }
            }
            remaining_cloud = new_cloud;
        }
    }

    std::cout << "\n=== SUMMARY ===" << std::endl;
    std::cout << "Found " << planes.size() << " horizontal planes (rejected "
              << rejected_count << " non-horizontal)" << std::endl;

    if (planes.empty()) {
        std::cerr << "\nNo horizontal planes found. Cannot estimate ground height." << std::endl;
        return NAN;
    }

    std::cout << "\nSelect the plane you're standing on based on:" << std::endl;
    std::cout << "  - Number of inliers (more = larger plane)" << std::endl;
    std::cout << "  - X-Y distance (smaller = closer to sensor location)" << std::endl;
    std::cout << "  - Height (should match your sensor height above ground)" << std::endl;

    return 0.0;  // Return success, user will manually select
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <pointcloud.pcd|pointcloud.ply>" << std::endl;
        std::cout << "Example: " << argv[0] << " scan.pcd" << std::endl;
        return -1;
    }

    std::string filename = argv[1];

    double ground_distance = estimateGroundDistance(filename);

    if (std::isnan(ground_distance)) {
        std::cerr << "Failed to estimate ground distance" << std::endl;
        return -1;
    }

    return 0;
}