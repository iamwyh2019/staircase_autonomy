#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include "staircase_perception_standalone/core/stair_detector.hpp"
#include "staircase_perception_standalone/utils/stair_utilities.hpp"
#include "staircase_perception_standalone/utils/config_parser.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace py = pybind11;

// Helper function to load PCD file and return as numpy arrays
py::dict load_pcd_file(const std::string& filename) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZI>(filename, *cloud) == -1) {
        throw std::runtime_error("Could not read file " + filename);
    }

    size_t n_points = cloud->points.size();

    // Create numpy arrays
    py::array_t<float> xyz(std::vector<size_t>{n_points, 3});
    py::array_t<float> intensity(std::vector<size_t>{n_points});

    auto xyz_buf = xyz.mutable_unchecked<2>();
    auto intensity_buf = intensity.mutable_unchecked<1>();

    for (size_t i = 0; i < n_points; ++i) {
        xyz_buf(i, 0) = cloud->points[i].x;
        xyz_buf(i, 1) = cloud->points[i].y;
        xyz_buf(i, 2) = cloud->points[i].z;
        intensity_buf(i) = cloud->points[i].intensity;
    }

    py::dict result;
    result["xyz"] = xyz;
    result["intensity"] = intensity;
    result["size"] = n_points;

    return result;
}

// Helper function to create point cloud from numpy array
pcl::PointCloud<pcl::PointXYZI>::Ptr numpy_to_pointcloud(
    py::array_t<float> xyz,
    py::array_t<float> intensity) {

    auto xyz_buf = xyz.unchecked<2>();
    auto intensity_buf = intensity.unchecked<1>();

    if (xyz_buf.shape(1) != 3) {
        throw std::runtime_error("xyz array must have shape (N, 3)");
    }

    size_t n_points = xyz_buf.shape(0);
    if (n_points != (size_t)intensity_buf.shape(0)) {
        throw std::runtime_error("xyz and intensity arrays must have same length");
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    cloud->points.resize(n_points);

    for (size_t i = 0; i < n_points; ++i) {
        cloud->points[i].x = xyz_buf(i, 0);
        cloud->points[i].y = xyz_buf(i, 1);
        cloud->points[i].z = xyz_buf(i, 2);
        cloud->points[i].intensity = intensity_buf(i);
    }

    cloud->width = n_points;
    cloud->height = 1;
    cloud->is_dense = false;

    return cloud;
}

// Wrapper class for StairDetector that works with numpy
class StairDetectorWrapper {
public:
    StairDetectorWrapper(
        const stair_utility::StaircaseDetectorParams& detector_params,
        const stair_utility::LineExtractorParams& line_params)
        : detector_(detector_params, line_params) {}

    void set_point_cloud(py::array_t<float> xyz, py::array_t<float> intensity) {
        auto cloud = numpy_to_pointcloud(xyz, intensity);
        detector_.setPointCloudAndOdometry(cloud);
    }

    void set_point_cloud_with_height(py::array_t<float> xyz, py::array_t<float> intensity, float r_height) {
        auto cloud = numpy_to_pointcloud(xyz, intensity);
        detector_.setPointCloudAndOdometry(cloud, r_height);
    }

    py::tuple detect_staircase() {
        stair_utility::StaircaseMeasurement stair_up, stair_down;
        auto result = detector_.detectStaircase(stair_up, stair_down);
        return py::make_tuple(result, stair_up, stair_down);
    }

private:
    StairDetector detector_;
};

PYBIND11_MODULE(stair_detector_py, m) {
    m.doc() = "Python bindings for staircase detection";

    // Enums
    py::enum_<stair_utility::StaircaseDetectorResult>(m, "StaircaseDetectorResult")
        .value("NoStairsDetected", stair_utility::StaircaseDetectorResult::NoStairsDetected)
        .value("StairsDetectedUp", stair_utility::StaircaseDetectorResult::StairsDetectedUp)
        .value("StairsDetectedDown", stair_utility::StaircaseDetectorResult::StairsDetectedDown)
        .value("StairsDetectedBoth", stair_utility::StaircaseDetectorResult::StairsDetectedBoth)
        .export_values();

    // StaircaseDetectorParams
    py::class_<stair_utility::StaircaseDetectorParams>(m, "StaircaseDetectorParams")
        .def(py::init<>())
        .def_readwrite("use_ramp_detection", &stair_utility::StaircaseDetectorParams::use_ramp_detection)
        .def_readwrite("verbose", &stair_utility::StaircaseDetectorParams::verbose)
        .def_readwrite("angle_resolution", &stair_utility::StaircaseDetectorParams::angle_resolution)
        .def_readwrite("leaf_size", &stair_utility::StaircaseDetectorParams::leaf_size)
        .def_readwrite("robot_height", &stair_utility::StaircaseDetectorParams::robot_height)
        .def_readwrite("initialization_range", &stair_utility::StaircaseDetectorParams::initialization_range)
        .def_readwrite("ground_height_buffer", &stair_utility::StaircaseDetectorParams::ground_height_buffer)
        .def_readwrite("min_stair_count", &stair_utility::StaircaseDetectorParams::min_stair_count)
        .def_readwrite("stair_slope_min", &stair_utility::StaircaseDetectorParams::stair_slope_min)
        .def_readwrite("stair_slope_max", &stair_utility::StaircaseDetectorParams::stair_slope_max)
        .def_readwrite("min_stair_width", &stair_utility::StaircaseDetectorParams::min_stair_width)
        .def_readwrite("min_stair_height", &stair_utility::StaircaseDetectorParams::min_stair_height)
        .def_readwrite("max_stair_height", &stair_utility::StaircaseDetectorParams::max_stair_height)
        .def_readwrite("min_stair_depth", &stair_utility::StaircaseDetectorParams::min_stair_depth)
        .def_readwrite("max_stair_depth", &stair_utility::StaircaseDetectorParams::max_stair_depth)
        .def_readwrite("max_stair_curvature", &stair_utility::StaircaseDetectorParams::max_stair_curvature)
        .def_readwrite("x_max", &stair_utility::StaircaseDetectorParams::x_max)
        .def_readwrite("x_min", &stair_utility::StaircaseDetectorParams::x_min)
        .def_readwrite("y_max", &stair_utility::StaircaseDetectorParams::y_max)
        .def_readwrite("y_min", &stair_utility::StaircaseDetectorParams::y_min)
        .def_readwrite("z_max", &stair_utility::StaircaseDetectorParams::z_max)
        .def_readwrite("z_min", &stair_utility::StaircaseDetectorParams::z_min);

    // LineExtractorParams
    py::class_<stair_utility::LineExtractorParams>(m, "LineExtractorParams")
        .def(py::init<>())
        .def_readwrite("bearing_var", &stair_utility::LineExtractorParams::bearing_var)
        .def_readwrite("range_var", &stair_utility::LineExtractorParams::range_var)
        .def_readwrite("z_var", &stair_utility::LineExtractorParams::z_var)
        .def_readwrite("least_sq_angle_thresh", &stair_utility::LineExtractorParams::least_sq_angle_thresh)
        .def_readwrite("least_sq_radius_thresh", &stair_utility::LineExtractorParams::least_sq_radius_thresh)
        .def_readwrite("max_line_gap", &stair_utility::LineExtractorParams::max_line_gap)
        .def_readwrite("min_line_length", &stair_utility::LineExtractorParams::min_line_length)
        .def_readwrite("min_range", &stair_utility::LineExtractorParams::min_range)
        .def_readwrite("max_range", &stair_utility::LineExtractorParams::max_range)
        .def_readwrite("min_split_dist", &stair_utility::LineExtractorParams::min_split_dist)
        .def_readwrite("outlier_dist", &stair_utility::LineExtractorParams::outlier_dist)
        .def_readwrite("min_line_points", &stair_utility::LineExtractorParams::min_line_points);

    // StairStep
    py::class_<stair_utility::StairStep>(m, "StairStep")
        .def(py::init<>())
        .def_readwrite("start_p", &stair_utility::StairStep::start_p)
        .def_readwrite("end_p", &stair_utility::StairStep::end_p)
        .def_readwrite("line_polar_form", &stair_utility::StairStep::line_polar_form)
        .def_readwrite("step_covariance", &stair_utility::StairStep::step_covariance)
        .def_readwrite("step_width", &stair_utility::StairStep::step_width);

    // StaircaseMeasurement
    py::class_<stair_utility::StaircaseMeasurement>(m, "StaircaseMeasurement")
        .def(py::init<>())
        .def_readwrite("stair_count", &stair_utility::StaircaseMeasurement::stair_count)
        .def_readwrite("steps", &stair_utility::StaircaseMeasurement::steps);

    // ConfigParser
    py::class_<stair_utility::ConfigParser>(m, "ConfigParser")
        .def(py::init<const std::string&>())
        .def("get_detector_params", &stair_utility::ConfigParser::getDetectorParams)
        .def("get_line_extractor_params", &stair_utility::ConfigParser::getLineExtractorParams);

    // StairDetectorWrapper
    py::class_<StairDetectorWrapper>(m, "StairDetector")
        .def(py::init<const stair_utility::StaircaseDetectorParams&,
                      const stair_utility::LineExtractorParams&>())
        .def("set_point_cloud", &StairDetectorWrapper::set_point_cloud)
        .def("set_point_cloud_with_height", &StairDetectorWrapper::set_point_cloud_with_height)
        .def("detect_staircase", &StairDetectorWrapper::detect_staircase);

    // Utility functions
    m.def("load_pcd_file", &load_pcd_file, "Load a PCD file and return as numpy arrays");
}
