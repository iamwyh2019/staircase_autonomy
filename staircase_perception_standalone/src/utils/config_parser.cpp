#include "staircase_perception_standalone/utils/config_parser.hpp"
#include <iostream>
#include <algorithm>

namespace stair_utility {

ConfigParser::ConfigParser(const std::string& config_file) {
    parseFile(config_file);
}

void ConfigParser::parseFile(const std::string& config_file) {
    std::ifstream file(config_file);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open config file " << config_file << std::endl;
        return;
    }

    std::string line;
    std::string current_section = "";

    while (std::getline(file, line)) {
        // Remove leading/trailing whitespace
        line.erase(0, line.find_first_not_of(" \t"));
        line.erase(line.find_last_not_of(" \t") + 1);

        // Skip empty lines and comments
        if (line.empty() || line[0] == '#') {
            continue;
        }

        // Check for section headers (lines ending with :)
        if (line.back() == ':') {
            current_section = line.substr(0, line.length() - 1);
            continue;
        }

        // Parse key-value pairs
        size_t colon_pos = line.find(':');
        if (colon_pos != std::string::npos) {
            std::string key = line.substr(0, colon_pos);
            std::string value = line.substr(colon_pos + 1);

            // Remove whitespace
            key.erase(0, key.find_first_not_of(" \t"));
            key.erase(key.find_last_not_of(" \t") + 1);
            value.erase(0, value.find_first_not_of(" \t"));
            value.erase(value.find_last_not_of(" \t") + 1);

            // Remove comments from value
            size_t comment_pos = value.find('#');
            if (comment_pos != std::string::npos) {
                value = value.substr(0, comment_pos);
                value.erase(value.find_last_not_of(" \t") + 1);
            }

            // Store with section prefix
            std::string full_key = current_section.empty() ? key : current_section + "." + key;
            config_map_[full_key] = value;
        }
    }
}

double ConfigParser::getDouble(const std::string& key, double default_value) const {
    auto it = config_map_.find(key);
    if (it != config_map_.end()) {
        try {
            return std::stod(it->second);
        } catch (const std::exception& e) {
            std::cerr << "Warning: Could not parse double value for key " << key << ": " << it->second << std::endl;
        }
    }
    return default_value;
}

int ConfigParser::getInt(const std::string& key, int default_value) const {
    auto it = config_map_.find(key);
    if (it != config_map_.end()) {
        try {
            return std::stoi(it->second);
        } catch (const std::exception& e) {
            std::cerr << "Warning: Could not parse int value for key " << key << ": " << it->second << std::endl;
        }
    }
    return default_value;
}

bool ConfigParser::getBool(const std::string& key, bool default_value) const {
    auto it = config_map_.find(key);
    if (it != config_map_.end()) {
        std::string value = it->second;
        std::transform(value.begin(), value.end(), value.begin(), ::tolower);
        return (value == "true" || value == "1" || value == "yes");
    }
    return default_value;
}

std::string ConfigParser::getString(const std::string& key, const std::string& default_value) const {
    auto it = config_map_.find(key);
    if (it != config_map_.end()) {
        return it->second;
    }
    return default_value;
}

StaircaseDetectorParams ConfigParser::getDetectorParams() const {
    StaircaseDetectorParams params;

    // Point cloud parameters
    params.leaf_size = getDouble("stair_pointcloud.leaf_size", 0.025);

    // Detection parameters
    params.use_ramp_detection = getBool("stair_detector.use_ramp_detection", true);
    params.angle_resolution = getDouble("stair_detector.angle_resolution", 1.0);
    params.robot_height = getDouble("stair_detector.robot_height", 0.5);
    params.initialization_range = getDouble("stair_detector.initialization_range", 0.5);
    params.ground_height_buffer = getDouble("stair_detector.ground_height_buffer", 0.05);

    // Stair parameters
    params.min_stair_count = getInt("stair_detector.min_stair_count", 3);
    params.stair_slope_min = getDouble("stair_detector.stair_slope_min", 0.35);
    params.stair_slope_max = getDouble("stair_detector.stair_slope_max", 1.22);
    params.min_stair_width = getDouble("stair_detector.min_stair_width", 0.75);
    params.min_stair_height = getDouble("stair_detector.min_stair_height", 0.11);
    params.max_stair_height = getDouble("stair_detector.max_stair_height", 0.3);
    params.min_stair_depth = getDouble("stair_detector.min_stair_depth", 0.15);
    params.max_stair_depth = getDouble("stair_detector.max_stair_depth", 0.45);
    params.max_stair_curvature = getDouble("stair_detector.max_stair_curvature", 0.55);

    // Point cloud bounds
    params.x_max = getDouble("stair_pointcloud.max_range_x", 4.0);
    params.x_min = getDouble("stair_pointcloud.min_range_x", -1.0);
    params.y_max = getDouble("stair_pointcloud.max_range_y", 3.0);
    params.y_min = getDouble("stair_pointcloud.min_range_y", -3.0);
    params.z_max = getDouble("stair_pointcloud.max_range_z", 1.5);
    params.z_min = getDouble("stair_pointcloud.min_range_z", -2.5);

    return params;
}

LineExtractorParams ConfigParser::getLineExtractorParams() const {
    LineExtractorParams params;

    params.bearing_var = getDouble("stair_line_extractor.bearing_variance", 0.0001);
    params.range_var = getDouble("stair_line_extractor.range_variance", 0.001);
    params.z_var = getDouble("stair_line_extractor.z_variance", 0.0004);
    params.least_sq_angle_thresh = getDouble("stair_line_extractor.least_sq_angle_thresh", 0.05);
    params.least_sq_radius_thresh = getDouble("stair_line_extractor.least_sq_radius_thresh", 0.075);
    params.max_line_gap = getDouble("stair_line_extractor.max_line_gap", 0.2);
    params.min_line_length = getDouble("stair_line_extractor.min_line_length", 0.75);
    params.min_range = getDouble("stair_line_extractor.min_range", 0.1);
    params.max_range = getDouble("stair_line_extractor.max_range", 5.0);
    params.min_split_dist = getDouble("stair_line_extractor.min_split_distance", 0.2);
    params.outlier_dist = getDouble("stair_line_extractor.outlier_distance", 0.2);
    params.min_line_points = getInt("stair_line_extractor.min_line_points", 7);

    return params;
}

} // namespace stair_utility