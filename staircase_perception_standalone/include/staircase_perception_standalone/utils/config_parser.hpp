#pragma once

#include <string>
#include <fstream>
#include <sstream>
#include <unordered_map>

#include "staircase_perception_standalone/utils/stair_utilities.hpp"

namespace stair_utility {

class ConfigParser {
public:
    ConfigParser(const std::string& config_file);

    StaircaseDetectorParams getDetectorParams() const;
    LineExtractorParams getLineExtractorParams() const;

private:
    std::unordered_map<std::string, std::string> config_map_;

    void parseFile(const std::string& config_file);
    double getDouble(const std::string& key, double default_value = 0.0) const;
    int getInt(const std::string& key, int default_value = 0) const;
    bool getBool(const std::string& key, bool default_value = false) const;
    std::string getString(const std::string& key, const std::string& default_value = "") const;
};

} // namespace stair_utility