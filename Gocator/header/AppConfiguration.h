#pragma once

#include <vector>
#include <string>
#include "AppParameters.h"
#include <boost/property_tree/ptree.hpp>

class AppConfiguration {
public:
    AppConfiguration() = default;
    AppConfiguration(const AppConfiguration& other);
    AppConfiguration& operator=(const AppConfiguration& other);
    AppConfiguration(AppConfiguration&& other) noexcept = default;
    AppConfiguration& operator=(AppConfiguration&& other) noexcept = default;
    ~AppConfiguration() = default;

    static AppConfiguration fromJson(const std::string& filename);
    std::string getParameterValue(const std::string& groupName, const std::string& paramName) const;

private:
    std::vector<AppParameters> groups;
};

