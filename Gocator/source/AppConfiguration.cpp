#include "AppConfiguration.h"
#include <boost/property_tree/json_parser.hpp>
#include <iostream>

// Copy constructor
AppConfiguration::AppConfiguration(const AppConfiguration& other) {
    groups = other.groups;
}

// Copy assignment operator
AppConfiguration& AppConfiguration::operator=(const AppConfiguration& other) {
    if (this != &other) {
        groups = other.groups;
    }
    return *this;
}

AppConfiguration AppConfiguration::fromJson(const std::string& filename) {
    AppConfiguration config;
    boost::property_tree::ptree pt;

    try {
        boost::property_tree::read_json(filename, pt);
    }
    catch (const boost::property_tree::json_parser_error& e) {
        std::cout << "Error reading JSON file: " << e.what() << std::endl;
        return config;
    }

    for (const auto& appParam : pt.get_child("appParameters")) {
        std::string group = appParam.second.get<std::string>("group");
        AppParameters appParameters(group);

        for (const auto& param : appParam.second.get_child("parameters")) {
            std::string name = param.second.get<std::string>("name");
            std::string value = param.second.get<std::string>("value");
            appParameters.addParameter(name, value);
        }

        config.groups.push_back(appParameters);
    }

    return config;
}

std::string AppConfiguration::getParameterValue(const std::string& groupName, const std::string& paramName) const {
    for (const auto& group : groups) {
        if (group.getGroupName() == groupName) {
            return group.getParameterValue(paramName);
        }
    }
    return ""; // Return an empty string if the group or parameter is not found
}