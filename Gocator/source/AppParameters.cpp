#include "AppParameters.h"

// Implementation of AppParameters class
AppParameters::AppParameters(const std::string& groupName)
    : groupName(groupName) {}

void AppParameters::addParameter(const std::string& name, const std::string& value) {
    parameters.emplace_back(name, value);
}

std::string AppParameters::getGroupName() const {
    return groupName;
}

std::string AppParameters::getParameterValue(const std::string& paramName) const {
    for (const auto& param : parameters) {
        if (param.getName() == paramName) {
            return param.getValue();
        }
    }
    return ""; // Return an empty string if the parameter is not found
}