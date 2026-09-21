#pragma once

#include <string>
#include <vector>
#include "Parameter.h"

class AppParameters {
public:
    AppParameters(const std::string& groupName);
    void addParameter(const std::string& name, const std::string& value);
    std::string getGroupName() const;
    std::string getParameterValue(const std::string& paramName) const;

private:
    std::string groupName;
    std::vector<Parameter> parameters;
};

