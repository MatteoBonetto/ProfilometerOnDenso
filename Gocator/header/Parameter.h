#pragma once

#include <string>

class Parameter {
public:
    Parameter(const std::string& name, const std::string& value);
    std::string getName() const;
    std::string getValue() const;

private:
    std::string name;
    std::string value;
};

