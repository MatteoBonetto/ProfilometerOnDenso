#include "Parameter.h"

Parameter::Parameter(const std::string& name, const std::string& value)
    : name(name), value(value) {}

std::string Parameter::getName() const {
    return name;
}

std::string Parameter::getValue() const {
    return value;
}
