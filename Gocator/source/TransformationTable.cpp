#include "TransformationTable.h"

// Constructor definition
TransformationTable::TransformationTable(double x, double y, double z, double rotX, double rotY, double rotZ)
    : xOffset(x), yOffset(y), zOffset(z), xAngle(rotX), yAngle(rotY), zAngle(rotZ) {}

// Method definitions
double TransformationTable::getXOffset() const {
    return xOffset;
}

double TransformationTable::getYOffset() const {
    return yOffset;
}

double TransformationTable::getZOffset() const {
    return zOffset;
}

double TransformationTable::getXAngle() const {
    return xAngle;
}

double TransformationTable::getYAngle() const {
    return yAngle;
}

double TransformationTable::getZAngle() const {
    return zAngle;
}
