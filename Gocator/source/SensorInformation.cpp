#include "SensorInformation.h"

SensorInformation::SensorInformation(double minResX, double maxResX, double resZ, double linZ, double nearFoV, double farFoV, double clearanceDist, double measRange)
    : minResolutionX(minResX), maxResolutionX(maxResX), resolutionZ(resZ), linearityZ(linZ), nearFieldOfView(nearFoV), farFieldOfView(farFoV), clearenceDistance(clearanceDist), measurementRange(measRange) {}

SensorInformation::SensorInformation(std::string s) {
    if (s == "2430") {
        minResolutionX = 37;
        maxResolutionX = 57;
        resolutionZ = 0.00;
        linearityZ = 0.01;
        nearFieldOfView = 47;
        farFieldOfView = 85;
        clearenceDistance = 75;
        measurementRange = 80;
    }
    else if (s == "2350") {
        minResolutionX = 150;
        maxResolutionX = 150;
        resolutionZ = 19;
        linearityZ = 0.00;
        nearFieldOfView = 158;
        farFieldOfView = 365;
        clearenceDistance = 380;
        measurementRange = 400;
    }
    else if (s == "2340") {
        minResolutionX = 95;
        maxResolutionX = 170;
        resolutionZ = 13;
        linearityZ = 0.01;
        nearFieldOfView = 96;
        farFieldOfView = 194;
        clearenceDistance = 190;
        measurementRange = 210;
    }
    else if (s == "2650") {
        minResolutionX = 46;
        maxResolutionX = 104;
        resolutionZ = 0.00;
        linearityZ = 0.00;
        nearFieldOfView = 190;
        farFieldOfView = 430;
        clearenceDistance = 330;
        measurementRange = 475;
    }
    else {
        minResolutionX = 0;
        maxResolutionX = 0;
        resolutionZ = 0;
        linearityZ = 0;
        nearFieldOfView = 0;
        farFieldOfView = 0;
        clearenceDistance = 0;
        measurementRange = 0;
    }
}

double SensorInformation::getMinResolutionX() const {
    return minResolutionX;
}

double SensorInformation::getMaxResolutionX() const {
    return maxResolutionX;
}

double SensorInformation::getResolutionZ() const {
    return resolutionZ;
}

double SensorInformation::getLinearityZ() const {
    return linearityZ;
}

double SensorInformation::getNearFieldOfView() const {
    return nearFieldOfView;
}

double SensorInformation::getFarFieldOfView() const {
    return farFieldOfView;
}

double SensorInformation::getClearenceDistance() const {
    return clearenceDistance;
}

double SensorInformation::getMeasurementRange() const {
    return measurementRange;
}

