#pragma once
#include <string>

class SensorInformation {
private:
    double minResolutionX;
    double maxResolutionX;
    double resolutionZ;
    double linearityZ;
    double nearFieldOfView;
    double farFieldOfView;
    double clearenceDistance;
    double measurementRange;

public:
    SensorInformation(double minResX, double maxResX, double resZ, double linZ, double nearFoV, double farFoV, double clearanceDist, double measRange);
    SensorInformation(std::string s);

    double getMinResolutionX() const;
    double getMaxResolutionX() const;
    double getResolutionZ() const;
    double getLinearityZ() const;
    double getNearFieldOfView() const;
    double getFarFieldOfView() const;
    double getClearenceDistance() const;
    double getMeasurementRange() const;
};

