#pragma once
class TransformationTable
{
private:
    double xOffset;
    double yOffset;
    double zOffset;
    double xAngle;
    double yAngle;
    double zAngle;

public:
    // Constructor for initializing the variables
    TransformationTable(double x, double y, double z, double rotX, double rotY, double rotZ);

    // Method to safely read variables
    double getXOffset() const;
    double getYOffset() const;
    double getZOffset() const;
    double getXAngle() const;
    double getYAngle() const;
    double getZAngle() const;
};

