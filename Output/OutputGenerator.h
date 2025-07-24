#ifndef OutputGenerator_H
#define OutputGenerator_H

#include <filesystem> // for paths
#include <iostream>
#include <fstream>
#include <Eigen/Dense>

#ifndef Sensor_H
    typedef struct ProfilePoint
    {
        double x;   // x-coordinate in engineering units (mm) - position along laser line
        double z;   // z-coordinate in engineering units (mm) - height (at the given x position)
        unsigned char intensity;
    } ProfilePoint;
#endif // !Sensor_H



class _OutputGenerator_ {
    private:
        std::filesystem::path current_path_;
        std::filesystem::path output_folder_;
        std::ofstream file_;
        double sampling_, total_profile_, total_points_;

        std::string GetCurrentDateTime();
        Eigen::Vector4d TransformPoint(ProfilePoint pointStruct, Eigen::Matrix4d transformationMatrix);

    public:
        _OutputGenerator_(double encoder_resolution, double total_profile, double total_points, int num = 0);
        void AddLine(std::vector<ProfilePoint> profile, double tick, Eigen::Matrix4d transformationMatrix, bool intensity = false);
        void CloseFile();

};

#endif
