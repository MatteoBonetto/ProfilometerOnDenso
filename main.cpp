#define _USE_MATH_DEFINES
#include <cmath>

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <thread>
#include <vector>
#include <chrono>

// User defined class
// Gocator
#include "AppConfiguration.h"
#define Sensor_H
#include "Sensor.h"
// Robot
#include "CAO.h"
#include "CAO_i.c"
#include "atlbase.h"
#include <iostream>
#include "Denso/Denso.h"
#include "Output/OutputGenerator.h"

// *******************************************************************************************************
int main(int argc, char** argv)
{
    //std::string fileConfigPath;
    //std::string fileCalibrationPath;
    //
    //// Check if at least one argument is provided
    //if (argc > 1) {
    //    // If an argument is provided, use it as the config filename
    //    fileConfigPath = argv[1];
    //}
    //else {
    //    // If no argument is provided, use the default config filename
    //    std::cout << "Warning! No argument provided, default config file will be used: config\\config.json" << std::endl << std::endl;
    //    fileConfigPath = "config\\config.json";
    //}
    //
    //// Check if the second argument is provided
    //if (argc > 2) {
    //    // If the second argument is provided, use it as the calibration matrix path
    //    fileCalibrationPath = argv[2];
    //}
    //else {
    //    // If no second argument is provided, use the default value for the calibration matrix path
    //    std::cout << "Warning! No second argument provided, default calibration file will be used: config\\CalibrationMatrix.txt" << std::endl << std::endl;
    //    fileCalibrationPath = "config\\CalibrationMatrix.txt";
    //}

    // Load configuration parameters
    //AppConfiguration config = AppConfiguration::fromJson(fileConfigPath);
    //
    //// sensor and robot object initialization
    //Sensor Gocator(config);

    _Robot_ DENSO_HS43452M;
    DENSO_HS43452M.MotorOn();
    DENSO_HS43452M.TakeArm();
    //Gocator.connect();
    //Gocator.start();
    //Gocator.SetTriggerSoftware();
    
    // set parameter for acquisition
    std::vector<double> positions;
    positions.resize(8);
    std::vector<double> target; // it contains the target position of each joint (variation of angles)
    target.resize(8);
    double angle_variation = 1;
    double total_profiles = 360 /std::abs( angle_variation);
    target[3] = angle_variation;
    //Eigen::Matrix4d matrix;
    //Gocator.getMatrixFromPointToSensorInverse(matrix);
    std::vector<double> move_down;
    move_down.resize(8);
    move_down[2] = -0;

    for (int num_scan = 0; num_scan <= 0; num_scan++) {
        //_OutputGenerator_ OutputGenerator(angle_variation, total_profiles, Gocator.getProfilePointCount(), num_scan);

        for (double angle = 0; std::abs(angle) < std::abs(360); angle += angle_variation) {
            std::cout << "\nAngle at plane " << num_scan << ": " << angle << std::endl;
            //GoSensor_Trigger(Gocator.getSensor()); // Sensor software trigger to acquire
            //Frame frame_test = Gocator.receiveData(); // Output profile acquired
            positions = DENSO_HS43452M.GetCurrentJointPositions();
            //OutputGenerator.AddLine(frame_test.profileData, positions[3], matrix, true); // Write profile on file
            DENSO_HS43452M.MoveJoint(target, 0.2); // move
            std::this_thread::sleep_for(std::chrono::milliseconds(250)); // wait
        }

        //OutputGenerator.CloseFile();
        //DENSO_HS43452M.MoveJoint(move_down, 0.1); // move
        std::this_thread::sleep_for(std::chrono::milliseconds(60)); // wait
        angle_variation = -angle_variation;

        target[3] = angle_variation;
    }


    // Clean objects
    DENSO_HS43452M.GiveArm();
    DENSO_HS43452M.MotorOff();
    DENSO_HS43452M.CleanUp();
    //Gocator.stop();
    //Gocator.disconnect();

    return 0;
}
