#define _USE_MATH_DEFINES
#include <cmath>

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <thread>
#include <vector>
#include <chrono>

//Keyence
#include "KEYENCE/Keyence.h"
// Robot
#include "CAO.h"
#include "CAO_i.c"
#include "atlbase.h"
#include <iostream>
#include "DensoRobot/Denso/Denso.h"
#include "DensoRobot/Terminal/Terminal.h"
#include "Output/OutputGenerator.h"

// *******************************************************************************************************
int main(int argc, char** argv)
{
    // sensor and robot object initialization
    Keyence profilometer(3200, 276.8);
    Generate_Error(profilometer.connect(), "Not able to connect to keyence");
    _Robot_ DENSO_HS43452M;
    DENSO_HS43452M.MotorOn();
    DENSO_HS43452M.TakeArm();

    // set parameter for acquisition
    std::vector<double> positions;
    positions.resize(8);
    std::vector<double> target; // it contains the target position of each joint (variation of angles)
    target.resize(8);
    double angle_variation = 1;
    double total_profiles = 360 / std::abs(angle_variation);
    target[3] = angle_variation;

    // for more acquisition at different distances
    std::vector<double> move_down;
    move_down.resize(8);
    move_down[2] = -0;

    std::vector<ProfilePoint> profile_data;
    for (int num_scan = 0; num_scan <= 0; num_scan++) {
        _OutputGenerator_ OutputGenerator(angle_variation, total_profiles, Gocator.getProfilePointCount(), num_scan);

        for (double angle = 0; std::abs(angle) < std::abs(360); angle += angle_variation) {
            std::cout << "\nAngle at plane " << num_scan << ": " << angle << std::endl;
            // Acquire a profile
            Generate_Error(profilometer.acquire(profile_data), "Not able to connect to keyence");
            // Acquire encoders
            positions = DENSO_HS43452M.GetCurrentJointPositions();
            // Write profile on file
            OutputGenerator.AddLine(profile_data, positions[3], Eigen::MatrixXd::Identity(4, 4), true);
            DENSO_HS43452M.MoveJoint(target, 0.2); // move
            std::this_thread::sleep_for(std::chrono::milliseconds(250)); // wait
        }

        OutputGenerator.CloseFile();
        DENSO_HS43452M.MoveJoint(move_down, 0.1); // move
        std::this_thread::sleep_for(std::chrono::milliseconds(60)); // wait
        angle_variation = -angle_variation;

        target[3] = angle_variation;
    }


    // Clean objects
    DENSO_HS43452M.GiveArm();
    DENSO_HS43452M.MotorOff();
    DENSO_HS43452M.CleanUp();

    return 0;
}
