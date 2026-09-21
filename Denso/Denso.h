#ifndef Denso_H
#define Denso_H
#include "CAO.h"
#include "atlbase.h"
#include <iostream>
#include <vector>
#include "../Terminal/Terminal.h"
#include <Eigen/Dense>

class _Robot_{
	private:
		ICaoEngine* pEngine_ = NULL;
		ICaoWorkspaces* pWorkspaces_ = NULL;
		ICaoWorkspace* pWorkspace_ = NULL;
		ICaoController* pController_ = NULL;
		ICaoRobot* pRobot_ = NULL;

		void CheckErrors(HRESULT hr, std::string message);

	public:
		_Robot_();
		void CleanUp();
		// Function to execute "CurJnt" and retrieve joint positions
		std::vector<double> GetCurrentJointPositions();
		std::vector<double> GetCurrentCartesianPositions();
		void MoveJoint(const std::vector<double>& targetPositions, float speed);
		void MoveToCartesian(const std::vector<double>& targetPositions, float speed);
		void MoveAlongDirection(const std::vector<double>& direction, float speed, double total_movement);
		void MotorOff();
		void MotorOn();
		void TakeArm();
		void GiveArm();
};

#endif 