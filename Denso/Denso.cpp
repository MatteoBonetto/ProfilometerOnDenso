#include "Denso.h"

//*******************************************************************************************************************************************
// Constructor
_Robot_::_Robot_() {
    CoInitialize(0);  // Initialize COM library

    // Create CAO Engine object
    HRESULT hr = CoCreateInstance(CLSID_CaoEngine, NULL, CLSCTX_LOCAL_SERVER, IID_ICaoEngine, (void**)&pEngine_);
    CheckErrors(hr, "Engine");

    // Connect to Workspace
    hr = pEngine_->get_Workspaces(&pWorkspaces_);
    CheckErrors(hr, "Workspaces");

    // Retrieve CaoWorkspace
    hr = pWorkspaces_->Item(CComVariant(0L), &pWorkspace_);
    CheckErrors(hr, "Workspace");

    // Establish connection with the controller, specifying the IP address
    hr = pWorkspace_->AddController(
        CComBSTR("rc8"),
        CComBSTR("CaoProv.DENSO.RC8"),
        CComBSTR(""),
        CComBSTR("Server=192.168.2.51"),
        &pController_
    );
    CheckErrors(hr, "Controller");

    // Proceed with encoder reading and joint control...
    hr = pController_->AddRobot(CComBSTR("Arm1"), CComBSTR(""), &pRobot_);
    CheckErrors(hr, "Robot");
}

//*******************************************************************************************************************************************
void _Robot_::CleanUp() {
    // Cleanup and release resources
    if (pRobot_) pRobot_->Release();
    if (pController_) pController_->Release();
    if (pWorkspace_) pWorkspace_->Release();
    if (pWorkspaces_) pWorkspaces_->Release();
    if (pEngine_) pEngine_->Release();
    CoUninitialize();
}

//*******************************************************************************************************************************************
std::vector<double> _Robot_::GetCurrentJointPositions() {
    std::vector<double> joints;

    // Initialize the variant to store the return value
    VARIANT jointPositions;
    VariantInit(&jointPositions);
    VARIANT vntParam;
    VariantInit(&vntParam); // Sets vntParam to VT_EMPTY


    // Execute "CurJnt" command to get the joint positions
    HRESULT hr = pRobot_->Execute(CComBSTR("CurJnt"), vntParam, &jointPositions);
    if (FAILED(hr)) {
        CleanUp();
        Generate_Error(FAILED(hr), "Failed to read joints");
    }

    // Check that the result is a SAFEARRAY of floats or doubles
    if (jointPositions.vt == (VT_ARRAY | VT_R8)) {  // VT_R8 for double, VT_R4 for float
        SAFEARRAY* pSafeArray = jointPositions.parray;
        double* pData = nullptr;  // Pointer to access SAFEARRAY data

        // Access the data in the SAFEARRAY
        SafeArrayAccessData(pSafeArray, (void**)&pData);

        // Get array bounds to determine the number of elements
        LONG lowerBound, upperBound;
        SafeArrayGetLBound(pSafeArray, 1, &lowerBound);
        SafeArrayGetUBound(pSafeArray, 1, &upperBound);
        LONG elementCount = upperBound - lowerBound + 1;
        joints.resize(elementCount);

        // Print each joint position
        for (LONG i = 0; i < elementCount; ++i) {
            joints[i] = pData[i];
        }

        // Release the data access
        SafeArrayUnaccessData(pSafeArray);
    }
    else {
        CleanUp();
        Generate_Error(1, "Unexpected data type returned from Execute('CurJnt').");
    }

    // Clear the variant to release memory
    VariantClear(&jointPositions);
    return joints;
}

//*******************************************************************************************************************************************
std::vector<double> _Robot_::GetCurrentCartesianPositions() {
    std::vector<double> cartesian;

    // Initialize the variant to store the return value
    VARIANT cartesianPositions;
    VariantInit(&cartesianPositions);
    VARIANT vntParam;
    VariantInit(&vntParam); // Sets vntParam to VT_EMPTY


    // Execute "CurJnt" command to get the joint positions
    HRESULT hr = pRobot_->Execute(CComBSTR(L"CurPos"), vntParam, &cartesianPositions);
    if (FAILED(hr)) {
        CleanUp();
        Generate_Error(FAILED(hr), "Failed to read joints");
    }

    // Check that the result is a SAFEARRAY of floats or doubles
    if (cartesianPositions.vt == (VT_ARRAY | VT_R8)) {  // VT_R8 for double, VT_R4 for float
        SAFEARRAY* pSafeArray = cartesianPositions.parray;
        double* pData = nullptr;  // Pointer to access SAFEARRAY data

        // Access the data in the SAFEARRAY
        SafeArrayAccessData(pSafeArray, (void**)&pData);

        // Get array bounds to determine the number of elements
        LONG lowerBound, upperBound;
        SafeArrayGetLBound(pSafeArray, 1, &lowerBound);
        SafeArrayGetUBound(pSafeArray, 1, &upperBound);
        LONG elementCount = upperBound - lowerBound + 1;
        cartesian.resize(elementCount);

        // Print each joint position
        for (LONG i = 0; i < elementCount; ++i) {
            cartesian[i] = pData[i];
        }

        // Release the data access
        SafeArrayUnaccessData(pSafeArray);
    }
    else {
        CleanUp();
        Generate_Error(1, "Unexpected data type returned from Execute('CurJnt').");
    }

    // Clear the variant to release memory
    VariantClear(&cartesianPositions);
    return cartesian;
}

//*******************************************************************************************************************************************
void _Robot_::CheckErrors(HRESULT hr, std::string message) {
    std::string error_message = message + std::string(" not generated!");
    std::string correct_message = message + std::string(" generated correctly!");
    if (FAILED(hr)) {
        CleanUp();
        Generate_Error(FAILED(hr), error_message);
    }
    else {
        SetColor(GREEN);
        std::cout << correct_message << std::endl;
        SetColor(WHITE);
    }
}

//*******************************************************************************************************************************************
void _Robot_::MoveJoint(const std::vector<double>& targetPositions, float speed) {

    // Construct POSEDATA string for joint movements
    std::wstring poseData = L"@0 ";
    for (int i = 0; i < targetPositions.size(); i++) {
        poseData += L"(" + std::to_wstring(i+1) + L", " + std::to_wstring(targetPositions[i]) + L"), ";
    }
    poseData.pop_back();
    poseData.pop_back();  // Remove the last comma and space
    //std::wcout << poseData << std::endl;
    // Set speed in the options
    std::wstring options = L"S=" + std::to_wstring(speed) + L", NEXT";
    //std::wcout << options << std::endl;

    // Convert poseData and options to CComBSTR for Execute method
    CComBSTR vntPoses(poseData.c_str());
    CComBSTR strOpt(options.c_str());

    // Execute DriveEx with poseData and speed options
    VARIANT argsDriveEx[2];
    argsDriveEx[0].vt = VT_BSTR;
    argsDriveEx[0].bstrVal = vntPoses;
    argsDriveEx[1].vt = VT_BSTR;
    argsDriveEx[1].bstrVal = strOpt;

    HRESULT hr = pRobot_->Execute(CComBSTR("DriveEx"), argsDriveEx[0], &argsDriveEx[1]);
    if (FAILED(hr)) {
        CleanUp();
        Generate_Error(hr, "Failed to execute DriveEx for relative joint movement.");
    }
}

//*******************************************************************************************************************************************
void _Robot_::MoveAlongDirection(const std::vector<double>& direction, float speed, double total_movement) {
    double L1 = 260, L2 = 220, current_movement = 0;
    double infinitesim = 1;

    while (current_movement < total_movement) {
        std::vector<double> joints = GetCurrentJointPositions();
        double q1 = joints[0] * 3.1415 / 180, q2 = joints[1] * 3.1415 / 180, q3 = joints[3] * 3.1415 / 180;

        // Create the 2x2 Jacobian matrix
        Eigen::Matrix3d J;
        J(0, 0) = -L1 * std::sin(q1) - L2 * std::sin(q1 + q2);
        J(0, 1) = -L2 * std::sin(q1 + q2);
        J(0, 2) = 0;
        J(1, 0) = L1 * std::cos(q1) + L2 * std::cos(q1 + q2);
        J(1, 1) = L2 * std::cos(q1 + q2);
        J(1, 2) = 0;
        J(2, 0) = 1;
        J(2, 1) = 1;
        J(2, 2) = 1;

        // Compute the determinant to check for singularity
        double det = - L1 * L2 * std::cos(q2 + q1) * std::sin(q1) + L1 * L2 * std::cos(q1) * std::sin(q1 + q2);
        if (std::fabs(det) < 1e-6) {
            CleanUp();
            Generate_Error(std::fabs(det) > 1e-6, "Jacobian is singular (determinant is close to zero)!");
        }

        // Compute the inverse of the Jacobian using Eigen's .inverse() method
        Eigen::Matrix3d J_inv = J.inverse();
        Eigen::Vector3d dir;
        // Initialize the vector with values
        dir << direction[0], direction[1], direction[2];
        Eigen::Vector3d new_joint_increments = J_inv * dir;
        std::vector<double> target;
        target.resize(8);
        target[0] = new_joint_increments[0] * infinitesim;
        target[1] = new_joint_increments[1] * infinitesim;
        target[3] = new_joint_increments[2] * infinitesim;
        MoveJoint(target, speed);

        double norm = std::sqrt(target[0] * target[0] + target[1] * target[1]);
        current_movement += norm;
        //std::cout << "current_movement:\t" << current_movement << std::endl;
    }

}


//*******************************************************************************************************************************************
void _Robot_::MoveToCartesian(const std::vector<double>& targetPositions, float speed) {

#if 0
    std::vector<double> position = GetCurrentCartesianPositions();

    double x = position[0];
    x += 10.0;
    position[0] = x;

    std::wstring movementMode = L"2";

    // Move robot to new position
    std::wstring moveCommand = L"@0 P(" + std::to_wstring(position[0]) + L", " +
        std::to_wstring(position[1]) + L", " +
        std::to_wstring(position[2]) + L", " +
        std::to_wstring(position[3]) + L", " +
        std::to_wstring(position[4]) + L", " +
        std::to_wstring(position[5]) + L", " +
        std::to_wstring(position[6]) + L")";

    std::wstring options = L"S=" + std::to_wstring(speed) + L", NEXT";

    // Convert poseData and options to CComBSTR for Execute method
    CComBSTR iComp(movementMode.c_str());
    CComBSTR vntPoses(moveCommand.c_str());
    CComBSTR strOpt(options.c_str());

    VARIANT argsDriveEx;
    argsDriveEx.vt = VT_BSTR;
    argsDriveEx.bstrVal = vntPoses;

    HRESULT hr = pRobot_->Move(2, argsDriveEx, strOpt);

    if (FAILED(hr)) {
        CleanUp();
        Generate_Error(hr, "Failed to execute DriveEx for relative coordinates movement.");
    }
#endif

#if 1
    std::vector<double> position = GetCurrentCartesianPositions();
    double x = position[0];
    double y = position[1];
    double z = position[2];
    double rotx = position[3];
    double roty = position[4];
    double rotz = position[5];

    x += 10.0;
    position[0] = x;
    /*position[1] = 0.0;
    position[2] = 0.0;
    position[3] = 0.0;
    position[4] = 0.0;
    position[5] = 0.0;*/
    
    // Construct POSEDATA string for joint movements
    /*std::wstring poseData = L"@1 ";
    for (int i = 0; i < position.size()-4; i++) {
        poseData += L"(" + std::to_wstring(i + 1) + L", " + std::to_wstring(position[i]) + L"), ";
    }*/

    std::wstring movementMode = L"2";
    std::wstring poseData = L"P(" + std::to_wstring(position[0]) + L", " + std::to_wstring(position[1]) + L", " + std::to_wstring(position[2])
        + L", " + std::to_wstring(position[3]) + L", " + std::to_wstring(position[4]) + L", " + std::to_wstring(position[5]) + L", " + std::to_wstring(position[6]) + L")";
    //std::wstring poseData = L"(" + std::to_wstring(position[0]) + L", " + std::to_wstring(position[1]) + L", " + std::to_wstring(position[2]) + L")";

    std::wstring varE = L"@E 56.8";

    //std::wcout << poseData << std::endl;
    // Set speed in the options
    std::wstring options = L"S=" + std::to_wstring(speed) + L", NEXT";
    //std::wcout << options << std::endl;

    // Convert poseData and options to CComBSTR for Execute method
    CComBSTR iComp(movementMode.c_str());
    CComBSTR vntPoses(poseData.c_str());
    CComBSTR e(varE.c_str());
    CComBSTR strOpt(options.c_str());

    // Execute DriveEx with poseData and speed options
    VARIANT argsDriveEx[4];
    argsDriveEx[0].vt = VT_BSTR;
    argsDriveEx[0].bstrVal = iComp;
    argsDriveEx[1].vt = VT_BSTR;
    argsDriveEx[1].bstrVal = vntPoses;
    argsDriveEx[2].vt = VT_BSTR;
    argsDriveEx[2].bstrVal = e;
    argsDriveEx[3].vt = VT_BSTR;
    argsDriveEx[3].bstrVal = strOpt;
    HRESULT hr = pRobot_->Execute(CComBSTR("Approach"),  argsDriveEx[0], &argsDriveEx[1]);
    if (FAILED(hr)) {
        CleanUp();
        Generate_Error(hr, "Failed to execute DriveEx for relative coordinates movement.");
    }
#endif
}

//*******************************************************************************************************************************************
void _Robot_::MotorOff() {
    VARIANT place_holder;
    VariantInit(&place_holder); // Sets to VT_EMPTY
    // Turn off the motors
    HRESULT hr = pRobot_->Execute(CComBSTR("Motor"), CComVariant(L"0"), &place_holder);  // "0" to turn on motors
    if (FAILED(hr)) {
        CleanUp();
        Generate_Error(hr, "Failed to turn off the motors.");
    }
}

//*******************************************************************************************************************************************
void _Robot_::MotorOn() {
    VARIANT place_holder;
    VariantInit(&place_holder); // Sets to VT_EMPTY
    // Turn off the motors
    HRESULT hr = pRobot_->Execute(CComBSTR("Motor"), CComVariant(L"1"), &place_holder);  // "1" to turn on motors
    if (FAILED(hr)) {
        CleanUp();
        Generate_Error(hr, "Failed to turn on the motors.");
    }
}

//*******************************************************************************************************************************************
void _Robot_::TakeArm() {
    VARIANT place_holder;
    VariantInit(&place_holder); // Sets to VT_EMPTY
    // Turn off the motors
    HRESULT hr = pRobot_->Execute(CComBSTR("TakeArm"), place_holder, &place_holder);  // "0" to turn on motors
    if (FAILED(hr)) {
        CleanUp();
        Generate_Error(hr, "Failed to Take arm");
    }
}

//*******************************************************************************************************************************************
void _Robot_::GiveArm() {
    VARIANT place_holder;
    VariantInit(&place_holder); // Sets to VT_EMPTY
    // Turn off the motors
    HRESULT hr = pRobot_->Execute(CComBSTR("Motor"), place_holder, &place_holder);  // "0" to turn on motors
    if (FAILED(hr)) {
        CleanUp();
        Generate_Error(hr, "Failed to turn off the motors.");
    }
}

