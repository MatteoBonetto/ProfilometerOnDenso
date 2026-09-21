#include "Sensor.h"
#define LEFT_HAND_SYSTEM

Sensor::Sensor(const AppConfiguration& config) : config_(config) {
    api_ = kNULL;
    setup_ = kNULL;
    profileBuffer_ = NULL;
    dataset_ = kNULL;
    stamp_ = kNULL;
    profilePointCount_ = 0;
    system_ = kNULL;
    sensor_ = kNULL;
    dataObj_ = kNULL;
    sensorSerialNumber_ = -9999;
    transform_ = kNULL;

    // Parse IP address into address data structure
    kIpAddress_Parse(&this->ipAddress_, config.getParameterValue("Sensor", "SensorIP").c_str());

    // Load sensor serial number from config file
    id_ = (k32u)std::stoi(config.getParameterValue("Sensor", "SensorID"));

}

Sensor::~Sensor() {
    // Cleanup
    if (system_ != kNULL)
    {
        GoDestroy(system_);
        system_ = kNULL;
    }

    if (api_ != kNULL)
    {
        GoDestroy(api_);
        api_ = kNULL;
    }

    if (profileBuffer_ != NULL)
    {
        delete[] profileBuffer_;
        profileBuffer_ = NULL;
    }
}

bool Sensor::connect() {
    kStatus status;

    bool connectionSucceded = false;

    try
    {
        if (api_ == kNULL)
        {
            // Construct Gocator API Library
            if ((status = GoSdk_Construct(&api_)) != kOK) {
                printf("Error: GoSdk_Construct:%d\n", status);
                return false;
            }
        }
        

        if (system_ == kNULL)
        {
            // Construct GoSystem object
            if ((status = GoSystem_Construct(&system_, kNULL)) != kOK) {
                printf("Error: GoSystem_Construct:%d\n", status);
            }
        }

        if (profileBuffer_ != kNULL)
        {
            delete[] profileBuffer_;
            profileBuffer_ = kNULL;
        }

        if (std::stoi(config_.getParameterValue("Sensor", "ConnectUsingSensorIP")) > 0)
        {
            // Obtain GoSensor object by sensor IP address
            if ((status = GoSystem_FindSensorByIpAddress(system_, &ipAddress_, &sensor_)) != kOK) {

                printf("Cannot connect to sensor with ip: %s\n", config_.getParameterValue("Sensor", "SensorIP").c_str());
                printf("Error: GoSystem_FindSensorByIpAddress:%d\n", status);
                connectionSucceded = false;
                GoDestroy(system_);
                system_ = kNULL;
                sensor_ = kNULL;
                return connectionSucceded;
            }
        }
        else
        {
            if ((status = GoSystem_FindSensorById(system_, id_, &sensor_)) != kOK) {
                printf("Cannot connect to sensor with id: %d\n", id_);
                printf("Error: GoSystem_FindSensorById: %d\n", status);
                connectionSucceded = false;
                GoDestroy(system_);
                system_ = kNULL;
                sensor_ = kNULL;
                return connectionSucceded;
            }
        }

        // Create connection to GoSensor object
        if ((status = GoSensor_Connect(sensor_)) != kOK) {
            printf("Error: GoSensor_Connect: %d\n", status);
            connectionSucceded = false;
            return connectionSucceded;
        }

        // Enable sensor data channel
        if ((status = GoSystem_EnableData(system_, kTRUE)) != kOK) {
            printf("Error: GoSensor_EnableData: %d\n", status);
            connectionSucceded = false;
            return connectionSucceded;
        }

        // retrieve setup handle
        if ((setup_ = GoSensor_Setup(sensor_)) == kNULL)
        {
            printf("Error: GoSensor_Setup: Invalid Handle\n");
            connectionSucceded = false;
            return connectionSucceded;
        }

        connectionSucceded = true;

        // retrieve the transformations
        transform_ = GoSensor_Transform(sensor_);

        // retrieve total number of profile points prior to starting the sensor
        if (GoSetup_UniformSpacingEnabled(setup_))
        {
            // Uniform spacing is enabled. The number is based on the X Spacing setting
            profilePointCount_ = GoSetup_XSpacingCount(setup_, GO_ROLE_MAIN);
        }
        else
        {
            // non-uniform spacing is enabled. The max number is based on the number of columns used in the camera. 
            profilePointCount_ = GoSetup_FrontCameraWidth(setup_, GO_ROLE_MAIN);
        }

        profileBuffer_ = new ProfilePoint[profilePointCount_];

        if (profileBuffer_ == kNULL)
        {
            printf("Error: Cannot allocate profileData, %d points\n", profilePointCount_);
            connectionSucceded = false;
            return connectionSucceded;
        }

        printf("\nSensor connected.\n");

        computeMatrixFromPointToSensor();
    }
    catch (const std::exception& ex)
    {
        connectionSucceded = false;
        printf("\nError: Sensor not connected.\n");
        std::cout << ex.what() << std::endl;
    }

    GoSensor_EnableData(sensor_, kTRUE);
    return connectionSucceded;
}

bool Sensor::disconnect()
{
    bool disconnectionSuceeded = true;

    try
    {
        if (system_ != kNULL && sensor_ != kNULL)
        {
            GoSensor_Disconnect(sensor_);
            printf("\nSensor disconnected.\n");
        }
        else
        {
            printf("\nError: Cannot disconnect sensor.\n");
            disconnectionSuceeded = false;
        }

        /*if (setup_ != kNULL)
        {
            GoDestroy(setup_);
            setup_ = kNULL;
        }

        if (transform_ != kNULL)
        {
            GoDestroy(transform_);
            transform_ = kNULL;
        }*/

        // Cleanup
        if (profileBuffer_ != NULL)
        {
            delete[] profileBuffer_;
            profileBuffer_ = NULL;
        }

        if (system_ != kNULL)
        {
            GoDestroy(system_);
            system_ = kNULL;
        }
        
        if (api_ != kNULL)
        {
            GoDestroy(api_);
            api_ = kNULL;
        }

        return disconnectionSuceeded;

    }
    catch (const std::exception& ex)
    {
        printf("\nError: Cannot disconnect sensor.\n");
        std::cout << ex.what() << std::endl;
        return false;
    }

    return true;
}

bool Sensor::start() {
    kStatus status;

    try
    {
        // Start Gocator sensor
        if (system_ != kNULL)
        {
            if ((status = GoSystem_Start(system_)) != kOK) {
                printf("\nError: GoSensor_Start: %d\n", status);
                return false;
            }
            return true;
        }
        else
        {
            printf("\nError: Cannot start sensor.\n");
            return false;
        }
    }
    catch (const std::exception& ex)
    {
        printf("\nError: Cannot start sensor.\n");
        std::cout << ex.what() << std::endl;
        return false;
    }

    return true;
}

bool Sensor::stop()
{
    kStatus status;

    try
    {
        if (system_ != kNULL)
        {
            // Stop Gocator sensor
            if ((status = GoSystem_Stop(system_)) != kOK) {
                printf("\nError: GoSensor_Stop: %d\n", status);
                return false;
            }
        }
        else
        {
            printf("\nError: Cannot stop sensor.\n");
            return false;
        }
    }
    catch (const std::exception& ex)
    {
        printf("\nError: Cannot stop sensor.\n");
        std::cout << ex.what() << std::endl;
        return false;
    }

    return true;
}

void Sensor::getMatrixFromPointToSensor(Eigen::Matrix4d& m)
{
    m = matrixFromPointToSensor_;
}

void Sensor::getMatrixFromPointToSensorInverse(Eigen::Matrix4d& m)
{
    m = matrixFromPointToSensorInverse_;
}

int Sensor::getProfilePointCount()
{
    return profilePointCount_;
}

double Sensor::getEncoderResolution()
{
    return (double)GoTransform_EncoderResolution(transform_);
}

double Sensor::getEncoderSpacing()
{
    return (double)GoSetup_EncoderSpacing(setup_);
}

Frame Sensor::receiveData(unsigned int timeout) {
    Frame currentFrame;
    // Receive data from sensor with timeout
    if (GoSystem_ReceiveData(system_, &dataset_, timeout) == kOK)
    {
        //printf("Data message received:\n");
        //printf("Dataset count: %u\n", (k32u)GoDataSet_Count(dataset));
        // each result can have multiple data items
        // loop through all items in result message
        for (unsigned int i = 0; i < GoDataSet_Count(dataset_); ++i)
        {
            dataObj_ = GoDataSet_At(dataset_, i);
            //Retrieve GoStamp message
            switch (GoDataMsg_Type(dataObj_))
            {
            case GO_DATA_MESSAGE_TYPE_STAMP:
            {
                GoStampMsg stampMsg = dataObj_;

                //printf("Stamp Message batch count: %u\n", (k32u)GoStampMsg_Count(stampMsg));
                for (unsigned int j = 0; j < GoStampMsg_Count(stampMsg); ++j)
                {
                    stamp_ = GoStampMsg_At(stampMsg, j);
                    //printf("  Timestamp: %llu\n", stamp->timestamp);
                    //printf("  Encoder: %lld\n", stamp->encoder);
                    //printf("  Frame index received: %llu\n", stamp->frameIndex);

                    currentFrame.timestamp = stamp_->timestamp;
                    currentFrame.encoder = stamp_->encoder;
                    currentFrame.frameIndex = stamp_->frameIndex;
                    currentFrame.isValid = true;
                }
            }
            break;
            case GO_DATA_MESSAGE_TYPE_UNIFORM_PROFILE:
            {
                GoResampledProfileMsg profileMsg = dataObj_;

                //printf("Resampled Profile Message batch count: %u\n", (k32u)GoResampledProfileMsg_Count(profileMsg));

                for (unsigned int k = 0; k < GoResampledProfileMsg_Count(profileMsg); ++k)
                {
                    unsigned int validPointCount = 0;
                    short* data = GoResampledProfileMsg_At(profileMsg, k);
                    double XResolution = NM_TO_MM(GoResampledProfileMsg_XResolution(profileMsg));
                    double ZResolution = NM_TO_MM(GoResampledProfileMsg_ZResolution(profileMsg));
                    double XOffset = UM_TO_MM(GoResampledProfileMsg_XOffset(profileMsg));
                    double ZOffset = UM_TO_MM(GoResampledProfileMsg_ZOffset(profileMsg));

                    //translate 16-bit range data to engineering units and copy profiles to memory array
                    for (unsigned int arrayIndex = 0; arrayIndex < GoResampledProfileMsg_Width(profileMsg); ++arrayIndex)
                    {
                        if (data[arrayIndex] != INVALID_RANGE_16BIT)
                        {
#ifdef LEFT_HAND_SYSTEM
                            profileBuffer_[arrayIndex].x = -(XOffset + XResolution * arrayIndex); // reverse x because Gocator system is left-hand
                            profileBuffer_[arrayIndex].z = ZOffset + ZResolution * data[arrayIndex];
#else
                            profileBuffer_[arrayIndex].x = (XOffset + XResolution * arrayIndex);
                            profileBuffer_[arrayIndex].z = ZOffset + ZResolution * data[arrayIndex];
#endif
                            validPointCount++;
                        }
                        else
                        {
#ifdef LEFT_HAND_SYSTEM
                            profileBuffer_[arrayIndex].x = -(XOffset + XResolution * arrayIndex); // reverse x because Gocator system is left-hand
                            profileBuffer_[arrayIndex].z = NAN; // INVALID_RANGE_DOUBLE;
#else
                            profileBuffer_[arrayIndex].x = (XOffset + XResolution * arrayIndex);
                            profileBuffer_[arrayIndex].z = NAN; // INVALID_RANGE_DOUBLE;
#endif
                        }
                    }
                    printf("  Profile Valid Point %d out of max %d\n", validPointCount, getProfilePointCount());
                    currentFrame.profileData.insert(currentFrame.profileData.end(), profileBuffer_, profileBuffer_ + GoResampledProfileMsg_Width(profileMsg));
                }
            }
            break;
            case GO_DATA_MESSAGE_TYPE_PROFILE_POINT_CLOUD: // Note this is NON resampled profile            
            {
                GoProfileMsg profileMsg = dataObj_;
                //printf("Profile Message batch count: %u\n", (k32u)GoProfileMsg_Count(profileMsg));

                for (unsigned int k = 0; k < GoProfileMsg_Count(profileMsg); ++k)
                {
                    kPoint16s* data = GoProfileMsg_At(profileMsg, k);
                    unsigned int validPointCount = 0;
                    double XResolution = NM_TO_MM(GoProfileMsg_XResolution(profileMsg));
                    double ZResolution = NM_TO_MM(GoProfileMsg_ZResolution(profileMsg));
                    double XOffset = UM_TO_MM(GoProfileMsg_XOffset(profileMsg));
                    double ZOffset = UM_TO_MM(GoProfileMsg_ZOffset(profileMsg));

                    //translate 16-bit range data to engineering units and copy profiles to memory array
                    for (unsigned int arrayIndex = 0; arrayIndex < GoProfileMsg_Width(profileMsg); ++arrayIndex)
                    {
                        if (data[arrayIndex].x != INVALID_RANGE_16BIT)
                        {
#ifdef LEFT_HAND_SYSTEM
                            profileBuffer_[arrayIndex].x = -(XOffset + XResolution * data[arrayIndex].x); // reverse x because Gocator system is left-hand
                            profileBuffer_[arrayIndex].z = ZOffset + ZResolution * data[arrayIndex].y;
#else
                            profileBuffer_[arrayIndex].x = (XOffset + XResolution * data[arrayIndex].x); // reverse x because Gocator system is left-hand
                            profileBuffer_[arrayIndex].z = ZOffset + ZResolution * data[arrayIndex].y;
#endif
                            validPointCount++;
                        }
                        else
                        {
                            profileBuffer_[arrayIndex].x = NAN; //INVALID_RANGE_DOUBLE;
                            profileBuffer_[arrayIndex].z = NAN; //INVALID_RANGE_DOUBLE;
                        }
                    }
                    //printf("  Profile Valid Point %d out of max %d\n", validPointCount, profilePointCount);

                    currentFrame.profileData.insert(currentFrame.profileData.end(), profileBuffer_, profileBuffer_ + GoResampledProfileMsg_Width(profileMsg));

                    //frames.push_back(std::move(currentFrame));
                }
            }
            break;
            case GO_DATA_MESSAGE_TYPE_PROFILE_INTENSITY:
            {
                //kSize validPointCount = 0;
                GoProfileIntensityMsg intensityMsg = dataObj_;
                printf("Intensity Message batch count: %u\n", (k32u)GoProfileIntensityMsg_Count(intensityMsg));

                for (unsigned int k = 0; k < GoProfileIntensityMsg_Count(intensityMsg); ++k)
                {
                    unsigned char* data = GoProfileIntensityMsg_At(intensityMsg, k);
                    for (unsigned int arrayIndex = 0; arrayIndex < GoProfileIntensityMsg_Width(intensityMsg); ++arrayIndex)
                    {
                        profileBuffer_[arrayIndex].intensity = data[arrayIndex];
                    }
                }
            }
            break;
            default:
            {
                std::cout << "Unknown type:\t" << GoDataMsg_Type(dataObj_) << std::endl;
            }
            }
        }
        GoDestroy(dataset_);
    }
    else
    {
        currentFrame.timestamp = 0;
        currentFrame.encoder = 0;
        currentFrame.frameIndex = 0;
        currentFrame.isValid = false;
    }

    //std::this_thread::sleep_for(std::chrono::milliseconds(3));
    //delete[] profileBuffer_;
    //profileBuffer_ = new ProfilePoint[profilePointCount_];
    return currentFrame;
}

void Sensor::computeMatrixFromPointToSensor()
{
    // read sensor transformation table
    k64f x = GoTransform_X(transform_, GO_ROLE_MAIN);
    k64f y = GoTransform_Y(transform_, GO_ROLE_MAIN);
    k64f z = GoTransform_Z(transform_, GO_ROLE_MAIN);
    k64f rotX = GoTransform_XAngle(transform_, GO_ROLE_MAIN) * M_PI / 180.0;
    k64f rotY = GoTransform_YAngle(transform_, GO_ROLE_MAIN) * M_PI / 180.0;
    k64f rotZ = GoTransform_ZAngle(transform_, GO_ROLE_MAIN) * M_PI / 180.0;
    TransformationTable transformationTable((double)x, (double)y, (double)z, (double)rotX, (double)rotY, (double)rotZ);

    std::cout << std::endl;

    std::cout << "Transformation table -> x offset: " << transformationTable.getXOffset() << std::endl;
    std::cout << "Transformation table -> y offset: " << transformationTable.getYOffset() << std::endl;
    std::cout << "Transformation table -> z offset: " << transformationTable.getZOffset() << std::endl;
    std::cout << "Transformation table -> x angle: " << transformationTable.getXAngle() << std::endl;
    std::cout << "Transformation table -> y angle: " << transformationTable.getYAngle() << std::endl;
    std::cout << "Transformation table -> z angle: " << transformationTable.getZAngle() << std::endl;

    std::cout << std::endl;

    // set sensor information
    kChar* modelDisplayName = new char[10];
    kSize capacity = 10;
    GoSensor_ModelDisplayName(sensor_, modelDisplayName, capacity);
    sensorSerialNumber_ = (int) GoSensor_Id(sensor_);
    std::string model = modelDisplayName;
    delete[] modelDisplayName;
    std::cout << "Sensor model: " << model << std::endl;
    std::cout << "Sensor serial number: " << sensorSerialNumber_ << std::endl;
    SensorInformation sensorInformation(model);

    std::cout << "minResolutionX: " << sensorInformation.getMinResolutionX() << std::endl;
    std::cout << "maxResolutionX: " << sensorInformation.getMaxResolutionX() << std::endl;
    std::cout << "resolutionZ: " << sensorInformation.getResolutionZ() << std::endl;
    std::cout << "linearityZ: " << sensorInformation.getLinearityZ() << std::endl;
    std::cout << "nearFieldOfView: " << sensorInformation.getNearFieldOfView() << std::endl;
    std::cout << "farFieldOfView: " << sensorInformation.getFarFieldOfView() << std::endl;
    std::cout << "clearenceDistance: " << sensorInformation.getClearenceDistance() << std::endl;
    std::cout << "measurementRange: " << sensorInformation.getMeasurementRange() << std::endl;

    std::cout << std::endl;

    // Create matrices
    // matrix sensor information
    // Define the rotation angle in radians
    double angle = M_PI; // Rotate by 180 degrees

    // Define the rotation matrix around the y-axis
    Eigen::Transform<double, 3, Eigen::Affine> transformRotY;
    transformRotY = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY());

    // Convert the transformation to a 4x4 matrix
    Eigen::Matrix4d matrixSensorInformation = transformRotY.matrix();
    matrixSensorInformation(0, 3) = 0.0;
    matrixSensorInformation(1, 3) = 0.0;
    matrixSensorInformation(2, 3) = sensorInformation.getClearenceDistance() + (sensorInformation.getMeasurementRange() / 2.0);

    // Print the rotation matrix
    std::cout << "Matrix sensor information:\n" << matrixSensorInformation << std::endl << std::endl;;

    // matrix transformation table
    // Define rotation angles (in radians) and translation distances
    double angle_x = transformationTable.getXAngle();
    double angle_y = transformationTable.getYAngle();
    double angle_z = transformationTable.getZAngle();
    double translation_x = transformationTable.getXOffset();
    double translation_y = transformationTable.getYOffset();
    double translation_z = transformationTable.getZOffset();

    // Define rotation matrices around x, y, z axes
    Eigen::Matrix3d rotation_x;
    rotation_x << 1, 0, 0,
        0, cos(angle_x), -sin(angle_x),
        0, sin(angle_x), cos(angle_x);

    Eigen::Matrix3d rotation_y;
    rotation_y << cos(angle_y), 0, sin(angle_y),
        0, 1, 0,
        -sin(angle_y), 0, cos(angle_y);

    Eigen::Matrix3d rotation_z;
    rotation_z << cos(angle_z), -sin(angle_z), 0,
        sin(angle_z), cos(angle_z), 0,
        0, 0, 1;

    // Define translation vector
    Eigen::Vector3d translation(translation_x, translation_y, translation_z);

    // Create a 4x4 transformation matrix
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
    transformation_matrix.block<3, 3>(0, 0) = rotation_x * rotation_y * rotation_z; // Apply rotations
    transformation_matrix.block<3, 1>(0, 3) = translation; // Apply translation

    std::cout << "Matrix transformation table:" << std::endl << transformation_matrix << std::endl << std::endl;;

    matrixFromPointToSensor_ =  matrixSensorInformation * transformation_matrix;
    matrixFromPointToSensorInverse_ = matrixFromPointToSensor_.inverse();

    std::cout << "Matrix from point to sensor:" << std::endl << matrixFromPointToSensor_ << std::endl;
}

int Sensor::getSensorSerialNumber()
{
    return sensorSerialNumber_;
}

GoSensor Sensor::getSensor() {
    return sensor_;
}

void Sensor::SetTriggerSoftware() {
    GoSetup_SetTriggerSource(setup_, GO_TRIGGER_SOFTWARE);
}