#define _USE_MATH_DEFINES
#include <cmath>
#include <GoSdk/GoSdk.h>
#include <iostream>
#include <vector>
#include <regex>
#include <thread>
#include <chrono>
#include <Eigen/Dense>

#include "SensorInformation.h"
#include "TransformationTable.h"
#include "AppConfiguration.h"

//#define RECEIVE_TIMEOUT         (20000000)
#define RECEIVE_TIMEOUT         (20000000) // 20s timeout
#define INVALID_RANGE_16BIT     ((signed short)0x8000)          // gocator transmits range data as 16-bit signed integers. 0x8000 signifies invalid range data. 
#define DOUBLE_MAX              ((k64f)1.7976931348623157e+308) // 64-bit double - largest positive value.  
#define INVALID_RANGE_DOUBLE    ((k64f)-DOUBLE_MAX)             // floating point value to represent invalid range data.    
//#define SENSOR_IP               "127.0.0.1" 

#define NM_TO_MM(VALUE) (((k64f)(VALUE))/1000000.0)
#define UM_TO_MM(VALUE) (((k64f)(VALUE))/1000.0)

typedef struct ProfilePoint
{
    double x;   // x-coordinate in engineering units (mm) - position along laser line
    double z;   // z-coordinate in engineering units (mm) - height (at the given x position)
    unsigned char intensity;
} ProfilePoint;

class Frame {
public:
    std::vector<ProfilePoint> profileData;
    k64u timestamp;
    k64s encoder;
    k64u frameIndex;
    bool isValid;
};

class Sensor
{
private:
    AppConfiguration config_;
    kAssembly api_;
    GoSystem system_;
    GoSensor sensor_;
    kIpAddress ipAddress_;
    k32u id_;
    GoSetup setup_;
    k32u profilePointCount_;
    ProfilePoint* profileBuffer_;
    GoDataSet dataset_;
    GoDataMsg dataObj_;
    GoStamp* stamp_;
    GoTransform transform_;
    Eigen::Matrix4d matrixFromPointToSensor_;
    Eigen::Matrix4d matrixFromPointToSensorInverse_;
    int sensorSerialNumber_;

    void computeMatrixFromPointToSensor();

public:
    Sensor(const AppConfiguration& config);
    ~Sensor();
    bool connect();
    bool start();
    bool stop();
    bool disconnect();
    Frame receiveData(unsigned int timeout = RECEIVE_TIMEOUT);
    void getMatrixFromPointToSensor(Eigen::Matrix4d& m);
    void getMatrixFromPointToSensorInverse(Eigen::Matrix4d& m);
    double getEncoderSpacing(); // deg space every trigger
    double getEncoderResolution(); // deg/thick
    int getProfilePointCount();
    int getSensorSerialNumber();
    void SetTriggerSoftware();
    GoSensor getSensor();
};

