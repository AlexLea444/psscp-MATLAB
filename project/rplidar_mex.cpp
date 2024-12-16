#include "mex.h"
#include "rplidar.h"
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <cstring>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

using namespace rp::standalone::rplidar;

static RPlidarDriver *drv = NULL;
static bool isConnected = false;

bool checkRPLIDARHealth(RPlidarDriver *drv)
{
    u_result op_result;
    rplidar_response_device_health_t healthinfo;
    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) {
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            mexPrintf("RPLidar internal error detected.\n");
            return false;
        }
        return true;
    } else {
        mexPrintf("Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

void setupLidar(const char *portPath, _u32 baudrate)
{
    if (drv != NULL) {
        RPlidarDriver::DisposeDriver(drv);
        drv = NULL;
    }
    
    drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
    if (!drv) {
        mexErrMsgIdAndTxt("MATLAB:lidarSetup:driverCreateFail", "Insufficient memory to create driver.");
        return;
    }

    if (IS_OK(drv->connect(portPath, baudrate))) {
        rplidar_response_device_info_t devinfo;
        if (IS_OK(drv->getDeviceInfo(devinfo))) {
            isConnected = true;
            mexPrintf("LIDAR connected successfully.\n");
        } else {
            mexErrMsgIdAndTxt("MATLAB:lidarSetup:deviceInfoFail", "Cannot retrieve device info.");
            RPlidarDriver::DisposeDriver(drv);
            drv = NULL;
        }
    } else {
        mexErrMsgIdAndTxt("MATLAB:lidarSetup:connectionFail", "Cannot connect to LIDAR.");
        RPlidarDriver::DisposeDriver(drv);
        drv = NULL;
    }

    if (isConnected && !checkRPLIDARHealth(drv)) {
        mexErrMsgIdAndTxt("MATLAB:lidarSetup:healthCheckFail", "LIDAR health check failed.");
    }
}

void startLidarScan()
{
    if (!isConnected) {
        mexErrMsgIdAndTxt("MATLAB:lidarScan:notConnected", "LIDAR is not connected.");
        return;
    }
    drv->startMotor();
    drv->startScan(0, 1);
}

void getLidarData(mxArray *plhs[])
{
    if (!isConnected) {
        mexErrMsgIdAndTxt("MATLAB:lidarData:notConnected", "LIDAR is not connected.");
        return;
    }

    rplidar_response_measurement_node_hq_t nodes[8192];
    size_t count = _countof(nodes);
    if (IS_OK(drv->grabScanDataHq(nodes, count))) {
        drv->ascendScanData(nodes, count);

        mxArray *angles = mxCreateDoubleMatrix(count, 1, mxREAL);
        mxArray *distances = mxCreateDoubleMatrix(count, 1, mxREAL);
        mxArray *qualities = mxCreateDoubleMatrix(count, 1, mxREAL);

        double *angles_ptr = mxGetPr(angles);
        double *distances_ptr = mxGetPr(distances);
        double *qualities_ptr = mxGetPr(qualities);

        for (size_t pos = 0; pos < count; ++pos) {
            angles_ptr[pos] = nodes[pos].angle_z_q14 * 90.0 / (1 << 14);
            distances_ptr[pos] = nodes[pos].dist_mm_q2 / 4.0f;
            qualities_ptr[pos] = nodes[pos].quality;
        }

        plhs[0] = angles;
        plhs[1] = distances;
        plhs[2] = qualities;
    } else {
        mexErrMsgIdAndTxt("MATLAB:lidarData:dataFetchFail", "Failed to grab scan data.");
    }
}

void closeLidar()
{
    if (drv != NULL) {
        drv->stop();
        drv->stopMotor();
        RPlidarDriver::DisposeDriver(drv);
        drv = NULL;
        isConnected = false;
        mexPrintf("LIDAR connection closed.\n");
    }
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    if (nrhs < 1 || !mxIsChar(prhs[0])) {
        mexErrMsgIdAndTxt("MATLAB:mexFunction:invalidInput", "First argument must be a command string.");
        return;
    }

    char command[64];
    mxGetString(prhs[0], command, sizeof(command));

    if (strcmp(command, "setup") == 0) {
        const char *portPath = nrhs > 1 && mxIsChar(prhs[1]) ? mxArrayToString(prhs[1]) : "/dev/ttyUSB0";
        _u32 baudrate = nrhs > 2 && mxIsNumeric(prhs[2]) ? (unsigned int) mxGetScalar(prhs[2]) : 115200;
        setupLidar(portPath, baudrate);
    } else if (strcmp(command, "startScan") == 0) {
        startLidarScan();
    } else if (strcmp(command, "getData") == 0) {
        getLidarData(plhs);
    } else if (strcmp(command, "close") == 0) {
        closeLidar();
    } else {
        mexErrMsgIdAndTxt("MATLAB:mexFunction:unknownCommand", "Unknown command.");
    }
}

