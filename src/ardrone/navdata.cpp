// -------------------------------------------------------------------------
// CV Drone (= OpenCV + AR.Drone)
// Copyright(C) 2013 puku0x
// https://github.com/puku0x/cvdrone
//
// This source file is part of CV Drone library.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of EITHER:
// (1) The GNU Lesser General Public License as published by the Free
//     Software Foundation; either version 2.1 of the License, or (at
//     your option) any later version. The text of the GNU Lesser
//     General Public License is included with this library in the
//     file cvdrone-license-LGPL.txt.
// (2) The BSD-style license that is included with this library in
//     the file cvdrone-license-BSD.txt.
// 
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files
// cvdrone-license-LGPL.txt and cvdrone-license-BSD.txt for more details.
// -------------------------------------------------------------------------

#include "ardrone.h"

// --------------------------------------------------------------------------
// ARDrone::initNavdata()
// Description  : Initialize Navdata.
// Return value : SUCCESS: 1  FAILURE: 0 
// --------------------------------------------------------------------------
int ARDrone::initNavdata(void)
{
    // Open the IP address and port
    if (!sockNavdata.open(ip, ARDRONE_NAVDATA_PORT)) {
        CVDRONE_ERROR("UDPSocket::open(port=%d) was failed. (%s, %d)\n", ARDRONE_NAVDATA_PORT, __FILE__, __LINE__);
        return 0;
    }

    // Clear Navdata
    ZeroMemory(&navdata, sizeof(NAVDATA));

    // Start Navdata
    sockNavdata.sendf("\x01\x00\x00\x00");

    // AR.Drone 2.0
    if (version.major == ARDRONE_VERSION_2) {
        // Disable BOOTSTRAP mode
        sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", seq++, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
        sockCommand.sendf("AT*CONFIG=%d,\"general:navdata_demo\",\"TRUE\"\r", seq++);
        Sleep(100);

        // Seed ACK
        sockCommand.sendf("AT*CTRL=%d,0\r", seq++);
    }
    // AR.Drone 1.0
    else {
        // Disable BOOTSTRAP mode
        sockCommand.sendf("AT*CONFIG=%d,\"general:navdata_demo\",\"TRUE\"\r", seq++);

        // Send ACK
        sockCommand.sendf("AT*CTRL=%d,0\r", seq++);
    }

    // Create a mutex
    mutexNavdata= CreateMutex(NULL, FALSE, NULL);

    // Enable thread loop
    flagNavdata = 1;

    // Create a thread
    UINT id;
    threadNavdata = (HANDLE)_beginthreadex(NULL, 0, runNavdata, this, 0, &id);
    if (threadNavdata == INVALID_HANDLE_VALUE) {
        CVDRONE_ERROR("_beginthreadex() was failed. (%s, %d)\n", __FILE__, __LINE__);
        return 0;
    }

    return 1;
}

// --------------------------------------------------------------------------
// ARDrone::loopNavdata()
// Description  : Thread function for Navdata.
// Return value : SUCCESS:0
// --------------------------------------------------------------------------
UINT ARDrone::loopNavdata(void)
{
    while (flagNavdata) {
        // Get Navdata
        if (!getNavdata()) break;
        Sleep(30);
    }

    // Disable thread loop
    flagNavdata = 0;

    return 0;
}

// --------------------------------------------------------------------------
// ARDrone::getNavdata()
// Description  : Get current navigation data of AR.Drone.
// Return value : SUCCESS: 1  FAILURE: 0
// --------------------------------------------------------------------------
int ARDrone::getNavdata(void)
{
    // Send a request
    sockNavdata.sendf("\x01\x00\x00\x00");

    // Receive data
    int buf[512];
    int size = sockNavdata.receive((void*)&buf, sizeof(buf));

    // Received something
    if (size > 0) {
        // Check header
        if (buf[0] == ARDRONE_NAVDATA_HEADER) {
            // Enable mutex lock
            WaitForSingleObject(mutexNavdata, INFINITE);

            // Update Navdata
            memcpy((void*)&navdata, (const void*)buf, sizeof(NAVDATA));

            // Disable mutex lock
            ReleaseMutex(mutexNavdata);
        }
    }

    return 1;
}

// --------------------------------------------------------------------------
// ARDrone::getRoll()
// Description  : Get current role angle of AR.Drone.
// Return value : Role angle [rad]
// --------------------------------------------------------------------------
double ARDrone::getRoll(void)
{
    return navdata.demo.phi * 0.001 * DEG_TO_RAD;
}

// --------------------------------------------------------------------------
// ARDrone::getPitch()
// Description  : Get current pitch angle of AR.Drone.
// Return value : Pitch angle [rad]
// --------------------------------------------------------------------------
double ARDrone::getPitch(void)
{
    return navdata.demo.theta * 0.001 * DEG_TO_RAD;
}

// --------------------------------------------------------------------------
// ARDrone::getYaw()
// Description  : Get current yaw angle of AR.Drone.
// Return value : Yaw angle [rad]
// --------------------------------------------------------------------------
double ARDrone::getYaw(void)
{
    return navdata.demo.psi * 0.001 * DEG_TO_RAD;
}

// --------------------------------------------------------------------------
// ARDrone::getAltitude()
// Description  : Get current altitude of AR.Drone.
// Return value : Altitude [m]
// --------------------------------------------------------------------------
double ARDrone::getAltitude(void)
{
    return navdata.demo.altitude * 0.001;
}

// --------------------------------------------------------------------------
// ARDrone::getVelocity(X velocity[m/s], Y velocity[m/s], Z velocity[m/s])
// Description  : Get estimated velocity of AR.Drone.
// Return value : Velocity [m/s]
// --------------------------------------------------------------------------
double ARDrone::getVelocity(double *vx, double *vy, double *vz)
{
    if (vx) *vx = navdata.demo.vx * 0.001;
    if (vy) *vy = navdata.demo.vy * 0.001;
    if (vz) *vz = navdata.demo.vz * 0.001;
    return sqrt(navdata.demo.vx*navdata.demo.vx + navdata.demo.vy*navdata.demo.vy + navdata.demo.vz*navdata.demo.vz);
}

// --------------------------------------------------------------------------
// ARDrone::getBatteryPercentage()
// Description  : Get current battery percentage of AR.Drone.
// Return value : Battery percentage [%]
// --------------------------------------------------------------------------
int ARDrone::getBatteryPercentage(void)
{
    return navdata.demo.vbat_flying_percentage;
}

// --------------------------------------------------------------------------
// ARDrone::onGround()
// Description  : Check whether AR.Drone is on ground.
// Return value : YES:1 NO:0
// --------------------------------------------------------------------------
int ARDrone::onGround(void)
{
    if (navdata.ardrone_state & ARDRONE_FLY_MASK) return 0;
    return 1;
}

// --------------------------------------------------------------------------
// ARDrone::finalizeNavdata()
// Description  : Finalize Navdata.
// Return value : NONE
// --------------------------------------------------------------------------
void ARDrone::finalizeNavdata(void)
{
    // Disable thread loop
    flagNavdata = 0;

    // Destroy the thread
    if (threadNavdata != INVALID_HANDLE_VALUE) {
        WaitForSingleObject(threadNavdata, INFINITE);
        CloseHandle(threadNavdata);
        threadNavdata = INVALID_HANDLE_VALUE;
    }

    // Delete the mutex
    if (mutexNavdata != INVALID_HANDLE_VALUE) {
        CloseHandle(mutexNavdata);
        mutexNavdata = INVALID_HANDLE_VALUE;
    }

    // Close the socket
    sockNavdata.close();
}