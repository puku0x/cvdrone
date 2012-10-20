#include "ardrone.h"

// --------------------------------------------------------------------------
// ARDrone::initNavdata()
// Initialize Navdata.
// Return value SUCCESS: 1  FAILED: 0 
// --------------------------------------------------------------------------
int ARDrone::initNavdata(void)
{
    // Open the socket
    if (!sockNavdata.open(ip, ARDRONE_NAVDATA_PORT)) {
        printf("ERROR: UDPSocket::open(port=%d) failed. (%s, %d)\n", ARDRONE_NAVDATA_PORT, __FILE__, __LINE__);
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
        printf("ERROR: _beginthreadex() failed. (%s, %d)\n", __FILE__, __LINE__);
        return 0;
    }

    return 1;
}

// --------------------------------------------------------------------------
// ARDrone::loopNavdata()
// Thread function.
// Return value 0
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
// Obtaining Navdata information.
// Return value SUCCESS: 1  FAILED: 0
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
            // Update Navdata
            WaitForSingleObject(mutexNavdata, INFINITE);
            memcpy(&navdata, buf, sizeof(NAVDATA));
            ReleaseMutex(mutexNavdata);
        }
    }

    return 1;
}

// --------------------------------------------------------------------------
// ARDrone::getRoll()
// Obtaining role angle.
// Return value Role angle [rad]
// --------------------------------------------------------------------------
double ARDrone::getRoll(void)
{
    return navdata.phi * 0.001 * DEG_TO_RAD;
}

// --------------------------------------------------------------------------
// ARDrone::getPitch()
// Obtaining pitch angle.
// Return value Pitch angle [rad]
// --------------------------------------------------------------------------
double ARDrone::getPitch(void)
{
    return navdata.theta * 0.001 * DEG_TO_RAD;
}

// --------------------------------------------------------------------------
// ARDrone::getYaw()
// Obtaining yaw angle.
// Return value Yaw angle [rad]
// --------------------------------------------------------------------------
double ARDrone::getYaw(void)
{
    return navdata.psi * 0.001 * DEG_TO_RAD;
}

// --------------------------------------------------------------------------
// ARDrone::getAltitude()
// Obtaining altitude.
// Return value Altitude [m]
// --------------------------------------------------------------------------
double ARDrone::getAltitude(void)
{
    return navdata.altitude * 0.001;
}

// --------------------------------------------------------------------------
// ARDrone::getVelocity(X velocity[m/s], Y velocity[m/s], Z velocity[m/s])
// Obtaining velocity.
// Return value Velocity [m/s]
// --------------------------------------------------------------------------
double ARDrone::getVelocity(double *vx, double *vy, double *vz)
{
    if (vx) *vx = navdata.vx * 0.001;
    if (vy) *vy = navdata.vy * 0.001;
    if (vz) *vz = navdata.vz * 0.001;
    return sqrt(navdata.vx*navdata.vx + navdata.vy*navdata.vy + navdata.vz*navdata.vz);
}

// --------------------------------------------------------------------------
// ARDrone::getBatteryPercentage()
// Obtaining tattery percentage.
// Return value Battery percentage [%]
// --------------------------------------------------------------------------
int ARDrone::getBatteryPercentage(void)
{
    return navdata.vbat_flying_percentage;
}

// --------------------------------------------------------------------------
// ARDrone::onGround()
// Check AR.Drone is on ground or not.
// Return value YES:1 NO:0
// --------------------------------------------------------------------------
int ARDrone::onGround(void)
{
    if (navdata.ardrone_state & ARDRONE_FLY_MASK) return 0;
    return 1;
}

// --------------------------------------------------------------------------
// ARDrone::finalizeNavdata()
// Finalize Navdata.
// Return value NONE
// --------------------------------------------------------------------------
void ARDrone::finalizeNavdata(void)
{
    // Disable the loop
    flagNavdata = 0;

    // Destroy teh thread
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