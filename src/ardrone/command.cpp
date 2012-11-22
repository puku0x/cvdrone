#include "ardrone.h"

// --------------------------------------------------------------------------
// ARDrone::initCommand()
// Initialize AT command.
// Return value SUCCESS: 1  FAILED: 0
// --------------------------------------------------------------------------
int ARDrone::initCommand(void)
{
    // Open the socket
    if (!sockCommand.open(ip, ARDRONE_COMMAND_PORT)) {
        printf("ERROR: UDPSocket::open(port=%d) failed. (%s, %d)\n", ARDRONE_COMMAND_PORT, __FILE__, __LINE__);
        return 0;
    }

    return 1;
}

// --------------------------------------------------------------------------
// ARDrone::takeoff()
// Take off the AR.Drone.
// Return value NONE
// --------------------------------------------------------------------------
void ARDrone::takeoff(void)
{
    if (navdata.ardrone_state & ARDRONE_EMERGENCY_MASK) sockCommand.sendf("AT*REF=%d,290717952\r", seq++);
    else                                                sockCommand.sendf("AT*REF=%d,290718208\r", seq++);
}

// --------------------------------------------------------------------------
// ARDrone::landing()
// Land the AR.Drone.
// Return value NONE
// --------------------------------------------------------------------------
void ARDrone::landing(void)
{
    sockCommand.sendf("AT*REF=%d,290717696\r", seq++);
}

// --------------------------------------------------------------------------
// ARDrone::emergency()
// Emergency stop.
// Return value NONE
// --------------------------------------------------------------------------
void ARDrone::emergency(void)
{
    sockCommand.sendf("AT*REF=%d,290717952\r", seq++);
}

// --------------------------------------------------------------------------
// ARDrone::move(X velocity[m/s], Y velocity[m/s], Rotational speed[rad/s])
// Move the AR.Drone in 2D plane.
// Return value NONE
// --------------------------------------------------------------------------
void ARDrone::move(double vx, double vy, double vr)
{
    move3D(vx, vy, 0.0, vr);
}

// --------------------------------------------------------------------------
// ARDrone::move3D(X velocity[m/s], Y velocity[m/s], Z velocity[m/s], Rotational speed[rad/s])
// Move the AR.Drone in 3D space.
// Return value NONE
// --------------------------------------------------------------------------
void ARDrone::move3D(double vx, double vy, double vz, double vr)
{
    const float gain = 0.4f;
    float v[4] = {-vy*gain, -vx*gain, vz*gain, -vr*gain};
    int mode = (fabs(vx) > 0.0 || fabs(vy) > 0.0);
    sockCommand.sendf("AT*PCMD=%d,%d,%d,%d,%d,%d\r", seq++, mode, *(int*)(&v[0]), *(int*)(&v[1]), *(int*)(&v[2]), *(int*)(&v[3]));
}

// --------------------------------------------------------------------------
// ARDrone::setCamera(Channel)
// Change the camera channel.
// ARDrone1.0 supports 0, 1, 2, 3.
// ARDrone2.0 supports 0, 1.
// Return value NONE
// --------------------------------------------------------------------------
void ARDrone::setCamera(int channel)
{
    // ARDrone 2.0
    if (version.major == ARDRONE_VERSION_2) {
        sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", seq++, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
        sockCommand.sendf("AT*CONFIG=%d,\"video:video_channel\",\"%d\"\r", seq++, channel);
        Sleep(100);
    }
    // ARDrone 1.0
    else {
        sockCommand.sendf("AT*CONFIG=%d,\"video:video_channel\",\"%d\"\r", seq++, channel);
    }
}

//// --------------------------------------------------------------------------
//// ARDrone::startRecord()
//// Start recording video.
//// This function is only for AR.Drone 2.0
//// You should set a USB key with > 100MB to your drone
//// Return value NONE
//// --------------------------------------------------------------------------
//void ARDrone::startRecord(void)
//{
//    if (version.major == ARDRONE_VERSION_2) {
//        sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", seq++, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
//        sockCommand.sendf("AT*CONFIG=%d,\"video:video_on_usb\",\"TRUE\"\r", seq++);
//        Sleep(100);
//    }
//}
//
//// --------------------------------------------------------------------------
//// ARDrone::stopRecord()
//// Stop recording video.
//// This function is only for AR.Drone 2.0
//// Return value NONE
//// --------------------------------------------------------------------------
//void ARDrone::stopRecord(void)
//{
//    if (version.major == ARDRONE_VERSION_2) {
//        sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", seq++, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
//        sockCommand.sendf("AT*CONFIG=%d,\"video:video_on_usb\",\"FALSE\"\r", seq++);
//        Sleep(100);
//    }
//}

// --------------------------------------------------------------------------
// ARDrone::setAnimation(Flight animation ID, Duration[s])
// Run specified flight animation.
// Return value NONE
// --------------------------------------------------------------------------
void ARDrone::setAnimation(int id, int duration)
{
    sockCommand.sendf("AT*ANIM=%d,%d,%d\r", seq++, id, duration);
    //sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", seq++, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
    //sockCommand.sendf("AT*CONFIG=%d,\"leds:flight_anim\",\"%d,%d\"\r", seq++, id, duration);
    //Sleep(100);
}

// --------------------------------------------------------------------------
// ARDrone::setLED(LED animation ID, Frequency[Hz], Duration[s])
// Run specified LED animation.
// Return value NONE
// --------------------------------------------------------------------------
void ARDrone::setLED(int id, float freq, int duration)
{
    sockCommand.sendf("AT*LED=%d,%d,%d,%d\r", seq++, id, *(int*)(&freq), duration);
    //sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", seq++, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
    //sockCommand.sendf("AT*CONFIG=%d,\"leds:leds_anim\",\"%d,%d,%d\"\r", seq++, id, *(int*)(&freq), duration);
    //Sleep(100);
}

// --------------------------------------------------------------------------
// ARDrone::resetWatchDog()
// Stop hovering.
// Return value NONE
// --------------------------------------------------------------------------
void ARDrone::resetWatchDog(void)
{
    if (navdata.ardrone_state & ARDRONE_COM_WATCHDOG_MASK) sockCommand.sendf("AT*COMWDG=%d\r", seq++);
}

// --------------------------------------------------------------------------
// ARDrone::resetEmergency()
// Disable the emergency lock.
// Return value NONE
// --------------------------------------------------------------------------
void ARDrone::resetEmergency(void)
{
    if (navdata.ardrone_state & ARDRONE_EMERGENCY_MASK) sockCommand.sendf("AT*REF=%d,290717952\r", seq++);
}

// --------------------------------------------------------------------------
// ARDrone::finalizeCommand()
// Finalize AT command
// Return value NONE
// --------------------------------------------------------------------------
void ARDrone::finalizeCommand(void)
{
    // Close the socket
    sockCommand.close();
}