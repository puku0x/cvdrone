#include "ardrone.h"

// --------------------------------------------------------------------------
// ARDrone::initConfig()
// Initialize the Config.
// Return value SUCCESS: 1  FAILED: 0
// --------------------------------------------------------------------------
int ARDrone::initConfig(void)
{
    //// Open the socket.
    //if (!sockConfig.open(ip, ARDRONE_CONFIG_PORT)) {
    //    printf("ERROR: TCPSocket::open(port=%d) failed. (%s, %d)\n", ARDRONE_CONFIG_PORT, __FILE__, __LINE__);
    //    return 0;
    //}

    // AR.Drone 2.0
    if (version.major == ARDRONE_VERSION_2) {
        // Send undocumented command
        sockCommand.sendf("AT*PMODE=%d,%d\r", seq++, 2);
        Sleep(100);

        // Send undocumented command
        sockCommand.sendf("AT*MISC=%d,%d,%d,%d,%d\r", seq++, 2, 20, 2000, 3000);
        Sleep(100);

        // Set the configuration IDs
        sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", seq++, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
        sockCommand.sendf("AT*CONFIG=%d,\"custom:session_id\",\"%s\"\r", seq++, ARDRONE_SESSION_ID);
        Sleep(100);
        sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", seq++, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
        sockCommand.sendf("AT*CONFIG=%d,\"custom:profile_id\",\"%s\"\r", seq++, ARDRONE_PROFILE_ID);
        Sleep(100);
        sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", seq++, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
        sockCommand.sendf("AT*CONFIG=%d,\"custom:application_id\",\"%s\"\r", seq++, ARDRONE_APPLOCATION_ID);
        Sleep(100);

        // Enable video
        sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", seq++, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
        sockCommand.sendf("AT*CONFIG=%d,\"general:video_enable\",\"TRUE\"\r", seq++);
        Sleep(100);

        // Disable bitrate control mode
        sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", seq++, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
        sockCommand.sendf("AT*CONFIG=%d,\"video:bitrate_ctrl_mode\",\"0\"\r", seq++);
        Sleep(100);

        // Output video with 360p
        sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", seq++, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
        sockCommand.sendf("AT*CONFIG=%d,\"video:video_codec\",\"%d\"\r", seq++, 0x81);
        //sockCommand.sendf("AT*CONFIG=%d,\"video:video_codec\",\"%d\"\r", seq++, 0x82);
        Sleep(100);

        // Output video with 720p
        //sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", seq++, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
        //sockCommand.sendf("AT*CONFIG=%d,\"video:video_codec\",\"%d\"\r", seq++, 0x83);
        //Sleep(100);

        // Set video channel
        sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", seq++, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
        sockCommand.sendf("AT*CONFIG=%d,\"video:video_channel\",\"0\"\r", seq++);
        Sleep(100);

        // Send flat trim
        sockCommand.sendf("AT*FTRIM=%d,\r", seq++);
        Sleep(100);
    }
    // AR.Drone 1.0
    else {
        // Send undocumented command
        sockCommand.sendf("AT*PMODE=%d,%d\r", seq++, 2);
        Sleep(100);

        // Send undocumented command
        sockCommand.sendf("AT*MISC=%d,%d,%d,%d,%d\r", seq++, 2, 20, 2000, 3000);
        Sleep(100);

        // Enable video
        sockCommand.sendf("AT*CONFIG=%d,\"general:video_enable\",\"TRUE\"\r", seq++);
        Sleep(100);

        // Disable bitrate control mode
        sockCommand.sendf("AT*CONFIG=%d,\"video:bitrate_ctrl_mode\",\"0\"\r", seq++);
        Sleep(100);

        // Output video with UVLC
        sockCommand.sendf("AT*CONFIG=%d,\"video:video_codec\",\"%d\"\r", seq++, 0x20);
        Sleep(100);

        // Output video with P264
        //sockCommand.sendf("AT*CONFIG=%d,\"video:video_codec\",\"%d\"\r", seq++, 0x40);
        //Sleep(100);
        
        // Set video channel
        sockCommand.sendf("AT*CONFIG=%d,\"video:video_channel\",\"0\"\r", seq++);
        Sleep(100);

        // Send flat trim
        sockCommand.sendf("AT*FTRIM=%d,\r", seq++);
        Sleep(100);
    }
    
    return 1;
}

// --------------------------------------------------------------------------
// ARDrone::getConfig()
// Get the AR.Drone's configurations.
// Not work yet.
// Return value SUCCESS: 1  FAILED: 0
// --------------------------------------------------------------------------
int ARDrone::getConfig(void)
{
    //// Send ACK
    //sockCommand.sendf("AT*CTRL=%d,4,0\r", seq++);

    //// Receive the data
    //char buf[1024];
    //int size = sockConfig.receive((void*)&buf, sizeof(buf));
    //printf("size = %d\n", size);

    //// Received something
    //if (size > 0) {
    //    printf("received = %s\n", buf);
    //}

    return 1;
}

// --------------------------------------------------------------------------
// ARDrone::finalizeConfig()
// Finalize configuration.
// Return value NONE
// --------------------------------------------------------------------------
void ARDrone::finalizeConfig(void)
{
    //// Close the socket
    //sockConfig.close();
}