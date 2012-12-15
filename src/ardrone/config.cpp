// Copyright(C) 2012 puku0x

#include "ardrone.h"

// --------------------------------------------------------------------------
// ARDrone::initConfig()
// Description  : Initialize the Config.
// Return value : SUCCESS: 1  FAILED: 0
// --------------------------------------------------------------------------
int ARDrone::initConfig(void)
{
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
// Description  : Get current configurations of AR.Drone.
// Return value : SUCCESS: 1  FAILED: 0
// --------------------------------------------------------------------------
int ARDrone::getConfig(void)
{
    return 1;
}

// --------------------------------------------------------------------------
// ARDrone::finalizeConfig()
// Description  : Finalize configuration.
// Return value : NONE
// --------------------------------------------------------------------------
void ARDrone::finalizeConfig(void)
{
}