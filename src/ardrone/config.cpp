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
// ARDrone::initConfig()
// Description  : Initialize the Config.
// Return value : SUCCESS: 1  FAILURE: 0
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

        // Send flat trim
        sockCommand.sendf("AT*FTRIM=%d,\r", seq++);
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

        // Set video codec
        sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", seq++, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
        sockCommand.sendf("AT*CONFIG=%d,\"video:video_codec\",\"%d\"\r", seq++, 0x81);   // H264_360P_CODEC
        //sockCommand.sendf("AT*CONFIG=%d,\"video:video_codec\",\"%d\"\r", seq++, 0x82); // MP4_360P_H264_720P_CODEC
        //sockCommand.sendf("AT*CONFIG=%d,\"video:video_codec\",\"%d\"\r", seq++, 0x83); // H264_720P_CODEC
        //sockCommand.sendf("AT*CONFIG=%d,\"video:video_codec\",\"%d\"\r", seq++, 0x88); // MP4_360P_H264_360P_CODEC
        Sleep(100);

        // Set video channel to default
        sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", seq++, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
        sockCommand.sendf("AT*CONFIG=%d,\"video:video_channel\",\"0\"\r", seq++);
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

        // Send flat trim
        sockCommand.sendf("AT*FTRIM=%d,\r", seq++);
        Sleep(100);

        // Enable video
        sockCommand.sendf("AT*CONFIG=%d,\"general:video_enable\",\"TRUE\"\r", seq++);
        Sleep(100);

        // Disable bitrate control mode
        sockCommand.sendf("AT*CONFIG=%d,\"video:bitrate_ctrl_mode\",\"0\"\r", seq++);
        Sleep(100);

        // Set video codec
        sockCommand.sendf("AT*CONFIG=%d,\"video:video_codec\",\"%d\"\r", seq++, 0x20);   // UVLC_CODEC
        //sockCommand.sendf("AT*CONFIG=%d,\"video:video_codec\",\"%d\"\r", seq++, 0x40); // P264_CODEC
        Sleep(100);
        
        // Set video channel to default
        sockCommand.sendf("AT*CONFIG=%d,\"video:video_channel\",\"0\"\r", seq++);
        Sleep(100);
    }
    
    return 1;
}

// --------------------------------------------------------------------------
// ARDrone::getConfig()
// Description  : Get current configurations of AR.Drone.
// Return value : SUCCESS: 1  FAILURE: 0
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