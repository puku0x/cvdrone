// -------------------------------------------------------------------------
// CV Drone (= OpenCV + AR.Drone)
// Copyright(C) 2016 puku0x
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
//
//! @file   ardrone.cpp
//! @brief  A source file of AR.Drone class
//
// -------------------------------------------------------------------------

#include "ardrone.h"

// --------------------------------------------------------------------------
//! @brief   Constructor of AR.Drone class
//! @param   ardrone_addr IP address of AR.Drone
//! @return  None
// --------------------------------------------------------------------------
ARDrone::ARDrone(const char *ardrone_addr)
{
    // IP Address
    strncpy(ip, ARDRONE_DEFAULT_ADDR, 16);

    // Sequence number
    seq = 0;

    // Camera image
    img = NULL;

    // Version information
    memset(&version, 0, sizeof(version));

    // Navdata
    memset(&navdata, 0, sizeof(navdata));

    // Configurations
    memset(&config, 0, sizeof(config));

    // Video
    pFormatCtx  = NULL;
    pCodecCtx   = NULL;
    pFrame      = NULL;
    pFrameBGR   = NULL;
    bufferBGR   = NULL;
    pConvertCtx = NULL;

    // Thread for AT command
    threadCommand = NULL;
    mutexCommand  = NULL;

    // Thread for Navdata
    threadNavdata = NULL;
    mutexNavdata  = NULL;

    // Thread for Video
    threadVideo = NULL;
    mutexVideo  = NULL;

    // Open if the IP address was specified
    if (ardrone_addr != NULL) {
        open(ardrone_addr);
    }
}

// --------------------------------------------------------------------------
//! @brief   Destructor of ARDrone class
//! @return  None
// --------------------------------------------------------------------------
ARDrone::~ARDrone()
{
    // See you
    close();
}

// --------------------------------------------------------------------------
//! @brief   Initialize the AR.Drone.
//! @param   ardrone_addr IP address of AR.Drone
//! @return  Result of initialization
//! @retval  1 Success
//! @retval  0 Failure
// --------------------------------------------------------------------------
int ARDrone::open(const char *ardrone_addr)
{
    // Initialize FFmpeg
    av_register_all();
    avformat_network_init();
    av_log_set_level(AV_LOG_QUIET);

    // Save IP address
    strncpy(ip, ardrone_addr, 16);

    // Get version information
    if (!getVersionInfo()) return 0;
    std::cout << "AR.Drone Ver. " << version.major << "." << version.minor << "." << version.revision << "." << std::endl;

    // Initialize AT command
    if (!initCommand()) return 0;

    // Blink LEDs
    setLED(ARDRONE_LED_ANIM_BLINK_GREEN);

    // Initialize Navdata
    if (!initNavdata()) return 0;

    // Initialize Video
    if (!initVideo()) return 0;

    // Wait for updating the status
    //msleep(500);

    // Get configurations
    if (!getConfig()) return 0;

    // Stop LED animation
    setLED(ARDRONE_LED_ANIM_STANDARD);

    // Reset emergency
    resetWatchDog();
    resetEmergency();

    return 1;
}

// --------------------------------------------------------------------------
//! @brief   Update the information of the AR.Drone.
//! @return  Result of update
//! @retval  1 Success
//! @retval  0 Failure
// --------------------------------------------------------------------------
int ARDrone::update(void)
{
    return 1;
}

// --------------------------------------------------------------------------
//! @brief   Finalize the AR.Drone class.
//! @return  None
// --------------------------------------------------------------------------
void ARDrone::close(void)
{
    // Stop AR.Drone
    if (!onGround()) landing();

    // Stop LED animation
    setLED(ARDRONE_LED_ANIM_STANDARD);

    // Finalize video
    finalizeVideo();

    // Finalize Navdata
    finalizeNavdata();

    // Finalize AT command
    finalizeCommand();
}
