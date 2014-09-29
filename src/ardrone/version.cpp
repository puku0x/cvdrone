// -------------------------------------------------------------------------
// CV Drone (= OpenCV + AR.Drone)
// Copyright(C) 2014 puku0x
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
//! @file   version.cpp
//! @brief  Version check using FTP
//
// -------------------------------------------------------------------------

#include "ardrone.h"

// --------------------------------------------------------------------------
//! @brief   Get the version information via FTP.
//! @return  Result of initialization
//! @retval  1 Success
//! @retval  0 Failure
// --------------------------------------------------------------------------
int ARDrone::getVersionInfo(void)
{
    TCPSocket socket1, socket2;

    // Open the IP address and port
    if (!socket1.open(ip, ARDRONE_FTP_PORT)) {
        CVDRONE_ERROR("TCPSocket::open(port=%d) failed. (%s, %d)\n", ARDRONE_FTP_PORT, __FILE__, __LINE__);
        return 0;
    }

    // Welcome message
    const size_t len = 1024;
    char buf[len] = {'\0'};
    socket1.receive(buf, len);

    // Log in as anonymous
    socket1.sendf("USER %s\r\n\0", "anonymous");
    socket1.receive(buf, len);

    // Set to PASV mode
    int a, b, c, dataport;
    socket1.sendf("PASV\r\n\0");
    socket1.receive(buf, len);
    sscanf(buf, "227 PASV ok (%d,%d,%d,%d,%d,%d)\n", &c, &c, &c, &c, &a, &b);
    dataport = (a << 8) + b;

    // Open the IP address and port
    if (!socket2.open(ip, dataport)) {
        CVDRONE_ERROR("TCPSocket::open(port=%d) failed. (%s, %d)\n", dataport, __FILE__, __LINE__);
        return 0;
    }

    // Send requests
    socket1.sendf("RETR %s\r\n\0", "version.txt");

    // Receive data
    socket2.receive(buf, len);

    // Get version information
    sscanf(buf, "%d.%d.%d", &version.major, &version.minor, &version.revision);
    //printf("AR.Drone Ver %d.%d.%d\n", major, minor, revision);

    // See you
    socket1.close();
    socket2.close();

    return 1;
}

// --------------------------------------------------------------------------
//! @brief   Get the version and the revision number
//! @param   major A pointer to the major version variable
//! @param   minor A pointer to the minor version variable
//! @param   revision A pointer to the revision number variable
//! @return  Major version of AR.Drone
// --------------------------------------------------------------------------
int ARDrone::getVersion(int *major, int *minor, int *revision)
{
    if (major) *major = version.major;
    if (minor) *minor = version.minor;
    if (revision) *revision = version.revision;
    return version.major;
}