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
// ARDrone::getVersionInfo()
// Description  : Get version information.
// Return value : SUCCESS: 1  FAILURE: 0
// --------------------------------------------------------------------------
int ARDrone::getVersionInfo(void)
{
    #if _WIN32
    const char *filename = "version.txt";

    // Initialize WinINet
    HINTERNET hInet = InternetOpen(NULL, INTERNET_OPEN_TYPE_DIRECT, NULL, NULL, 0 );

    // Set timeout [ms]
    DWORD ms = 500;
    InternetSetOption(hInet, INTERNET_OPTION_CONNECT_TIMEOUT, &ms, sizeof(ms));

    // Connect to FTP server
    HINTERNET hConnection = InternetConnect(hInet, ip, ARDRONE_VERSION_PORT, "anonymous", "", INTERNET_SERVICE_FTP, INTERNET_FLAG_PASSIVE, 0);
    if (hConnection == NULL) {
        CVDRONE_ERROR("InternetConnect(port=%d) was failed. (%s, %d)\n", ARDRONE_VERSION_PORT, __FILE__, __LINE__);
        InternetCloseHandle(hInet);
        return 0;
    }

    // Clear version
    ZeroMemory(&version, sizeof(ARDRONE_VERSION));

    // Get a file through FTP
    if (!FtpGetFile(hConnection, filename, filename, FALSE, FILE_ATTRIBUTE_NORMAL, INTERNET_FLAG_TRANSFER_BINARY, 0)) {
        CVDRONE_ERROR("FtpGetFile() was failed. (%s, %d)\n", __FILE__, __LINE__);
        InternetCloseHandle(hConnection);
        InternetCloseHandle(hInet);
        return 0;
    }
    // 
    else {
        // Open the file
        FILE *file = fopen(filename, "r");

        // Read FW version
        fscanf(file, "%d.%d.%d\n", &version.major, &version.minor, &version.revision);

        // Close the file
        fclose(file);

        // Delete the file
        remove(filename);
    }

    // Finalize WinINet
    InternetCloseHandle(hConnection);
    InternetCloseHandle(hInet);
    #endif

    return 1;
}

// --------------------------------------------------------------------------
// ARDrone::getVersion(Major version, Minor version, Revision number)
// Description  : Get AR.Drone's version.
// Return value : Version number of the AR.Drone.
// --------------------------------------------------------------------------
int ARDrone::getVersion(int *major, int *minor, int *revision)
{
    if (major) *major = version.major;
    if (minor) *minor = version.minor;
    if (revision) *revision = version.revision;
    return version.major;
}