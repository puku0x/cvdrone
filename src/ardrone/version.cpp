// Copyright(C) 2012 puku0x

#include "ardrone.h"

// --------------------------------------------------------------------------
// ARDrone::getVersionInfo()
// Description  : Getting version information.
// Return value : SUCCESS: 1  FAILURE: 0
// --------------------------------------------------------------------------
int ARDrone::getVersionInfo(void)
{
    const char *filename = "version.txt";

    // Initialize WinINet
    HINTERNET hInet = InternetOpen(NULL, INTERNET_OPEN_TYPE_DIRECT, NULL, NULL, 0 );

    // Set timeout [ms]
    DWORD ms = 500;
    InternetSetOption(hInet, INTERNET_OPTION_CONNECT_TIMEOUT, &ms, sizeof(ms));

    // Connect to FTP server
    HINTERNET hConnection = InternetConnect(hInet, ip, ARDRONE_VERSION_PORT, "anonymous", "", INTERNET_SERVICE_FTP, INTERNET_FLAG_PASSIVE, 0);
    if (hConnection == NULL) {
        printf("ERROR: InternetConnect(port=%d) failed. (%s, %d)\n", ARDRONE_VERSION_PORT, __FILE__, __LINE__);
        InternetCloseHandle(hInet);
        return 0;
    }

    // Clear version
    ZeroMemory(&version, sizeof(VERSION_INFO));

    // Get the file
    if (!FtpGetFile(hConnection, filename, filename, FALSE, FILE_ATTRIBUTE_NORMAL, INTERNET_FLAG_TRANSFER_BINARY, 0)) {
        printf("ERROR: FtpGetFile() failed. (%s, %d)\n", __FILE__, __LINE__);
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

    return 1;
}

// --------------------------------------------------------------------------
// ARDrone::getVersion()
// Description  : Getting AR.Drone's version.
// Return value : Version number of the AR.Drone.
// --------------------------------------------------------------------------
int ARDrone::getVersion(void)
{
    return version.major;
}