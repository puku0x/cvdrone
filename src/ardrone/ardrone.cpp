// Copyright(C) 2012 puku0x

#include "ardrone.h"

// --------------------------------------------------------------------------
// ARDrone::ARDrone(IP address of your AR.Drone)
// Description : Constructor of ARDrone class.
// --------------------------------------------------------------------------
ARDrone::ARDrone(const char *ardrone_addr)
{
    // IP Address
    strncpy(ip, ARDRONE_DEFAULT_ADDR, 16);

    // Sequence number
    seq = 1;

    // Camera image
    img = NULL;

    // Timer
    timerWdg = ardGetTickCount();

    // Navdata
    ZeroMemory(&navdata, sizeof(NAVDATA));

    // Thread for Navdata
    flagNavdata   = 0;
    threadNavdata = INVALID_HANDLE_VALUE;
    mutexNavdata  = INVALID_HANDLE_VALUE;

    // Video
    pFormatCtx  = NULL;
    pCodecCtx   = NULL;
    pFrame      = NULL;
    pFrameBGR   = NULL;
    bufferBGR   = NULL;
    pConvertCtx = NULL;

    // Thread for video
    flagVideo   = 0;
    threadVideo = INVALID_HANDLE_VALUE;
    mutexVideo  = INVALID_HANDLE_VALUE;

    // When IP address is specified, open it
    if (ardrone_addr) open(ardrone_addr);
}

// --------------------------------------------------------------------------
// ARDrone::~ARDrone()
// Description : Destructor of ARDrone class.
// --------------------------------------------------------------------------
ARDrone::~ARDrone()
{
    // Finalize the AR.Drone
    close();
}

// --------------------------------------------------------------------------
// ARDrone::open(IP address of AR.Drone)
// Description  : Initialize the AR.Drone.
// Return value : SUCCESS: 1  FAILURE: 0
// --------------------------------------------------------------------------
int ARDrone::open(const char *ardrone_addr)
{
    // Initialize WSA
    WSAData wsaData;
    WSAStartup(MAKEWORD(1,1), &wsaData);

    // Initialize FFmpeg
    av_register_all();
    avformat_network_init();
    av_log_set_level(AV_LOG_QUIET);

    // Save IP address
    strncpy(ip, ardrone_addr, 16);

    // Get version informations
    if (!getVersionInfo()) return 0;
    printf("AR.Drone Ver. %d.%d.%d\n", version.major, version.minor, version.revision);

    // Initialize AT Command
    if (!initCommand()) return 0;

    // Initialize Config
    if (!initConfig()) return 0;

    // Initialize Navdata
    if (!initNavdata()) return 0;

    // Initialize Video
    if (!initVideo()) return 0;

    // Wait for updating state
    Sleep(500);

    // Reset emergency
    resetWatchDog();
    resetEmergency();

    return 1;
}

// --------------------------------------------------------------------------
// ARDrone::update()
// Description  : Update the informations.
// Return value : SUCCESS: 1  FAILURE: 0
// --------------------------------------------------------------------------
int ARDrone::update(void)
{
    // Check threads
    if (!flagVideo) return 0;
    if (!flagNavdata) return 0;

    // Reset Watch-Dog every 100ms
    if (ardGetTickCount() - timerWdg > 100) {
        sockCommand.sendf("AT*COMWDG=%d\r", seq++);
        timerWdg = ardGetTickCount();
    }

    return 1;
}

// --------------------------------------------------------------------------
// ARDrone::close()
// Description  : Finalize
// Return value : NONE
// --------------------------------------------------------------------------
void ARDrone::close(void)
{
    // Stop AR.Drone
    if (!onGround()) landing();

    // Finalize Navdata
    finalizeNavdata();

    // Finalize configuration
    finalizeConfig();

    // Finalize AT command
    finalizeCommand();

    // Finalize video
    finalizeVideo();

    // Finalize WSA
    WSACleanup();
}