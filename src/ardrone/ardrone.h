#ifndef __HEADER_ARDRONE_LIB__
#define __HEADER_ARDRONE_LIB__

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

// Coordinate system
//   Front of the AR.Drone is X-axis, left is Y-axis, upper is Z-axis.
//   Also front is 0.0 [rad], each axis CCW is positive.
//            X
//           +^-
//            |
//            |
//    Y <-----+ (0,0)
//            Z
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// Win32API
#include <windows.h>

// Thread
#include <process.h>

// WinSock
#include <winsock.h>
#pragma comment(lib, "wsock32.lib")

// WinINet
#include <wininet.h>
#pragma comment(lib, "wininet.lib")

// FFmpeg
extern "C" {
#include <libavcodec/avcodec.h>
    #include <libavformat/avformat.h>
    #include <libswscale/swscale.h>
}
#pragma comment(lib, "avcodec.lib")
#pragma comment(lib, "avdevice.lib")
#pragma comment(lib, "avfilter.lib")
#pragma comment(lib, "avformat.lib")
#pragma comment(lib, "avutil.lib")
#pragma comment(lib, "swresample.lib")
#pragma comment(lib, "swscale.lib")

// OpenCV
#include <opencv2/opencv.hpp>
#pragma comment(lib, "opencv_calib3d249.lib")
#pragma comment(lib, "opencv_contrib249.lib")
#pragma comment(lib, "opencv_core249.lib")
#pragma comment(lib, "opencv_features2d249.lib")
#pragma comment(lib, "opencv_flann249.lib")
#pragma comment(lib, "opencv_highgui249.lib")
#pragma comment(lib, "opencv_imgproc249.lib")
#pragma comment(lib, "opencv_legacy249.lib")
#pragma comment(lib, "opencv_ml249.lib")
#pragma comment(lib, "opencv_objdetect249.lib")
#pragma comment(lib, "opencv_photo249.lib")
#pragma comment(lib, "opencv_stitching249.lib")
#pragma comment(lib, "opencv_video249.lib")
#pragma comment(lib, "opencv_videostab249.lib")

// Macro definitions
#define ARDRONE_VERSION_1           (1)             // AR.Drone 1.0
#define ARDRONE_VERSION_2           (2)             // AR.Drone 2.0
#define ARDRONE_SESSION_ID          "d2e081a3"      // SessionID
#define ARDRONE_PROFILE_ID          "be27e2e4"      // Profile ID
#define ARDRONE_APPLOCATION_ID      "d87f7e0c"      // Application ID
#define ARDRONE_VERSION_PORT        (5551)          // Port number for FTP
#define ARDRONE_NAVDATA_PORT        (5554)          // Port number for Navdata
#define ARDRONE_VIDEO_PORT          (5555)          // Port number for Video
#define ARDRONE_COMMAND_PORT        (5556)          // Port number for AT command
#define ARDRONE_CONFIG_PORT         (5559)          // Port number for configuration
#define ARDRONE_DEFAULT_ADDR        "192.168.1.1"   // Default IP address of AR.Drone
#define ARDRONE_NAVDATA_HEADER      (0x55667788)    // Header of Navdata

// Math constants
#ifndef M_PI
#define M_PI  (3.14159265358979323846264338327)
#endif
#ifndef RAD_TO_DEG
#define RAD_TO_DEG (180/M_PI)
#endif
#ifndef DEG_TO_RAD
#define DEG_TO_RAD (M_PI/180)
#endif

// State masks
typedef enum ARDRONE_STATE_MASK {
    ARDRONE_FLY_MASK            = 1 <<  0, // FLY MASK                  : (0) Ardrone is landed, (1) Ardrone is flying
    ARDRONE_VIDEO_MASK          = 1 <<  1, // VIDEO MASK                : (0) Video disable, (1) Video enable
    ARDRONE_VISION_MASK         = 1 <<  2, // VISION MASK               : (0) Vision disable, (1) Vision enable
    ARDRONE_CONTROL_MASK        = 1 <<  3, // CONTROL ALGO              : (0) Euler angles control, (1) Angular speed control
    ARDRONE_ALTITUDE_MASK       = 1 <<  4, // ALTITUDE CONTROL ALGO     : (0) Altitude control inactive (1) Altitude control active
    ARDRONE_USER_FEEDBACK_START = 1 <<  5, // USER feedback             :     Start button state 
    ARDRONE_COMMAND_MASK        = 1 <<  6, // Control command ACK       : (0) None, (1) One received
//  ARDRONE_FW_FILE_MASK        = 1 <<  7, //                           : (1) Firmware file is good
//  ARDRONE_FW_VER_MASK         = 1 <<  8, //                           : (1) Firmware update is newer
//  ARDRONE_FW_UPD_MASK         = 1 <<  9, //                           : (1) Firmware update is ongoing
    ARDRONE_NAVDATA_DEMO_MASK   = 1 << 10, // Navdata demo              : (0) All navdata, (1) Only navdata demo
    ARDRONE_NAVDATA_BOOTSTRAP   = 1 << 11, // Navdata bootstrap         : (0) Options sent in all or demo mode, (1) No navdata options sent
    ARDRONE_MOTORS_MASK         = 1 << 12, // Motors status             : (0) Ok, (1) Motors problem
    ARDRONE_COM_LOST_MASK       = 1 << 13, // Communication Lost        : (1) Com problem, (0) Com is ok
    ARDRONE_VBAT_LOW            = 1 << 15, // VBat low                  : (1) Too low, (0) Ok
    ARDRONE_USER_EL             = 1 << 16, // User Emergency Landing    : (1) User EL is ON, (0) User EL is OFF
    ARDRONE_TIMER_ELAPSED       = 1 << 17, // Timer elapsed             : (1) Elapsed, (0) Not elapsed
    ARDRONE_ANGLES_OUT_OF_RANGE = 1 << 19, // Angles                    : (0) Ok, (1) Out of range
    ARDRONE_ULTRASOUND_MASK     = 1 << 21, // Ultrasonic sensor         : (0) Ok, (1) Deaf
    ARDRONE_CUTOUT_MASK         = 1 << 22, // Cutout system detection   : (0) Not detected, (1) Detected
    ARDRONE_PIC_VERSION_MASK    = 1 << 23, // PIC Version number OK     : (0) A bad version number, (1) Version number is OK */
    ARDRONE_ATCODEC_THREAD_ON   = 1 << 24, // ATCodec thread ON         : (0) Thread OFF (1) thread ON
    ARDRONE_NAVDATA_THREAD_ON   = 1 << 25, // Navdata thread ON         : (0) Thread OFF (1) thread ON
    ARDRONE_VIDEO_THREAD_ON     = 1 << 26, // Video thread ON           : (0) Thread OFF (1) thread ON
    ARDRONE_ACQ_THREAD_ON       = 1 << 27, // Acquisition thread ON     : (0) Thread OFF (1) thread ON
    ARDRONE_CTRL_WATCHDOG_MASK  = 1 << 28, // CTRL watchdog             : (1) Delay in control execution (> 5ms), (0) Control is well scheduled
    ARDRONE_ADC_WATCHDOG_MASK   = 1 << 29, // ADC Watchdog              : (1) Delay in uart2 dsr (> 5ms), (0) Uart2 is good
    ARDRONE_COM_WATCHDOG_MASK   = 1 << 30, // Communication Watchdog    : (1) Com problem, (0) Com is ok
    ARDRONE_EMERGENCY_MASK      = 1 << 31  // Emergency landing         : (0) No emergency, (1) Emergency
};

// Flight animation IDs
typedef enum ARDRONE_ANIMATION_ID {
    ARDRONE_ANIM_PHI_M30_DEG = 0,
    ARDRONE_ANIM_PHI_30_DEG,
    ARDRONE_ANIM_THETA_M30_DEG,
    ARDRONE_ANIM_THETA_30_DEG,
    ARDRONE_ANIM_THETA_20DEG_YAW_200DEG,
    ARDRONE_ANIM_THETA_20DEG_YAW_M200DEG,
    ARDRONE_ANIM_TURNAROUND,
    ARDRONE_ANIM_TURNAROUND_GODOWN,
    ARDRONE_ANIM_YAW_SHAKE,
    ARDRONE_ANIM_YAW_DANCE,
    ARDRONE_ANIM_PHI_DANCE,
    ARDRONE_ANIM_THETA_DANCE,
    ARDRONE_ANIM_VZ_DANCE,
    ARDRONE_ANIM_WAVE,
    ARDRONE_ANIM_PHI_THETA_MIXED,
    ARDRONE_ANIM_DOUBLE_PHI_THETA_MIXED,
    ARDRONE_NB_ANIM_MAYDAY
};

// LED animation IDs
enum ARDRONE_LED_ANIMATION_ID {
    BLINK_GREEN_RED = 0,
    BLINK_GREEN,
    BLINK_RED,
    BLINK_ORANGE,
    SNAKE_GREEN_RED,
    FIRE,
    STANDARD,
    RED,
    GREEN,
    RED_SNAKE,
    BLANK,
    RIGHT_MISSILE,
    LEFT_MISSILE,
    DOUBLE_MISSILE,
    FRONT_LEFT_GREEN_OTHERS_RED,
    FRONT_RIGHT_GREEN_OTHERS_RED,
    REAR_RIGHT_GREEN_OTHERS_RED,
    REAR_LEFT_GREEN_OTHERS_RED,
    LEFT_GREEN_RIGHT_RED,
    LEFT_RED_RIGHT_GREEN,
    BLINK_STANDARD
};

// UDP Class
class UDPSocket {
public:
    UDPSocket();                            // Constructor
    ~UDPSocket();                           // Destructor
    int  open(const char *addr, int port);  // Initialize
    int  send2(void *data, int size);       // Send data
    int  sendf(char *str, ...);             // Send with format
    int  receive(void *data, int size);     // Receive data
    void close(void);                       // Finalize
private:
    SOCKET sock;                            // Sockets
    sockaddr_in server_addr, client_addr;   // Server/Client IP adrress
};

// Navdata
#pragma pack(push, 1)
struct NAVDATA {
    // 3x3 matrix
    struct matrix33_t
    { 
        float m11, m12, m13;
        float m21, m22, m23;
        float m31, m32, m33;
    } ;

    // 3x1 vector
    union vector31_t {
        float v[3];
        struct    
        {
            float x;
            float y;
            float z;
        };
    };

    // Header
    unsigned int   header;
    unsigned int   ardrone_state;
    unsigned int   sequence;
    unsigned int   vision_defined;

    // Demo
    struct NAVDATA_DEMO {
        unsigned short tag;
        unsigned short size;
        unsigned int   ctrl_state;
        unsigned int   vbat_flying_percentage;
        float          theta;
        float          phi;
        float          psi;
        int            altitude;
        float          vx;
        float          vy;
        float          vz;

        // Streamed frame index
        unsigned int num_frames;                // Don't use

        // Camera parameters compute by detection
        matrix33_t   detection_camera_rot;      // Don't use
        vector31_t   detection_camera_trans;    // Don't use
        unsigned int detection_tag_index;       // Don't use

        // Type of tag searched in detection
        unsigned int detection_camera_type;     // Don't use

        // Camera parameters compute by drone
        matrix33_t drone_camera_rot;            // Don't use
        vector31_t drone_camera_trans;          // Don't use
    } demo;

    // Vision
    struct NAVDATA_VISION {
        unsigned short tag;
        unsigned short size;
        unsigned int   nb_detected;
        unsigned int   type[4];
        unsigned int   xc[4];
        unsigned int   yc[4];
        unsigned int   width[4];
        unsigned int   height[4];
        unsigned int   dist[4];
        float          orientation_angle[4];
        matrix33_t     rotation[4];
        vector31_t     translation[4];
        unsigned int   camera_source[4];
    } vision;
};
#pragma pack(pop)

// Version information
struct VERSION_INFO {
    int major;
    int minor;
    int revision;
};

// AR.Drone class
class ARDrone {
public:
    // Constructor / Destructor
    ARDrone(const char *ardrone_addr = NULL);
    virtual ~ARDrone();

    // Initialize
    int open(const char *ardrone_addr = ARDRONE_DEFAULT_ADDR);

    // Update (Call this function in each loop)
    int update(void);

    // Finalize (Automatically called)
    void close(void);

    // Get an image for OpenCV
    IplImage* getImage(void);

    // Get AR.Drone's firmware version
    int getVersion(void);

    // Get sensor values
    double getRoll(void);       // Roll angle  [rad]
    double getPitch(void);      // Pitch angle [rad]
    double getYaw(void);        // Yaw angle   [rad]
    double getAltitude(void);   // Altitude    [m]
    double getVelocity(double *vx = NULL, double *vy = NULL, double *vz = NULL); // Velocity [m/s]

    // Battery charge [%]
    int getBatteryPercentage(void);

    // Take off / Landing / Emergency
    void takeoff(void);
    void landing(void);
    void emergency(void);

    // Move with velocity [m/s]
    void move(double vx, double vy, double vr);
    void move3D(double vx, double vy, double vz, double vr);

    // Change camera channel
    void setCamera(int channel);

    // Others
    int  onGround(void);                            // Check on ground
    void setAnimation(int id, int duration);        // Flight animation
    void setLED(int id, float freq, int duration);  // LED animation
    void startVideoRecord(void);                    // Video recording for AR.Drone 2.0
    void stopVideoRecord(void);                     // You should set a USB key with > 100MB to your drone

protected:
    // IP address
    char ip[16];

    // Sequence number
    long int seq;

    // Camera image
    IplImage *img;

    // Timer
    int64 timer;

    // Sockets
    UDPSocket sockNavdata;
    UDPSocket sockVideo;
    UDPSocket sockCommand;

    // Version information
    VERSION_INFO version;

    // Navigation data
    NAVDATA navdata;

    // Thread for Navdata
    int    flagNavdata;
    HANDLE threadNavdata;
    HANDLE mutexNavdata;
    UINT   loopNavdata(void);
    static UINT WINAPI runNavdata(void *args) {
        return reinterpret_cast<ARDrone*>(args)->loopNavdata();
    }

    // Video
    AVFormatContext *pFormatCtx;
    AVCodecContext  *pCodecCtx;
    AVFrame         *pFrame, *pFrameBGR;
    uint8_t         *bufferBGR;
    SwsContext      *pConvertCtx;

    // Thread for video
    int    flagVideo;
    HANDLE threadVideo;
    HANDLE mutexVideo;
    UINT   loopVideo(void);
    static UINT WINAPI runVideo(void *args) {
        return reinterpret_cast<ARDrone*>(args)->loopVideo();
    }

    // Initialize (internal)
    int initNavdata(void);
    int initVideo(void);
    int initCommand(void);
    int initConfig(void);

    // Get informations (internal)
    int getVersionInfo(void);
    int getNavdata(void);
    int getVideo(void);
    int getConfig(void);

    // Send commands (internal)
    void resetWatchDog(void);
    void resetEmergency(void);

    // Finalize (internal)
    void finalizeNavdata(void);
    void finalizeVideo(void);
    void finalizeCommand(void);
    void finalizeConfig(void);
};

// --------------------------------------------------------------------------
// CVDRONE_ERROR(Message)
// Description  : Show an error message.
// Return value : NONE
// --------------------------------------------------------------------------
CV_INLINE void CVDRONE_ERROR(const char *message, ...)
{
    char *arg;
    char str[256];

    // Apply format
    va_start(arg, message);
    vsprintf(str, message, arg);
    va_end(arg);

    // Show message box
    MessageBox(NULL, str, "CVDRONE ERROR MESSAGE", MB_OK|MB_ICONERROR|MB_TOPMOST|MB_SETFOREGROUND);
}

#endif