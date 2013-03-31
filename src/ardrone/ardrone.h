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

// OpenCV
#include <opencv2/opencv.hpp>

// FFmpeg
extern "C" {
    #include <libavcodec/avcodec.h>
    #include <libavformat/avformat.h>
    #include <libswscale/swscale.h>
}

// POSIX threads
#include <pthread.h>

// Sleep [ms] implementation
#ifdef _WIN32
#include <windows.h>
#define msleep(ms) Sleep((DWORD)ms)
#else
#include <unistd.h>
inline void msleep(unsigned long ms) {
    while (ms--) usleep(1000);
}
#endif

// Fix for MinGW
#ifdef __GNUC__
#define vsprintf_s vsnprintf
#endif

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
#ifndef NULL
#define NULL 0
#endif
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
enum ARDRONE_STATE_MASK {
    ARDRONE_FLY_MASK            = 1 <<  0, // FLY MASK                  : (0) Ardrone is landed, (1) Ardrone is flying
    ARDRONE_VIDEO_MASK          = 1 <<  1, // VIDEO MASK                : (0) Video disable, (1) Video enable
    ARDRONE_VISION_MASK         = 1 <<  2, // VISION MASK               : (0) Vision disable, (1) Vision enable
    ARDRONE_CONTROL_MASK        = 1 <<  3, // CONTROL ALGO              : (0) Euler angles control, (1) Angular speed control
    ARDRONE_ALTITUDE_MASK       = 1 <<  4, // ALTITUDE CONTROL ALGO     : (0) Altitude control inactive (1) Altitude control active
    ARDRONE_USER_FEEDBACK_START = 1 <<  5, // USER feedback             :     Start button state 
    ARDRONE_COMMAND_MASK        = 1 <<  6, // Control command ACK       : (0) None, (1) One received
  //ARDRONE_FW_FILE_MASK        = 1 <<  7, //                           : (1) Firmware file is good
  //ARDRONE_FW_VER_MASK         = 1 <<  8, //                           : (1) Firmware update is newer
  //ARDRONE_FW_UPD_MASK         = 1 <<  9, //                           : (1) Firmware update is ongoing
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
enum ARDRONE_ANIMATION_ID {
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

// TCP Class
class TCPSocket {
public:
    TCPSocket();                            // Constructor
    virtual ~TCPSocket();                   // Destructor
    int  open(const char *addr, int port);  // Initialize
    int  send2(void *data, int size);       // Send data
    int  sendf(char *str, ...);             // Send with format
    int  receive(void *data, int size);     // Receive data
    void close(void);                       // Finalize
private:
    SOCKET sock;                            // Sockets
    sockaddr_in server_addr, client_addr;   // Server/Client IP adrress
};

// UDP Class
class UDPSocket {
public:
    UDPSocket();                            // Constructor
    virtual ~UDPSocket();                   // Destructor
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
struct ARDRONE_NAVDATA {
    // 3x3 matrix
    struct matrix33_t { 
        float m11, m12, m13;
        float m21, m22, m23;
        float m31, m32, m33;
    } ;

    // 3x1 vector
    union vector31_t {
        float v[3];
        struct {
            float x;
            float y;
            float z;
        };
    };

    // Header
    unsigned int header;
    unsigned int ardrone_state;
    unsigned int sequence;
    unsigned int vision_defined;

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
        unsigned int   num_frames;                // Don't use
        matrix33_t     detection_camera_rot;      // Don't use
        vector31_t     detection_camera_trans;    // Don't use
        unsigned int   detection_tag_index;       // Don't use
        unsigned int   detection_camera_type;     // Don't use
        matrix33_t     drone_camera_rot;          // Don't use
        vector31_t     drone_camera_trans;        // Don't use
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

// Configurations
struct ARDRONE_CONFIG {
    struct GENERAL {
        int   num_version_config;
        int   num_version_mb;
        char  num_version_soft[32];
        char  drone_serial[32];
        char  soft_build_date[32];
        float motor1_soft;
        float motor1_hard;
        float motor1_supplier;
        float motor2_soft;
        float motor2_hard;
        float motor2_supplier;
        float motor3_soft;
        float motor3_hard;
        float motor3_supplier;
        float motor4_soft;
        float motor4_hard;
        float motor4_supplier;
        char  ardrone_name[32];
        int   flying_time;
        bool  navdata_demo;
        int   com_watchdog;
        bool  video_enable;
        bool  vision_enable;
        int   vbat_min;
        int   localtime;
        int   navdata_options;
    } general;

    struct CONTROL {
        float accs_offset[3];
        float accs_gains[9];
        float gyros_offset[3];
        float gyros_gains[3];
        float gyros110_offset[2];
        float gyros110_gains[2];
        float magneto_offset[3];
        float magneto_radius;
        float gyro_offset_thr_x;
        float gyro_offset_thr_y;
        float gyro_offset_thr_z;
        int   pwm_ref_gyros;
        int   osctun_value;
        bool  osctun_test;
        int   altitude_max;
        int   altitude_min;
        bool  outdoor;
        bool  flight_without_shell;
        bool  autonomous_flight;
        int   flight_anim[2];
        int   control_level;
        float euler_angle_max;
        float control_iphone_tilt;
        float control_vz_max;
        float control_yaw;
        bool  manual_trim;
        float indoor_euler_angle_max;
        float indoor_control_vz_max;
        float indoor_control_yaw;
        float outdoor_euler_angle_max;
        float outdoor_control_vz_max;
        float outdoor_control_yaw;
        int   flying_mode;
        int   hovering_range;
    } control;

    struct NETWORK {
        char ssid_single_player[32];
        char ssid_multi_player[32];
        int  wifi_mode;
        int  wifi_rate;
        char owner_mac[18];
    } network;

    struct PIC {
        int ultrasound_freq;
        int ultrasound_watchdog;
        int pic_version;
    } pic;

    struct VIDEO {
        int  camif_fps;
        int  camif_buffers;
        int  num_trackers;
        int  video_storage_space;
        bool video_on_usb;
        int  video_file_index;
        int  bitrate;
        int  bitrate_ctrl_mode;
        int  bitrate_storage;
        int  codec_fps;
        int  video_codec;
        int  video_slices;
        int  video_live_socket;
        int  max_bitrate;
        int  video_channel;
    } video;

    struct LEDS {
        int leds_anim[3];
    } leds;

    struct DETECT {
        int enemy_colors;
        int enemy_without_shell;
        int groundstripe_colors;
        int detect_type;
        int detections_select_h;
        int detections_select_v_hsync;
        int detections_select_v;
    } detect;

    struct SYSLOG {
        int output;
        int max_size;
        int nb_files;
    } syslog;

    struct CUSTOM {
        char application_desc[64];
        char profile_desc[64];
        char session_desc[64];
        char application_id[9];
        char profile_id[9];
        char session_id[9];
    } custom;

    struct USERBOX {
        int userbox_cmd;
    } userbox;

    struct GPS {
        float latitude;
        float longitude;
        float altitude;
    } gps;
};

// Version information
struct ARDRONE_VERSION {
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
    virtual int open(const char *ardrone_addr = ARDRONE_DEFAULT_ADDR);

    // Update
    virtual int update(void);

    // Finalize (Automatically called)
    virtual void close(void);

    // Get an image for OpenCV
    virtual IplImage* getImage(void);

    // Get AR.Drone's firmware version
    virtual int getVersion(int *major = NULL, int *minor = NULL, int *revision = NULL);

    // Get sensor values
    virtual double getRoll(void);       // Roll angle  [rad]
    virtual double getPitch(void);      // Pitch angle [rad]
    virtual double getYaw(void);        // Yaw angle   [rad]
    virtual double getAltitude(void);   // Altitude    [m]
    virtual double getVelocity(double *vx = NULL, double *vy = NULL, double *vz = NULL); // Velocity [m/s]

    // Battery charge [%]
    virtual int getBatteryPercentage(void);

    // Take off / Landing / Emergency
    virtual void takeoff(void);
    virtual void landing(void);
    virtual void emergency(void);

    // Move with velocity [m/s]
    virtual void move(double vx, double vy, double vr);
    virtual void move3D(double vx, double vy, double vz, double vr);

    // Change camera channel
    virtual void setCamera(int channel);

    // Others
    virtual int  onGround(void);                            // Check on ground
    virtual void setAnimation(int id, int duration);        // Flight animation
    virtual void setLED(int id, float freq, int duration);  // LED animation
    virtual void setVideoRecord(bool activate);             // Video recording (only for AR.Drone 2.0)
    virtual void setOutdoorMode(bool activate);             // Outdoor mode (experimental)

protected:
    // IP address
    char ip[16];

    // Sequence number
    unsigned long int seq;

    // Camera image
    IplImage *img;

    // Sockets
    UDPSocket sockCommand;
    UDPSocket sockNavdata;
    UDPSocket sockVideo;

    // Version information
    ARDRONE_VERSION version;

    // Navigation data
    ARDRONE_NAVDATA navdata;

    // Configurations
    ARDRONE_CONFIG config;

    // Video
    AVFormatContext *pFormatCtx;
    AVCodecContext  *pCodecCtx;
    AVFrame         *pFrame, *pFrameBGR;
    uint8_t         *bufferBGR;
    SwsContext      *pConvertCtx;

    // Thread for AT command
    pthread_t *threadCommand;
    pthread_mutex_t *mutexCommand;
    virtual void loopCommand(void);
    static void *runCommand(void *args) {
        reinterpret_cast<ARDrone*>(args)->loopCommand();
        return NULL;
    }

    // Thread for Navdata
    pthread_t *threadNavdata;
    pthread_mutex_t *mutexNavdata;
    virtual void loopNavdata(void);
    static void *runNavdata(void *args) {
        reinterpret_cast<ARDrone*>(args)->loopNavdata();
        return NULL;
    }

    // Thread for Video
    pthread_t *threadVideo;
    pthread_mutex_t *mutexVideo;
    virtual void loopVideo(void);
    static void *runVideo(void *args) {
        reinterpret_cast<ARDrone*>(args)->loopVideo();
        return NULL;
    }

    // Initialize (internal)
    virtual int initCommand(void);
    virtual int initNavdata(void);
    virtual int initVideo(void);

    // Get informations (internal)
    virtual int getVersionInfo(void);
    virtual int getConfig(void);
    virtual int getNavdata(void);
    virtual int getVideo(void);

    // Send commands (internal)
    virtual void resetWatchDog(void);
    virtual void resetEmergency(void);

    // Finalize (internal)
    virtual void finalizeCommand(void);
    virtual void finalizeNavdata(void);
    virtual void finalizeVideo(void);
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

    // Show the message
    #ifdef _WIN32
    MessageBox(NULL, str, "CVDRONE ERROR MESSAGE", MB_OK|MB_ICONERROR|MB_TOPMOST|MB_SETFOREGROUND);
    #else
    fprintf(stderr, str);
    #endif
}

#endif