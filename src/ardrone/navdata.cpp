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
//! @file   navdata.cpp
//! @brief  Navigation data
//
// -------------------------------------------------------------------------

#include "ardrone.h"

// --------------------------------------------------------------------------
//! @brief   Initialize Navdata.
//! @return  Result of initialization
//! @retval  1 Success
//! @retval  0 Failure
// --------------------------------------------------------------------------
int ARDrone::initNavdata(void)
{
    // Open the IP address and port
    if (!sockNavdata.open(ip, ARDRONE_NAVDATA_PORT)) {
        CVDRONE_ERROR("UDPSocket::open(port=%d) was failed. (%s, %d)\n", ARDRONE_NAVDATA_PORT, __FILE__, __LINE__);
        return 0;
    }

    // Clear Navdata
    memset(&navdata, 0, sizeof(navdata));

    // Start Navdata
    sockNavdata.sendf("\x01\x00\x00\x00");

    // AR.Drone 2.0
    if (version.major == ARDRONE_VERSION_2) {
        // Disable BOOTSTRAP mode
        if (mutexCommand) pthread_mutex_lock(mutexCommand);
        sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", ++seq, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
        //sockCommand.sendf("AT*CONFIG=%d,\"general:navdata_demo\",\"TRUE\"\r", ++seq);
        sockCommand.sendf("AT*CONFIG=%d,\"general:navdata_demo\",\"FALSE\"\r", ++seq);
        if (mutexCommand) pthread_mutex_unlock(mutexCommand);
        msleep(100);

        // Seed ACK
        sockCommand.sendf("AT*CTRL=%d,0\r", ++seq);
    }
    // AR.Drone 1.0
    else {
        // Disable BOOTSTRAP mode
        if (mutexCommand) pthread_mutex_lock(mutexCommand);
        //sockCommand.sendf("AT*CONFIG=%d,\"general:navdata_demo\",\"TRUE\"\r", ++seq);
        sockCommand.sendf("AT*CONFIG=%d,\"general:navdata_demo\",\"FALSE\"\r", ++seq);
        if (mutexCommand) pthread_mutex_unlock(mutexCommand);

        // Send ACK
        sockCommand.sendf("AT*CTRL=%d,0\r", ++seq);
    }

    // Create a mutex
    mutexNavdata = new pthread_mutex_t;
    pthread_mutex_init(mutexNavdata, NULL);

    // Create a thread
    threadNavdata = new pthread_t;
    if (pthread_create(threadNavdata, NULL, runNavdata, this) != 0) {
        CVDRONE_ERROR("pthread_create() was failed. (%s, %d)\n", __FILE__, __LINE__);
        return 0;
    }

    return 1;
}

// --------------------------------------------------------------------------
//! @brief   Thread function for Navdata.
//! @return  None
// --------------------------------------------------------------------------
void ARDrone::loopNavdata(void)
{
    while (1) {
        // Get Navdata
        if (!getNavdata()) break;
        pthread_testcancel();
        msleep(10);
    }
}

// --------------------------------------------------------------------------
//! @brief   Get current navigation data of AR.Drone.
//! @return  Result of this function
//! @retval  1 Success
//! @retval  0 Failure
// --------------------------------------------------------------------------
int ARDrone::getNavdata(void)
{
    // Send a request
    sockNavdata.sendf("\x01\x00\x00\x00");

    // Receive data
    char buf[4096] = {'\0'};
    int size = sockNavdata.receive((void*)&buf, sizeof(buf));

    // Received something
    if (size > 0) {
        // Enable mutex lock
        if (mutexNavdata) pthread_mutex_lock(mutexNavdata);

        // Header
        int index = 0;
        memcpy((void*)&(navdata.header),         (const void*)(buf + index), 4); index += 4;
        memcpy((void*)&(navdata.ardrone_state),  (const void*)(buf + index), 4); index += 4;
        memcpy((void*)&(navdata.sequence),       (const void*)(buf + index), 4); index += 4;
        memcpy((void*)&(navdata.vision_defined), (const void*)(buf + index), 4); index += 4;

        // Parse navdata
        while (index < size) {
            // Tag and data size
            unsigned short tmp_tag, tmp_size;
            memcpy((void*)&tmp_tag,  (const void*)(buf + index), 2); index += 2;  // tag
            memcpy((void*)&tmp_size, (const void*)(buf + index), 2); index += 2;  // size
            index -= 4;

            // Copy to NAVDATA structure
            switch (tmp_tag) {
                case ARDRONE_NAVDATA_DEMO_TAG:
                    memcpy((void*)&(navdata.demo),            (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.demo)));
                    break;
                case ARDRONE_NAVDATA_TIME_TAG:
                    memcpy((void*)&(navdata.time),            (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.time)));
                    break;
                case ARDRONE_NAVDATA_RAW_MEASURES_TAG:
                    memcpy((void*)&(navdata.raw_measures),    (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.raw_measures)));
                    break;
                case ARDRONE_NAVDATA_PHYS_MEASURES_TAG:
                    memcpy((void*)&(navdata.phys_measures),   (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.phys_measures)));
                    break;
                case ARDRONE_NAVDATA_GYROS_OFFSETS_TAG:
                    memcpy((void*)&(navdata.gyros_offsets),   (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.gyros_offsets)));
                    break;
                case ARDRONE_NAVDATA_EULER_ANGLES_TAG:
                    memcpy((void*)&(navdata.euler_angles),    (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.euler_angles)));
                    break;
                case ARDRONE_NAVDATA_REFERENCES_TAG:
                    memcpy((void*)&(navdata.references),      (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.references)));
                    break;
                case ARDRONE_NAVDATA_TRIMS_TAG:
                    memcpy((void*)&(navdata.trims),           (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.trims)));
                    break;
                case ARDRONE_NAVDATA_RC_REFERENCES_TAG:
                    memcpy((void*)&(navdata.rc_references),   (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.rc_references)));
                    break;
                case ARDRONE_NAVDATA_PWM_TAG:
                    memcpy((void*)&(navdata.pwm),             (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.pwm)));
                    break;
                case ARDRONE_NAVDATA_ALTITUDE_TAG:
                    memcpy((void*)&(navdata.altitude),        (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.altitude)));
                    break;
                case ARDRONE_NAVDATA_VISION_RAW_TAG:
                    memcpy((void*)&(navdata.vision_raw),      (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.vision_raw)));
                    break;
                case ARDRONE_NAVDATA_VISION_OF_TAG:
                    memcpy((void*)&(navdata.vision_of),       (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.vision_of)));
                    break;
                case ARDRONE_NAVDATA_VISION_TAG:
                    memcpy((void*)&(navdata.vision),          (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.vision)));
                    break;
                case ARDRONE_NAVDATA_VISION_PERF_TAG:
                    memcpy((void*)&(navdata.vision_perf),     (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.vision_perf)));
                    break;
                case ARDRONE_NAVDATA_TRACKERS_SEND_TAG:
                    memcpy((void*)&(navdata.trackers_send),   (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.trackers_send)));
                    break;
                case ARDRONE_NAVDATA_VISION_DETECT_TAG:
                    memcpy((void*)&(navdata.vision_detect),   (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.vision_detect)));
                    break;
                case ARDRONE_NAVDATA_WATCHDOG_TAG:
                    memcpy((void*)&(navdata.watchdog),        (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.watchdog)));
                    break;
                case ARDRONE_NAVDATA_ADC_DATA_FRAME_TAG:
                    memcpy((void*)&(navdata.adc_data_frame),  (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.adc_data_frame)));
                    break;
                case ARDRONE_NAVDATA_VIDEO_STREAM_TAG:
                    memcpy((void*)&(navdata.video_stream),    (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.video_stream)));
                    break;
                case ARDRONE_NAVDATA_GAME_TAG:
                    memcpy((void*)&(navdata.games),           (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.games)));
                    break;
                case ARDRONE_NAVDATA_PRESSURE_RAW_TAG:
                    memcpy((void*)&(navdata.pressure_raw),    (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.pressure_raw)));
                    break;
                case ARDRONE_NAVDATA_MAGNETO_TAG:
                    memcpy((void*)&(navdata.magneto),         (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.magneto)));
                    break;
                case ARDRONE_NAVDATA_WIND_TAG:
                    memcpy((void*)&(navdata.wind),            (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.wind)));
                    break;
                case ARDRONE_NAVDATA_KALMAN_PRESSURE_TAG:
                    memcpy((void*)&(navdata.kalman_pressure), (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.kalman_pressure)));
                    break;
                case ARDRONE_NAVDATA_HDVIDEO_STREAM_TAG:
                    memcpy((void*)&(navdata.hdvideo_stream),  (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.hdvideo_stream)));
                    break;
                case ARDRONE_NAVDATA_WIFI_TAG:
                    memcpy((void*)&(navdata.wifi),            (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.wifi)));
                    break;
                case ARDRONE_NAVDATA_GPS_TAG:
                    if (version.major == 2 && version.minor == 4) memcpy((void*)&(navdata.gps),        (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.gps)));
                    else                                          memcpy((void*)&(navdata.zimmu_3000), (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.zimmu_3000)));
                    break;
                case 28:
                    break;
                case 29:
                    break;
                default:
                    memcpy((void*)&(navdata.cks),             (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.cks)));
                    break;
            }
            index += tmp_size;
        }

        // Disable mutex lock
        if (mutexNavdata) pthread_mutex_unlock(mutexNavdata);
    }

    return 1;
}

// --------------------------------------------------------------------------
//! @brief   Get current role angle of AR.Drone.
//! @return  Role angle [rad]
// --------------------------------------------------------------------------
double ARDrone::getRoll(void)
{
    // Get the data
    if (mutexNavdata) pthread_mutex_lock(mutexNavdata);
    double roll = navdata.demo.phi * 0.001 * DEG_TO_RAD;
    if (mutexNavdata) pthread_mutex_unlock(mutexNavdata);

    return roll;
}

// --------------------------------------------------------------------------
//! @brief   Get current pitch angle of AR.Drone.
//! @return  Pitch angle [rad]
// --------------------------------------------------------------------------
double ARDrone::getPitch(void)
{
    // Get the data
    if (mutexNavdata) pthread_mutex_lock(mutexNavdata);
    double pitch = -navdata.demo.theta * 0.001 * DEG_TO_RAD;
    if (mutexNavdata) pthread_mutex_unlock(mutexNavdata);

    return pitch;
}

// --------------------------------------------------------------------------
//! @brief   Get current yaw angle of AR.Drone.
//! @return  Yaw angle [rad]
// --------------------------------------------------------------------------
double ARDrone::getYaw(void)
{
    // Get the data
    if (mutexNavdata) pthread_mutex_lock(mutexNavdata);
    double yaw = -navdata.demo.psi * 0.001 * DEG_TO_RAD;
    if (mutexNavdata) pthread_mutex_unlock(mutexNavdata);

    return yaw;
}

// --------------------------------------------------------------------------
//! @brief   Get current altitude of AR.Drone.
//! @return  Altitude [m]
// --------------------------------------------------------------------------
double ARDrone::getAltitude(void)
{
    // Get the data
    if (mutexNavdata) pthread_mutex_lock(mutexNavdata);
    double altitude = navdata.demo.altitude * 0.001;
    if (mutexNavdata) pthread_mutex_unlock(mutexNavdata);

    return altitude;
}

// --------------------------------------------------------------------------
//! @brief   Get estimated velocity of AR.Drone.
//! @param   vx A pointer to the X velocity variable [m/s]
//! @param   vy A pointer to the Y velocity variable [m/s]
//! @param   vz A pointer to the Z velocity variable [m/s]
//! @return Velocity [m/s]
// --------------------------------------------------------------------------
double ARDrone::getVelocity(double *vx, double *vy, double *vz)
{
    // Get the data
    if (mutexNavdata) pthread_mutex_lock(mutexNavdata);
    double velocity_x =  navdata.demo.vx * 0.001;
    double velocity_y = -navdata.demo.vy * 0.001;
    //double velocity_z = -navdata.demo.vz * 0.001;
    double velocity_z = -navdata.altitude.altitude_vz * 0.001;
    if (mutexNavdata) pthread_mutex_unlock(mutexNavdata);

    // Velocities
    if (vx) *vx = velocity_x;
    if (vy) *vy = velocity_y;
    if (vz) *vz = velocity_z;

    // Velocity [m/s]
    double velocity = sqrt(velocity_x*velocity_x + velocity_y*velocity_y + velocity_z*velocity_z);
    return velocity;
}

// --------------------------------------------------------------------------
//! @brief   Get GPS position.
//! @note    This function requires AR.Drone2.0 Flight Recorder
//! @param   latitude A pointer to the latitude variable [deg]
//! @param   longitude A pointer to the longitude variable [deg]
//! @param   elevation A pointer to the elevation variable [deg]
//! @return  Result of this function
//! @retval  1 Success
//! @retval  0 Failure
// --------------------------------------------------------------------------
int ARDrone::getPosition(double *latitude, double *longitude, double *elevation)
{
    // Get the data
    if (mutexNavdata) pthread_mutex_lock(mutexNavdata);
    double gps_latitude  = navdata.gps.lat;
    double gps_longitude = navdata.gps.lon;
    double gps_elevation = navdata.gps.elevation;
    int    available     = navdata.gps.data_available;
    if (mutexNavdata) pthread_mutex_unlock(mutexNavdata);

    // Positions
    if (latitude)  *latitude  = gps_latitude;
    if (longitude) *longitude = gps_longitude;
    if (elevation) *elevation = gps_elevation;

    return available;
}

// --------------------------------------------------------------------------
//! @brief   Get current battery percentage of AR.Drone.
//! @return  Battery percentage [%]
// --------------------------------------------------------------------------
int ARDrone::getBatteryPercentage(void)
{
    // Get the data
    if (mutexNavdata) pthread_mutex_lock(mutexNavdata);
    int battery = navdata.demo.vbat_flying_percentage;
    if (mutexNavdata) pthread_mutex_unlock(mutexNavdata);

    return battery;
}

// --------------------------------------------------------------------------
//! @brief   Check whether AR.Drone is on ground.
//! @return  Result of this function
//! @retval  1 Yes
//! @retval  0 No
// --------------------------------------------------------------------------
int ARDrone::onGround(void)
{
    // Get the data
    if (mutexNavdata) pthread_mutex_lock(mutexNavdata);
    int on_ground = (navdata.ardrone_state & ARDRONE_FLY_MASK) ? 0 : 1;
    if (mutexNavdata) pthread_mutex_unlock(mutexNavdata);

    return on_ground;
}

// --------------------------------------------------------------------------
//! @brief   Finalize Navdata.
//! @return  None
// --------------------------------------------------------------------------
void ARDrone::finalizeNavdata(void)
{
    // Destroy the thread
    if (threadNavdata) {
        pthread_cancel(*threadNavdata);
        pthread_join(*threadNavdata, NULL);
        delete threadNavdata;
        threadNavdata = NULL;
    }

    // Delete the mutex
    if (mutexNavdata) {
        pthread_mutex_destroy(mutexNavdata);
        delete mutexNavdata;
        mutexNavdata = NULL;
    }

    // Close the socket
    sockNavdata.close();
}