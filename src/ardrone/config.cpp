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
//! @file   config.cpp
//! @brief  A source file of AR.Drone class
//
// -------------------------------------------------------------------------

#include "ardrone.h"

// --------------------------------------------------------------------------
//! @brief   Parse a configuration string.
//! @param   str Configuration string
//! @param   config Configuration struct
//! @return  None
// --------------------------------------------------------------------------
void parse(const char *str, ARDRONE_CONFIG *config)
{
    // Split key and value
    char category[256] = {'\0'}, key[256] = {'\0'}, val[256] = {'\0'};
    sscanf(str, "%[^:]:%s = %[^\n]", category, key, val);
    //printf("category = %s, key = %s, val = %s\n", category, key, val);

    // Parse the values
    if (!(strcmp(category, "general"))) {
        if      (!strcmp(key, "num_version_config")) sscanf(val, "%d", &(config->general.num_version_config));
        else if (!strcmp(key, "num_version_mb"))     sscanf(val, "%d", &(config->general.num_version_mb));
        else if (!strcmp(key, "num_version_soft"))   strncpy(config->general.num_version_soft, val, 32);
        else if (!strcmp(key, "drone_serial"))       strncpy(config->general.drone_serial, val, 32);
        else if (!strcmp(key, "soft_build_date"))    strncpy(config->general.soft_build_date, val, 32);
        else if (!strcmp(key, "motor1_soft"))        sscanf(val, "%f", &(config->general.motor1_soft));
        else if (!strcmp(key, "motor1_hard"))        sscanf(val, "%f", &(config->general.motor1_hard));
        else if (!strcmp(key, "motor1_supplier"))    sscanf(val, "%f", &(config->general.motor1_supplier));
        else if (!strcmp(key, "motor2_soft"))        sscanf(val, "%f", &(config->general.motor2_soft));
        else if (!strcmp(key, "motor2_hard"))        sscanf(val, "%f", &(config->general.motor2_hard));
        else if (!strcmp(key, "motor2_supplier"))    sscanf(val, "%f", &(config->general.motor2_supplier));
        else if (!strcmp(key, "motor3_soft"))        sscanf(val, "%f", &(config->general.motor3_soft));
        else if (!strcmp(key, "motor3_hard"))        sscanf(val, "%f", &(config->general.motor3_hard));
        else if (!strcmp(key, "motor3_supplier"))    sscanf(val, "%f", &(config->general.motor3_supplier));
        else if (!strcmp(key, "motor4_soft"))        sscanf(val, "%f", &(config->general.motor4_soft));
        else if (!strcmp(key, "motor4_hard"))        sscanf(val, "%f", &(config->general.motor4_hard));
        else if (!strcmp(key, "motor4_supplier"))    sscanf(val, "%f", &(config->general.motor4_supplier));
        else if (!strcmp(key, "ardrone_name"))       strncpy(config->general.ardrone_name, val, 32);
        else if (!strcmp(key, "flying_time"))        sscanf(val, "%d", &(config->general.flying_time));
        else if (!strcmp(key, "navdata_demo"))       config->general.navdata_demo = (!strcmp(val, "TRUE")) ? true : false;
        else if (!strcmp(key, "com_watchdog"))       sscanf(val, "%d", &(config->general.com_watchdog));
        else if (!strcmp(key, "video_enable"))       config->general.video_enable = (!strcmp(val, "TRUE")) ? true : false;
        else if (!strcmp(key, "vision_enable"))      config->general.vision_enable = (!strcmp(val, "TRUE")) ? true : false;
        else if (!strcmp(key, "vbat_min"))           sscanf(val, "%d", &(config->general.vbat_min));
        else if (!strcmp(key, "localtime"))          sscanf(val, "%d", &(config->general.localtime));
        else if (!strcmp(key, "navdata_options"))    sscanf(val, "%d", &(config->general.navdata_options));
        else if (!strcmp(key, "gps_soft"))           sscanf(val, "%f", &(config->general.motor1_soft));
        else if (!strcmp(key, "gps_hard"))           sscanf(val, "%f", &(config->general.motor1_hard));
        else if (!strcmp(key, "localtime_zone"))     strncpy(config->general.localtime_zone, val, 32);
        else if (!strcmp(key, "timezone"))           strncpy(config->general.timezone, val, 32);
        else if (!strcmp(key, "battery_type"))       sscanf(val, "%d", &(config->general.battery_type));
    }
    else if (!(strcmp(category, "control"))) {
        if      (!strcmp(key, "accs_offset"))             sscanf(val, "{ %f %f %f }", &(config->control.accs_offset[0]), &(config->control.accs_offset[1]), &(config->control.accs_offset[2]));
        else if (!strcmp(key, "accs_gains"))              sscanf(val, "{ %f %f %f %f %f %f %f %f %f }", &(config->control.accs_gains[0]), &(config->control.accs_gains[1]), &(config->control.accs_gains[2]), &(config->control.accs_gains[3]), &(config->control.accs_gains[4]), &(config->control.accs_gains[5]), &(config->control.accs_gains[6]), &(config->control.accs_gains[7]), &(config->control.accs_gains[8]));
        else if (!strcmp(key, "gyros_offset"))            sscanf(val, "{ %f %f %f }", &(config->control.gyros_offset[0]), &(config->control.gyros_offset[1]), &(config->control.gyros_offset[2]));
        else if (!strcmp(key, "gyros_gains"))             sscanf(val, "{ %f %f %f }", &(config->control.gyros_gains[0]), &(config->control.gyros_gains[1]), &(config->control.gyros_gains[2]));
        else if (!strcmp(key, "gyros110_offset"))         sscanf(val, "{ %f %f }", &(config->control.gyros110_offset[0]), &(config->control.gyros110_offset[1]));
        else if (!strcmp(key, "gyros110_gains"))          sscanf(val, "{ %f %f }", &(config->control.gyros110_gains[0]), &(config->control.gyros110_gains[1]));
        else if (!strcmp(key, "magneto_offset"))          sscanf(val, "{ %f %f %f }", &(config->control.magneto_offset[0]), &(config->control.magneto_offset[1]), &(config->control.magneto_offset[2]));
        else if (!strcmp(key, "magneto_radius"))          sscanf(val, "%f", &(config->control.magneto_radius));
        else if (!strcmp(key, "gyro_offset_thr_x"))       sscanf(val, "%f", &(config->control.gyro_offset_thr_x));
        else if (!strcmp(key, "gyro_offset_thr_y"))       sscanf(val, "%f", &(config->control.gyro_offset_thr_y));
        else if (!strcmp(key, "gyro_offset_thr_z"))       sscanf(val, "%f", &(config->control.gyro_offset_thr_z));
        else if (!strcmp(key, "pwm_ref_gyros"))           sscanf(val, "%d", &(config->control.pwm_ref_gyros));
        else if (!strcmp(key, "osctun_value"))            sscanf(val, "%d", &(config->control.osctun_value));
        else if (!strcmp(key, "osctun_test"))             config->control.osctun_test = (!strcmp(val, "TRUE")) ? true : false;
        else if (!strcmp(key, "altitude_max"))            sscanf(val, "%d", &(config->control.altitude_max));
        else if (!strcmp(key, "altitude_min"))            sscanf(val, "%d", &(config->control.altitude_min));
        else if (!strcmp(key, "outdoor"))                 config->control.outdoor = (!strcmp(val, "TRUE")) ? true : false;
        else if (!strcmp(key, "flight_without_shell"))    config->control.flight_without_shell = (!strcmp(val, "TRUE")) ? true : false;
        else if (!strcmp(key, "autonomous_flight"))       config->control.autonomous_flight = (!strcmp(val, "TRUE")) ? true : false;
        else if (!strcmp(key, "flight_anim"))             sscanf(val, "%d,%d", &(config->control.flight_anim[0]), &(config->control.flight_anim[1]));
        else if (!strcmp(key, "control_level"))           sscanf(val, "%d", &(config->control.control_level));
        else if (!strcmp(key, "euler_angle_max"))         sscanf(val, "%f", &(config->control.euler_angle_max));
        else if (!strcmp(key, "control_iphone_tilt"))     sscanf(val, "%f", &(config->control.control_iphone_tilt));
        else if (!strcmp(key, "control_vz_max"))          sscanf(val, "%f", &(config->control.control_vz_max));
        else if (!strcmp(key, "control_yaw"))             sscanf(val, "%f", &(config->control.control_yaw));
        else if (!strcmp(key, "manual_trim"))             config->control.manual_trim = (!strcmp(val, "TRUE")) ? true : false;
        else if (!strcmp(key, "indoor_euler_angle_max"))  sscanf(val, "%f", &(config->control.indoor_euler_angle_max));
        else if (!strcmp(key, "indoor_control_vz_max"))   sscanf(val, "%f", &(config->control.indoor_control_vz_max));
        else if (!strcmp(key, "indoor_control_yaw"))      sscanf(val, "%f", &(config->control.indoor_control_yaw));
        else if (!strcmp(key, "outdoor_euler_angle_max")) sscanf(val, "%f", &(config->control.outdoor_euler_angle_max));
        else if (!strcmp(key, "outdoor_control_vz_max"))  sscanf(val, "%f", &(config->control.outdoor_control_vz_max));
        else if (!strcmp(key, "outdoor_control_yaw"))     sscanf(val, "%f", &(config->control.outdoor_control_yaw));
        else if (!strcmp(key, "flying_mode"))             sscanf(val, "%d", &(config->control.flying_mode));
        else if (!strcmp(key, "hovering_range"))          sscanf(val, "%d", &(config->control.hovering_range));
        else if (!strcmp(key, "flying_camera_mode"))      sscanf(val, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", &(config->control.flying_camera_mode[0]), &(config->control.flying_camera_mode[1]), &(config->control.flying_camera_mode[2]), &(config->control.flying_camera_mode[3]), &(config->control.flying_camera_mode[4]), &(config->control.flying_camera_mode[5]), &(config->control.flying_camera_mode[6]), &(config->control.flying_camera_mode[7]), &(config->control.flying_camera_mode[8]), &(config->control.flying_camera_mode[9]));
        else if (!strcmp(key, "flying_camera_enable"))    config->control.flying_camera_enable = (!strcmp(val, "TRUE")) ? true : false;
    }
    else if (!(strcmp(category, "network"))) {
        if      (!strcmp(key, "ssid_single_player")) strncpy(config->network.ssid_single_player, val, 32);
        else if (!strcmp(key, "ssid_multi_player"))  strncpy(config->network.ssid_multi_player, val, 32);
        else if (!strcmp(key, "wifi_mode"))          sscanf(val, "%d", &(config->network.wifi_mode));
        else if (!strcmp(key, "wifi_rate"))          sscanf(val, "%d", &(config->network.wifi_rate));
        else if (!strcmp(key, "owner_mac"))          strncpy(config->network.owner_mac, val, 18);
    }
    else if (!(strcmp(category, "pic"))) {
        if      (!strcmp(key, "ultrasound_freq"))     sscanf(val, "%d", &(config->pic.ultrasound_freq));
        else if (!strcmp(key, "ultrasound_watchdog")) sscanf(val, "%d", &(config->pic.ultrasound_watchdog));
        else if (!strcmp(key, "pic_version"))         sscanf(val, "%d", &(config->pic.pic_version));
    }
    else if (!(strcmp(category, "video"))) {
        if      (!strcmp(key, "camif_fps"))           sscanf(val, "%d", &(config->video.camif_fps));
        else if (!strcmp(key, "camif_buffers"))       sscanf(val, "%d", &(config->video.camif_buffers));
        else if (!strcmp(key, "num_trackers"))        sscanf(val, "%d", &(config->video.num_trackers));
        else if (!strcmp(key, "video_storage_space")) sscanf(val, "%d", &(config->video.video_storage_space));
        else if (!strcmp(key, "video_on_usb"))        config->video.video_on_usb = (!strcmp(val, "TRUE")) ? true : false;
        else if (!strcmp(key, "video_file_index"))    sscanf(val, "%d", &(config->video.video_file_index));
        else if (!strcmp(key, "bitrate"))             sscanf(val, "%d", &(config->video.bitrate));
        else if (!strcmp(key, "bitrate_ctrl_mode"))   sscanf(val, "%d", &(config->video.bitrate_ctrl_mode));
        else if (!strcmp(key, "bitrate_storage"))     sscanf(val, "%d", &(config->video.bitrate_storage));
        else if (!strcmp(key, "codec_fps"))           sscanf(val, "%d", &(config->video.codec_fps));
        else if (!strcmp(key, "video_codec"))         sscanf(val, "%d", &(config->video.video_codec));
        else if (!strcmp(key, "video_slices"))        sscanf(val, "%d", &(config->video.video_slices));
        else if (!strcmp(key, "video_live_socket"))   sscanf(val, "%d", &(config->video.video_live_socket));
        else if (!strcmp(key, "max_bitrate"))         sscanf(val, "%d", &(config->video.max_bitrate));
        else if (!strcmp(key, "video_channel"))       sscanf(val, "%d", &(config->video.video_channel));
        else if (!strcmp(key, "exposure_mode"))       sscanf(val, "%d,%d,%d,%d", &(config->video.exposure_mode[0]), &(config->video.exposure_mode[1]), &(config->video.exposure_mode[2]), &(config->video.exposure_mode[3]));
        else if (!strcmp(key, "saturation_mode"))     sscanf(val, "%d", &(config->video.saturation_mode));
        else if (!strcmp(key, "whitebalance_mode"))   sscanf(val, "%d,%d", &(config->video.whitebalance_mode[0]), &(config->video.whitebalance_mode[1]));
    }
    else if (!(strcmp(category, "leds"))) {
        if (!strcmp(key, "leds_anim")) sscanf(val, "%d,%d,%d", &(config->leds.leds_anim[0]), &(config->leds.leds_anim[1]), &(config->leds.leds_anim[2]));
    }
    else if (!(strcmp(category, "detect"))) {
        if      (!strcmp(key, "enemy_colors"))              sscanf(val, "%d", &(config->detect.enemy_colors));
        else if (!strcmp(key, "enemy_without_shell"))       sscanf(val, "%d", &(config->detect.enemy_without_shell));
        else if (!strcmp(key, "groundstripe_colors"))       sscanf(val, "%d", &(config->detect.groundstripe_colors));
        else if (!strcmp(key, "detect_type"))               sscanf(val, "%d", &(config->detect.detect_type));
        else if (!strcmp(key, "detections_select_h"))       sscanf(val, "%d", &(config->detect.detections_select_h));
        else if (!strcmp(key, "detections_select_v_hsync")) sscanf(val, "%d", &(config->detect.detections_select_v_hsync));
        else if (!strcmp(key, "detections_select_v"))       sscanf(val, "%d", &(config->detect.detections_select_v));
    }
    else if (!(strcmp(category, "syslog"))) {
        if      (!strcmp(key, "output"))   sscanf(val, "%d", &(config->syslog.output));
        else if (!strcmp(key, "max_size")) sscanf(val, "%d", &(config->syslog.max_size));
        else if (!strcmp(key, "nb_files")) sscanf(val, "%d", &(config->syslog.nb_files));
    }
    else if (!(strcmp(category, "custom"))) {
        if      (!strcmp(key, "application_desc")) strncpy(config->custom.application_desc, val, 64);
        else if (!strcmp(key, "profile_desc"))     strncpy(config->custom.profile_desc, val, 64);
        else if (!strcmp(key, "session_desc"))     strncpy(config->custom.session_desc, val, 64);
        else if (!strcmp(key, "application_id"))   strncpy(config->custom.application_id, val, 8);
        else if (!strcmp(key, "profile_id"))       strncpy(config->custom.profile_id, val, 8);
        else if (!strcmp(key, "session_id"))       strncpy(config->custom.session_id, val, 8);
    }
    else if (!(strcmp(category, "userbox"))) {
        if (!strcmp(key, "userbox_cmd")) sscanf(val, "%d", &(config->userbox.userbox_cmd));
    }
    else if (!(strcmp(category, "gps"))) {
        if      (!strcmp(key, "latitude"))  sscanf(val, "%f", &(config->gps.latitude));
        else if (!strcmp(key, "longitude")) sscanf(val, "%f", &(config->gps.longitude));
        else if (!strcmp(key, "altitude"))  sscanf(val, "%f", &(config->gps.altitude));
        else if (!strcmp(key, "accuracy"))  sscanf(val, "%f", &(config->gps.accuracy));
    }
    else if (!(strcmp(category, "flightplan"))) {
        if      (!strcmp(key, "default_validation_radius")) sscanf(val, "%f", &(config->flightplan.default_validation_radius));
        else if (!strcmp(key, "default_validation_time"))   sscanf(val, "%f", &(config->flightplan.default_validation_time));
        else if (!strcmp(key, "max_distance_from_takeoff")) sscanf(val, "%d", &(config->flightplan.max_distance_from_takeoff));
        else if (!strcmp(key, "gcs_ip"))                    sscanf(val, "%d", &(config->flightplan.gcs_ip));
        else if (!strcmp(key, "video_stop_delay"))          sscanf(val, "%d", &(config->flightplan.video_stop_delay));
        else if (!strcmp(key, "low_battery_go_home"))       config->flightplan.low_battery_go_home = (!strcmp(val, "TRUE")) ? true : false;
        else if (!strcmp(key, "automatic_heading"))         config->flightplan.automatic_heading = (!strcmp(val, "TRUE")) ? true : false;
        else if (!strcmp(key, "com_lost_action_delay"))     sscanf(val, "%d", &(config->flightplan.com_lost_action_delay));
        else if (!strcmp(key, "altitude_go_home"))          sscanf(val, "%d", &(config->flightplan.altitude_go_home));
        else if (!strcmp(key, "mavlink_js_roll_left"))      strncpy(config->flightplan.mavlink_js_roll_left, val, 3);
        else if (!strcmp(key, "mavlink_js_roll_right"))     strncpy(config->flightplan.mavlink_js_roll_right, val, 3);
        else if (!strcmp(key, "mavlink_js_pitch_front"))    strncpy(config->flightplan.mavlink_js_pitch_front, val, 3);
        else if (!strcmp(key, "mavlink_js_pitch_back"))     strncpy(config->flightplan.mavlink_js_pitch_back, val, 3);
        else if (!strcmp(key, "mavlink_js_yaw_left"))       strncpy(config->flightplan.mavlink_js_yaw_left, val, 3);
        else if (!strcmp(key, "mavlink_js_yaw_right"))      strncpy(config->flightplan.mavlink_js_yaw_right, val, 3);
        else if (!strcmp(key, "mavlink_js_go_up"))          strncpy(config->flightplan.mavlink_js_go_up, val, 3);
        else if (!strcmp(key, "mavlink_js_go_down"))        strncpy(config->flightplan.mavlink_js_go_down, val, 3);
        else if (!strcmp(key, "mavlink_js_inc_gains"))      strncpy(config->flightplan.mavlink_js_inc_gains, val, 3);
        else if (!strcmp(key, "mavlink_js_dec_gains"))      strncpy(config->flightplan.mavlink_js_dec_gains, val, 3);
        else if (!strcmp(key, "mavlink_js_select"))         strncpy(config->flightplan.mavlink_js_select, val, 3);
        else if (!strcmp(key, "mavlink_js_start"))          strncpy(config->flightplan.mavlink_js_start, val, 3);
    }
    else if (!(strcmp(category, "rescue"))) {
        if (!strcmp(key, "rescue")) sscanf(val, "%d", &(config->rescue.rescue));
    }
}

// --------------------------------------------------------------------------
//! @brief   Get current configurations of AR.Drone.
//! @return  Result of this function
//! @retval  1 Success
//! @retval  0 Failure
// --------------------------------------------------------------------------
int ARDrone::getConfig(void)
{
    // Open the IP address and port
    TCPSocket sockConfig;
    if (!sockConfig.open(ip, ARDRONE_CONTROL_PORT)) {
        CVDRONE_ERROR("TCPSocket::open(port=%d) failed. (%s, %d)\n", ARDRONE_CONTROL_PORT, __FILE__, __LINE__);
        return 0;
    }

    // Send requests
    UDPSocket tmpCommand;
    tmpCommand.open(ip, ARDRONE_AT_PORT);
    tmpCommand.sendf("AT*CTRL=%d,5,0\r", ++seq);
    tmpCommand.sendf("AT*CTRL=%d,4,0\r", ++seq);
    msleep(500);
    tmpCommand.close();

    // Receive data
    char buf[10000] = {'\0'};
    int size = sockConfig.receive((void*)&buf, sizeof(buf));

    // Received something
    if (size > 0) {
        #if 0
        // Saving config.ini
        FILE *file = fopen("config.ini", "w");
        if (file) {
            fprintf(file, buf);
            fclose(file);
        }
        #endif

        // Clear config struct
        memset(&config, 0, sizeof(config));

        // Parsing configurations
        char *token = strtok(buf, "\n");
        if (token != NULL) parse(token, &config);
        while (token != NULL) {
            token = strtok(NULL, "\n");
            if (token != NULL) parse(token, &config);
        }
    }

    #if 0
    // For debug
    printf("general.num_version_config = %d\n", config.general.num_version_config);
    printf("general.num_version_mb = %d\n", config.general.num_version_mb);
    printf("general.num_version_soft = %s\n", config.general.num_version_soft);
    printf("general.drone_serial = %s\n", config.general.drone_serial);
    printf("general.soft_build_date = %s\n", config.general.soft_build_date);
    printf("general.motor1_soft = %f\n", config.general.motor1_soft);
    printf("general.motor1_hard = %f\n", config.general.motor1_hard);
    printf("general.motor1_supplier = %f\n", config.general.motor1_supplier);
    printf("general.motor2_soft = %f\n", config.general.motor2_soft);
    printf("general.motor2_hard = %f\n", config.general.motor2_hard);
    printf("general.motor2_supplier = %f\n", config.general.motor2_supplier);
    printf("general.motor3_soft = %f\n", config.general.motor3_soft);
    printf("general.motor3_hard = %f\n", config.general.motor3_hard);
    printf("general.motor3_supplier = %f\n", config.general.motor3_supplier);
    printf("general.motor4_soft = %f\n", config.general.motor4_soft);
    printf("general.motor4_hard = %f\n", config.general.motor4_hard);
    printf("general.motor4_supplier = %f\n", config.general.motor4_supplier);
    printf("general.ardrone_name = %s\n", config.general.ardrone_name);
    printf("general.flying_time = %d\n", config.general.flying_time);
    printf("general.navdata_demo = %s\n", config.general.navdata_demo ? "true" : "false");
    printf("general.com_watchdog = %d\n", config.general.com_watchdog);
    printf("general.video_enable = %s\n", config.general.video_enable ? "true" : "false");
    printf("general.vision_enable = %s\n", config.general.vision_enable ? "true" : "false");
    printf("general.vbat_min = %d\n", config.general.vbat_min);
    printf("general.localtime = %d\n", config.general.localtime);
    printf("general.navdata_options = %d\n", config.general.navdata_options);
    printf("general.gps_soft = %f\n", config.general.gps_soft);
    printf("general.gps_hard = %f\n", config.general.gps_hard);
    printf("general.localtime_zone = %s\n", config.general.localtime_zone);
    printf("general.battery_type = %d\n", config.general.battery_type);
    printf("control.accs_offset = {%f, %f, %f}\n", config.control.accs_offset[0], config.control.accs_offset[1], config.control.accs_offset[2]);
    printf("control.accs_gains = { %f %f %f %f %f %f %f %f %f }\n", config.control.accs_gains[0], config.control.accs_gains[1], config.control.accs_gains[2], config.control.accs_gains[3], config.control.accs_gains[4], config.control.accs_gains[5], config.control.accs_gains[6], config.control.accs_gains[7], config.control.accs_gains[8]);
    printf("control.gyros_offset = { %f %f %f }\n", config.control.gyros_offset[0], config.control.gyros_offset[1], config.control.gyros_offset[2]);
    printf("control.gyros_gains = { %f %f %f }\n", config.control.gyros_gains[0], config.control.gyros_gains[1], config.control.gyros_gains[2]);
    printf("control.gyros110_offset = { %f %f }\n", config.control.gyros110_offset[0], config.control.gyros110_offset[1]);
    printf("control.gyros110_gains = { %f %f }\n", config.control.gyros110_gains[0], config.control.gyros110_gains[1]);
    printf("control.magneto_offset = { %f %f %f }\n", config.control.magneto_offset[0], config.control.magneto_offset[1], config.control.magneto_offset[2]);
    printf("control.magneto_radius = %f\n", config.control.magneto_radius);
    printf("control.gyro_offset_thr_x = %f\n", config.control.gyro_offset_thr_x);
    printf("control.gyro_offset_thr_y = %f\n", config.control.gyro_offset_thr_y);
    printf("control.gyro_offset_thr_z = %f\n", config.control.gyro_offset_thr_z);
    printf("control.pwm_ref_gyros = %d\n", config.control.pwm_ref_gyros);
    printf("control.osctun_value = %d\n", config.control.osctun_value);
    printf("control.osctun_test = %s\n", config.control.osctun_test ? "true" : "false");
    printf("control.altitude_max = %d\n", config.control.altitude_max);
    printf("control.altitude_min = %d\n", config.control.altitude_min);
    printf("control.outdoor = %s\n", config.control.outdoor ? "true" : "false");
    printf("control.flight_without_shell = %s\n", config.control.flight_without_shell ? "true" : "false");
    printf("control.autonomous_flight = %s\n", config.control.autonomous_flight ? "true" : "false");
    printf("control.flight_anim = %d,%d\n", config.control.flight_anim[0], config.control.flight_anim[1]);
    printf("control.control_level = %d\n", config.control.control_level);
    printf("control.euler_angle_max = %f\n", config.control.euler_angle_max);
    printf("control.control_iphone_tilt = %f\n", config.control.control_iphone_tilt);
    printf("control.control_vz_max = %f\n", config.control.control_vz_max);
    printf("control.control_yaw = %f\n", config.control.control_yaw);
    printf("control.manual_trim = %s\n", config.control.manual_trim ? "true" : "false");
    printf("control.indoor_euler_angle_max = %f\n", config.control.indoor_euler_angle_max);
    printf("control.indoor_control_vz_max = %f\n", config.control.indoor_control_vz_max);
    printf("control.indoor_control_yaw = %f\n", config.control.indoor_control_yaw);
    printf("control.outdoor_euler_angle_max = %f\n", config.control.outdoor_euler_angle_max);
    printf("control.outdoor_control_vz_max = %f\n", config.control.outdoor_control_vz_max);
    printf("control.outdoor_control_yaw = %f\n", config.control.outdoor_control_yaw);
    printf("control.flying_mode = %d\n", config.control.flying_mode);
    printf("control.hovering_range = %d\n", config.control.hovering_range);
    printf("control.flying_camera_mode = %d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", config.control.flying_camera_mode[0], config.control.flying_camera_mode[1], config.control.flying_camera_mode[2], config.control.flying_camera_mode[3], config.control.flying_camera_mode[4], config.control.flying_camera_mode[5], config.control.flying_camera_mode[6], config.control.flying_camera_mode[7], config.control.flying_camera_mode[8], config.control.flying_camera_mode[9]);
    printf("control.flying_camera_enable = %s\n", config.control.flying_camera_enable ? "true" : "false");
    printf("network.ssid_single_player = %s\n", config.network.ssid_single_player);
    printf("network.ssid_multi_player = %s\n", config.network.ssid_multi_player);
    printf("network.wifi_mode = %d\n", config.network.wifi_mode);
    printf("network.wifi_rate = %d\n", config.network.wifi_rate);
    printf("network.owner_mac = %s\n", config.network.owner_mac);
    printf("pic.ultrasound_freq = %d\n", config.pic.ultrasound_freq);
    printf("pic.ultrasound_watchdog = %d\n", config.pic.ultrasound_watchdog);
    printf("pic.pic_version = %d\n", config.pic.pic_version);
    printf("video.camif_fps = %d\n", config.video.camif_fps);
    printf("video.camif_buffers = %d\n", config.video.camif_buffers);
    printf("video.num_trackers = %d\n", config.video.num_trackers);
    printf("video.video_storage_space = %d\n", config.video.video_storage_space);
    printf("video.video_on_usb = %s\n", config.video.video_on_usb ? "true" : "false");
    printf("video.video_file_index = %d\n", config.video.video_file_index);
    printf("video.bitrate = %d\n", config.video.bitrate);
    printf("video.bitrate_ctrl_mode = %d\n", config.video.bitrate_ctrl_mode);
    printf("video.bitrate_storage = %d\n", config.video.bitrate_storage);
    printf("video.codec_fps = %d\n", config.video.codec_fps);
    printf("video.video_codec = %d\n", config.video.video_codec);
    printf("video.video_slices = %d\n", config.video.video_slices);
    printf("video.video_live_socket = %d\n", config.video.video_live_socket);
    printf("video.max_bitrate = %d\n", config.video.max_bitrate);
    printf("video.video_channel = %d\n", config.video.video_channel);
    printf("video.exposure_mode = %d,%d,%d,%d\n", config.video.exposure_mode[0], config.video.exposure_mode[1], config.video.exposure_mode[2], config.video.exposure_mode[3]);
    printf("video.saturation_mode = %d\n", config.video.saturation_mode);
    printf("video.whitebalance_mode = %d,%d\n", config.video.whitebalance_mode[0], config.video.whitebalance_mode[1]);
    printf("leds.leds_anim = %d,%d,%d\n", config.leds.leds_anim[0], config.leds.leds_anim[1], config.leds.leds_anim[2]);
    printf("detect.enemy_colors = %d\n", config.detect.enemy_colors);
    printf("detect.enemy_without_shell = %d\n", config.detect.enemy_without_shell);
    printf("detect.groundstripe_colors = %d\n", config.detect.groundstripe_colors);
    printf("detect.detect_type = %d\n", config.detect.detect_type);
    printf("detect.detections_select_h = %d\n", config.detect.detections_select_h);
    printf("detect.detections_select_v_hsync = %d\n", config.detect.detections_select_v_hsync);
    printf("detect.detections_select_v = %d\n", config.detect.detections_select_v);
    printf("syslog.output = %d\n", config.syslog.output);
    printf("syslog.max_size = %d\n", config.syslog.max_size);
    printf("syslog.nb_files = %d\n", config.syslog.nb_files);
    printf("custom.application_desc = %s\n", config.custom.application_desc);
    printf("custom.profile_desc = %s\n", config.custom.profile_desc);
    printf("custom.session_desc = %s\n", config.custom.session_desc);
    printf("custom.application_id = %s\n", config.custom.application_id);
    printf("custom.profile_id = %s\n", config.custom.profile_id);
    printf("custom.session_id = %s\n", config.custom.session_id);
    printf("userbox.userbox_cmd = %d\n", config.userbox.userbox_cmd);
    printf("gps.latitude = %f\n", config.gps.latitude);
    printf("gps.longitude = %f\n", config.gps.longitude);
    printf("gps.altitude = %f\n", config.gps.altitude);
    printf("gps.accuracy = %f\n", config.gps.accuracy);
    printf("flightplan.default_validation_radius = %f\n", config.flightplan.default_validation_radius);
    printf("flightplan.default_validation_time = %f\n", config.flightplan.default_validation_time);
    printf("flightplan.max_distance_from_takeoff = %d\n", config.flightplan.max_distance_from_takeoff);
    printf("flightplan.gcs_ip = %d\n", config.flightplan.gcs_ip);
    printf("flightplan.video_stop_delay = %d\n", config.flightplan.video_stop_delay);
    printf("flightplan.low_battery_go_home = %s\n", config.flightplan.low_battery_go_home ? "true" : "false");
    printf("flightplan.automatic_heading = %s\n", config.flightplan.automatic_heading ? "true" : "false");
    printf("flightplan.com_lost_action_delay = %d\n", config.flightplan.com_lost_action_delay);
    printf("flightplan.altitude_go_home = %d\n", config.flightplan.altitude_go_home);
    printf("flightplan.mavlink_js_roll_left = %s\n", config.flightplan.mavlink_js_roll_left);
    printf("flightplan.mavlink_js_roll_right = %s\n", config.flightplan.mavlink_js_roll_right);
    printf("flightplan.mavlink_js_pitch_front = %s\n", config.flightplan.mavlink_js_pitch_front);
    printf("flightplan.mavlink_js_pitch_back = %s\n", config.flightplan.mavlink_js_pitch_back);
    printf("flightplan.mavlink_js_yaw_left = %s\n", config.flightplan.mavlink_js_yaw_left);
    printf("flightplan.mavlink_js_yaw_right = %s\n", config.flightplan.mavlink_js_yaw_right);
    printf("flightplan.mavlink_js_go_up = %s\n", config.flightplan.mavlink_js_go_up);
    printf("flightplan.mavlink_js_go_down = %s\n", config.flightplan.mavlink_js_go_down);
    printf("flightplan.mavlink_js_inc_gains = %s\n", config.flightplan.mavlink_js_inc_gains);
    printf("flightplan.mavlink_js_dec_gains = %s\n", config.flightplan.mavlink_js_dec_gains);
    printf("flightplan.mavlink_js_select = %s\n", config.flightplan.mavlink_js_select);
    printf("flightplan.mavlink_js_start = %s\n", config.flightplan.mavlink_js_start);
    printf("rescue.rescue = %d\n", config.rescue.rescue);
    #endif

    // Finalize
    sockConfig.close();

    return 1;
}