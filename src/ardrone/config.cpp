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
// ARDrone::getConfig()
// Description  : Parse a configuration string.
// Return value : NONE
// --------------------------------------------------------------------------
void parse(const char *str, ARDRONE_CONFIG *config)
{
    // Split key and value
    char key[256] = {'\0'}, val[256] = {'\0'};
    sscanf(str, "%s = %[^\0]", key, val);

    // Parse the values
    if (!strcmp(key, "general:num_version_config"))             sscanf(val, "%d", &(config->general.num_version_config));
    else if (!strcmp(key, "general:num_version_mb"))            sscanf(val, "%d", &(config->general.num_version_mb));
    else if (!strcmp(key, "general:num_version_soft"))          strncpy(config->general.num_version_soft, val, 32);
    else if (!strcmp(key, "general:drone_serial"))              strncpy(config->general.drone_serial, val, 32);
    else if (!strcmp(key, "general:soft_build_date"))           strncpy(config->general.soft_build_date, val, 32);
    else if (!strcmp(key, "general:motor1_soft"))               sscanf(val, "%f", &(config->general.motor1_soft));
    else if (!strcmp(key, "general:motor1_hard"))               sscanf(val, "%f", &(config->general.motor1_hard));
    else if (!strcmp(key, "general:motor1_supplier"))           sscanf(val, "%f", &(config->general.motor1_supplier));
    else if (!strcmp(key, "general:motor2_soft"))               sscanf(val, "%f", &(config->general.motor2_soft));
    else if (!strcmp(key, "general:motor2_hard"))               sscanf(val, "%f", &(config->general.motor2_hard));
    else if (!strcmp(key, "general:motor2_supplier"))           sscanf(val, "%f", &(config->general.motor2_supplier));
    else if (!strcmp(key, "general:motor3_soft"))               sscanf(val, "%f", &(config->general.motor3_soft));
    else if (!strcmp(key, "general:motor3_hard"))               sscanf(val, "%f", &(config->general.motor3_hard));
    else if (!strcmp(key, "general:motor3_supplier"))           sscanf(val, "%f", &(config->general.motor3_supplier));
    else if (!strcmp(key, "general:motor4_soft"))               sscanf(val, "%f", &(config->general.motor4_soft));
    else if (!strcmp(key, "general:motor4_hard"))               sscanf(val, "%f", &(config->general.motor4_hard));
    else if (!strcmp(key, "general:motor4_supplier"))           sscanf(val, "%f", &(config->general.motor4_supplier));
    else if (!strcmp(key, "general:ardrone_name"))              strncpy(config->general.ardrone_name, val, 32);
    else if (!strcmp(key, "general:flying_time"))               sscanf(val, "%d", &(config->general.flying_time));
    else if (!strcmp(key, "general:navdata_demo"))              config->general.navdata_demo = (!strcmp(val, "TRUE")) ? true : false;
    else if (!strcmp(key, "general:com_watchdog"))              sscanf(val, "%d", &(config->general.com_watchdog));
    else if (!strcmp(key, "general:video_enable"))              config->general.video_enable = (!strcmp(val, "TRUE")) ? true : false;
    else if (!strcmp(key, "general:vision_enable"))             config->general.vision_enable = (!strcmp(val, "TRUE")) ? true : false;
    else if (!strcmp(key, "general:vbat_min"))                  sscanf(val, "%d", &(config->general.vbat_min));
    else if (!strcmp(key, "general:localtime"))                 sscanf(val, "%d", &(config->general.localtime));
    else if (!strcmp(key, "general:navdata_options"))           sscanf(val, "%d", &(config->general.navdata_options));
    else if (!strcmp(key, "control:accs_offset"))               sscanf(val, "{ %f %f %f }", &(config->control.accs_offset[0]), &(config->control.accs_offset[1]), &(config->control.accs_offset[2]));
    else if (!strcmp(key, "control:accs_gains"))                sscanf(val, "{ %f %f %f %f %f %f %f %f %f }", &(config->control.accs_gains[0]), &(config->control.accs_gains[1]), &(config->control.accs_gains[2]), &(config->control.accs_gains[3]), &(config->control.accs_gains[4]), &(config->control.accs_gains[5]), &(config->control.accs_gains[6]), &(config->control.accs_gains[7]), &(config->control.accs_gains[8]));
    else if (!strcmp(key, "control:gyros_offset"))              sscanf(val, "{ %f %f %f }", &(config->control.gyros_offset[0]), &(config->control.gyros_offset[1]), &(config->control.gyros_offset[2]));
    else if (!strcmp(key, "control:gyros_gains"))               sscanf(val, "{ %f %f %f }", &(config->control.gyros_gains[0]), &(config->control.gyros_gains[1]), &(config->control.gyros_gains[2]));
    else if (!strcmp(key, "control:gyros110_offset"))           sscanf(val, "{ %f %f }", &(config->control.gyros110_offset[0]), &(config->control.gyros110_offset[1]));
    else if (!strcmp(key, "control:gyros110_gains"))            sscanf(val, "{ %f %f }", &(config->control.gyros110_gains[0]), &(config->control.gyros110_gains[1]));
    else if (!strcmp(key, "control:magneto_offset"))            sscanf(val, "{ %f %f %f }", &(config->control.magneto_offset[0]), &(config->control.magneto_offset[1]), &(config->control.magneto_offset[2]));
    else if (!strcmp(key, "control:magneto_radius"))            sscanf(val, "%f", &(config->control.magneto_radius));
    else if (!strcmp(key, "control:gyro_offset_thr_x"))         sscanf(val, "%f", &(config->control.gyro_offset_thr_x));
    else if (!strcmp(key, "control:gyro_offset_thr_y"))         sscanf(val, "%f", &(config->control.gyro_offset_thr_y));
    else if (!strcmp(key, "control:gyro_offset_thr_z"))         sscanf(val, "%f", &(config->control.gyro_offset_thr_z));
    else if (!strcmp(key, "control:pwm_ref_gyros"))             sscanf(val, "%d", &(config->control.pwm_ref_gyros));
    else if (!strcmp(key, "control:osctun_value"))              sscanf(val, "%d", &(config->control.osctun_value));
    else if (!strcmp(key, "control:osctun_test"))               config->control.osctun_test = (!strcmp(val, "TRUE")) ? true : false;
    else if (!strcmp(key, "control:altitude_max"))              sscanf(val, "%d", &(config->control.altitude_max));
    else if (!strcmp(key, "control:altitude_min"))              sscanf(val, "%d", &(config->control.altitude_min));
    else if (!strcmp(key, "control:outdoor"))                   config->control.outdoor = (!strcmp(val, "TRUE")) ? true : false;
    else if (!strcmp(key, "control:flight_without_shell"))      config->control.flight_without_shell = (!strcmp(val, "TRUE")) ? true : false;
    else if (!strcmp(key, "control:autonomous_flight"))         config->control.autonomous_flight = (!strcmp(val, "TRUE")) ? true : false;
    else if (!strcmp(key, "control:flight_anim"))               sscanf(val, "%d,%d", &(config->control.flight_anim[0]), &(config->control.flight_anim[1]));
    else if (!strcmp(key, "control:control_level"))             sscanf(val, "%d", &(config->control.control_level));
    else if (!strcmp(key, "control:euler_angle_max"))           sscanf(val, "%f", &(config->control.euler_angle_max));
    else if (!strcmp(key, "control:control_iphone_tilt"))       sscanf(val, "%f", &(config->control.control_iphone_tilt));
    else if (!strcmp(key, "control:control_vz_max"))            sscanf(val, "%f", &(config->control.control_vz_max));
    else if (!strcmp(key, "control:control_yaw"))               sscanf(val, "%f", &(config->control.control_yaw));
    else if (!strcmp(key, "control:manual_trim"))               config->control.manual_trim = (!strcmp(val, "TRUE")) ? true : false;
    else if (!strcmp(key, "control:indoor_euler_angle_max"))    sscanf(val, "%f", &(config->control.indoor_euler_angle_max));
    else if (!strcmp(key, "control:indoor_control_vz_max"))     sscanf(val, "%f", &(config->control.indoor_control_vz_max));
    else if (!strcmp(key, "control:indoor_control_yaw"))        sscanf(val, "%f", &(config->control.indoor_control_yaw));
    else if (!strcmp(key, "control:outdoor_euler_angle_max"))   sscanf(val, "%f", &(config->control.outdoor_euler_angle_max));
    else if (!strcmp(key, "control:outdoor_control_vz_max"))    sscanf(val, "%f", &(config->control.outdoor_control_vz_max));
    else if (!strcmp(key, "control:outdoor_control_yaw"))       sscanf(val, "%f", &(config->control.outdoor_control_yaw));
    else if (!strcmp(key, "control:flying_mode"))               sscanf(val, "%d", &(config->control.flying_mode));
    else if (!strcmp(key, "control:hovering_range"))            sscanf(val, "%d", &(config->control.hovering_range));
    else if (!strcmp(key, "network:ssid_single_player"))        strncpy(config->network.ssid_single_player, val, 32);
    else if (!strcmp(key, "network:ssid_multi_player"))         strncpy(config->network.ssid_multi_player, val, 32);
    else if (!strcmp(key, "network:wifi_mode"))                 sscanf(val, "%d", &(config->network.wifi_mode));
    else if (!strcmp(key, "network:wifi_rate"))                 sscanf(val, "%d", &(config->network.wifi_rate));
    else if (!strcmp(key, "network:owner_mac"))                 strncpy(config->network.owner_mac, val, 32);
    else if (!strcmp(key, "pic:ultrasound_freq"))               sscanf(val, "%d", &(config->pic.ultrasound_freq));
    else if (!strcmp(key, "pic:ultrasound_watchdog"))           sscanf(val, "%d", &(config->pic.ultrasound_watchdog));
    else if (!strcmp(key, "pic:pic_version"))                   sscanf(val, "%d", &(config->pic.pic_version));
    else if (!strcmp(key, "video:camif_fps"))                   sscanf(val, "%d", &(config->video.camif_fps));
    else if (!strcmp(key, "video:camif_buffers"))               sscanf(val, "%d", &(config->video.camif_buffers));
    else if (!strcmp(key, "video:num_trackers"))                sscanf(val, "%d", &(config->video.num_trackers));
    else if (!strcmp(key, "video:video_storage_space"))         sscanf(val, "%d", &(config->video.video_storage_space));
    else if (!strcmp(key, "video:video_on_usb"))                config->video.video_on_usb = (!strcmp(val, "TRUE")) ? true : false;
    else if (!strcmp(key, "video:video_file_index"))            sscanf(val, "%d", &(config->video.video_file_index));
    else if (!strcmp(key, "video:bitrate"))                     sscanf(val, "%d", &(config->video.bitrate));
    else if (!strcmp(key, "video:bitrate_ctrl_mode"))           sscanf(val, "%d", &(config->video.bitrate_ctrl_mode));
    else if (!strcmp(key, "video:bitrate_storage"))             sscanf(val, "%d", &(config->video.bitrate_storage));
    else if (!strcmp(key, "video:codec_fps"))                   sscanf(val, "%d", &(config->video.codec_fps));
    else if (!strcmp(key, "video:video_codec"))                 sscanf(val, "%d", &(config->video.video_codec));
    else if (!strcmp(key, "video:video_slices"))                sscanf(val, "%d", &(config->video.video_slices));
    else if (!strcmp(key, "video:video_live_socket"))           sscanf(val, "%d", &(config->video.video_live_socket));
    else if (!strcmp(key, "video:max_bitrate"))                 sscanf(val, "%d", &(config->video.max_bitrate));
    else if (!strcmp(key, "video:video_channel"))               sscanf(val, "%d", &(config->video.video_channel));
    else if (!strcmp(key, "leds:leds_anim"))                    sscanf(val, "%d,%d,%d", &(config->leds.leds_anim[0]), &(config->leds.leds_anim[1]), &(config->leds.leds_anim[2]));
    else if (!strcmp(key, "detect:enemy_colors"))               sscanf(val, "%d", &(config->detect.enemy_colors));
    else if (!strcmp(key, "detect:enemy_without_shell"))        sscanf(val, "%d", &(config->detect.enemy_without_shell));
    else if (!strcmp(key, "detect:groundstripe_colors"))        sscanf(val, "%d", &(config->detect.groundstripe_colors));
    else if (!strcmp(key, "detect:detect_type"))                sscanf(val, "%d", &(config->detect.detect_type));
    else if (!strcmp(key, "detect:detections_select_h"))        sscanf(val, "%d", &(config->detect.detections_select_h));
    else if (!strcmp(key, "detect:detections_select_v_hsync"))  sscanf(val, "%d", &(config->detect.detections_select_v_hsync));
    else if (!strcmp(key, "detect:detections_select_v"))        sscanf(val, "%d", &(config->detect.detections_select_v));
    else if (!strcmp(key, "syslog:output"))                     sscanf(val, "%d", &(config->syslog.output));
    else if (!strcmp(key, "syslog:max_size"))                   sscanf(val, "%d", &(config->syslog.max_size));
    else if (!strcmp(key, "syslog:nb_files"))                   sscanf(val, "%d", &(config->syslog.nb_files));
    else if (!strcmp(key, "custom:application_desc"))           strncpy(config->custom.application_desc, val, 64);
    else if (!strcmp(key, "custom:profile_desc"))               strncpy(config->custom.profile_desc, val, 64);
    else if (!strcmp(key, "custom:session_desc"))               strncpy(config->custom.session_desc, val, 64);
    else if (!strcmp(key, "custom:application_id"))             strncpy(config->custom.application_id, val, 8);
    else if (!strcmp(key, "custom:profile_id"))                 strncpy(config->custom.profile_id, val, 8);
    else if (!strcmp(key, "custom:session_id"))                 strncpy(config->custom.session_id, val, 8);
    else if (!strcmp(key, "userbox:userbox_cmd"))               sscanf(val, "%d", &(config->userbox.userbox_cmd));
    else if (!strcmp(key, "gps:latitude"))                      sscanf(val, "%f", &(config->gps.latitude));
    else if (!strcmp(key, "gps:longitude"))                     sscanf(val, "%f", &(config->gps.longitude));
    else if (!strcmp(key, "gps:altitude"))                      sscanf(val, "%f", &(config->gps.altitude));
}

// --------------------------------------------------------------------------
// ARDrone::getConfig()
// Description  : Get current configurations of AR.Drone.
// Return value : SUCCESS: 1  FAILURE: 0
// --------------------------------------------------------------------------
int ARDrone::getConfig(void)
{
    // Open the IP address and port
    TCPSocket sockConfig;
    if (!sockConfig.open(ip, ARDRONE_CONFIG_PORT)) {
        CVDRONE_ERROR("TCPSocket::open(port=%d) failed. (%s, %d)\n", ARDRONE_CONFIG_PORT, __FILE__, __LINE__);
        return 0;
    }

    // Send requests
    UDPSocket tmpCommand;
    tmpCommand.open(ip, ARDRONE_COMMAND_PORT);
    tmpCommand.sendf("AT*CTRL=%d,5,0\r", seq++);
    tmpCommand.sendf("AT*CTRL=%d,4,0\r", seq++);
    msleep(500);
    tmpCommand.close();

    // Receive data
    char buf[4096] = {'\0'};
    int size = sockConfig.receive((void*)&buf, sizeof(buf));

    // Received something
    if (size > 0) {
        // Clear config struct
        memset(&config, 0, sizeof(ARDRONE_CONFIG));

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
    printf("general:soft_build_date = %s\n", config.general.soft_build_date);
    printf("general:motor1_soft = %f\n", config.general.motor1_soft);
    printf("general:motor1_hard = %f\n", config.general.motor1_hard);
    printf("general:motor1_supplier = %f\n", config.general.motor1_supplier);
    printf("general:motor2_soft = %f\n", config.general.motor2_soft);
    printf("general:motor2_hard = %f\n", config.general.motor2_hard);
    printf("general:motor2_supplier = %f\n", config.general.motor2_supplier);
    printf("general:motor3_soft = %f\n", config.general.motor3_soft);
    printf("general:motor3_hard = %f\n", config.general.motor3_hard);
    printf("general:motor3_supplier = %f\n", config.general.motor3_supplier);
    printf("general:motor4_soft = %f\n", config.general.motor4_soft);
    printf("general:motor4_hard = %f\n", config.general.motor4_hard);
    printf("general:motor4_supplier = %f\n", config.general.motor4_supplier);
    printf("general.ardrone_name = %s\n", config.general.ardrone_name);
    printf("general.flying_time = %d\n", config.general.flying_time);
    printf("general.navdata_demo = %s\n", config.general.navdata_demo ? "true" : "false");
    printf("general.com_watchdog = %d\n", config.general.com_watchdog);
    printf("general.video_enable = %s\n", config.general.video_enable ? "true" : "false");
    printf("general.vision_enable = %s\n", config.general.vision_enable ? "true" : "false");
    printf("general.vbat_min = %d\n", config.general.vbat_min);
    printf("general.localtime = %d\n", config.general.localtime);
    printf("general.navdata_options = %d\n", config.general.navdata_options);
    printf("control:accs_offset = {%f, %f, %f}\n", config.control.accs_offset[0], config.control.accs_offset[1], config.control.accs_offset[2]);
    printf("control:accs_gains = { %f %f %f %f %f %f %f %f %f }\n", config.control.accs_gains[0], config.control.accs_gains[1], config.control.accs_gains[2], config.control.accs_gains[3], config.control.accs_gains[4], config.control.accs_gains[5], config.control.accs_gains[6], config.control.accs_gains[7], config.control.accs_gains[8]);
    printf("control:gyros_offset = { %f %f %f }\n", config.control.gyros_offset[0], config.control.gyros_offset[1], config.control.gyros_offset[2]);
    printf("control:gyros_gains = { %f %f %f }\n", config.control.gyros_gains[0], config.control.gyros_gains[1], config.control.gyros_gains[2]);
    printf("control:gyros110_offset = { %f %f }\n", config.control.gyros110_offset[0], config.control.gyros110_offset[1]);
    printf("control:gyros110_gains = { %f %f }\n", config.control.gyros110_gains[0], config.control.gyros110_gains[1]);
    printf("control:magneto_offset = { %f %f %f }\n", config.control.magneto_offset[0], config.control.magneto_offset[1], config.control.magneto_offset[2]);
    printf("control:magneto_radius = %f\n", config.control.magneto_radius);
    printf("control:gyro_offset_thr_x = %f\n", config.control.gyro_offset_thr_x);
    printf("control:gyro_offset_thr_y = %f\n", config.control.gyro_offset_thr_y);
    printf("control:gyro_offset_thr_z = %f\n", config.control.gyro_offset_thr_z);
    printf("control:pwm_ref_gyros = %d\n", config.control.pwm_ref_gyros);
    printf("control:osctun_value = %d\n", config.control.osctun_value);
    printf("control:osctun_test = %s\n", config.control.osctun_test ? "true" : "false");
    printf("control:altitude_max = %d\n", config.control.altitude_max);
    printf("control:altitude_min = %d\n", config.control.altitude_min);
    printf("control:outdoor = %s\n", config.control.outdoor ? "true" : "false");
    printf("control:flight_without_shell = %s\n", config.control.flight_without_shell ? "true" : "false");
    printf("control:autonomous_flight = %s\n", config.control.autonomous_flight ? "true" : "false");
    printf("control:flight_anim = %d,%d\n", config.control.flight_anim[0], config.control.flight_anim[1]);
    printf("control:control_level = %d\n", config.control.control_level);
    printf("control:euler_angle_max = %f\n", config.control.euler_angle_max);
    printf("control:control_iphone_tilt = %f\n", config.control.control_iphone_tilt);
    printf("control:control_vz_max = %f\n", config.control.control_vz_max);
    printf("control:control_yaw = %f\n", config.control.control_yaw);
    printf("control:manual_trim = %s\n", config.control.manual_trim ? "true" : "false");
    printf("control:indoor_euler_angle_max = %f\n", config.control.indoor_euler_angle_max);
    printf("control:indoor_control_vz_max = %f\n", config.control.indoor_control_vz_max);
    printf("control:indoor_control_yaw = %f\n", config.control.indoor_control_yaw);
    printf("control:outdoor_euler_angle_max = %f\n", config.control.outdoor_euler_angle_max);
    printf("control:outdoor_control_vz_max = %f\n", config.control.outdoor_control_vz_max);
    printf("control:outdoor_control_yaw = %f\n", config.control.outdoor_control_yaw);
    printf("control:flying_mode = %d\n", config.control.flying_mode);
    printf("control:hovering_range = %d\n", config.control.hovering_range);
    printf("network:ssid_single_player = %s\n", config.network.ssid_single_player);
    printf("network:ssid_multi_player = %s\n", config.network.ssid_multi_player);
    printf("network:wifi_mode = %d\n", config.network.wifi_mode);
    printf("network:wifi_rate = %d\n", config.network.wifi_rate);
    printf("network:owner_mac = %s\n", config.network.owner_mac);
    printf("pic:ultrasound_freq = %d\n", config.pic.ultrasound_freq);
    printf("pic:ultrasound_watchdog = %d\n", config.pic.ultrasound_watchdog);
    printf("pic:pic_version = %d\n", config.pic.pic_version);
    printf("video:camif_fps = %d\n", config.video.camif_fps);
    printf("video:camif_buffers = %d\n", config.video.camif_buffers);
    printf("video:num_trackers = %d\n", config.video.num_trackers);
    printf("video:video_storage_space = %d\n", config.video.video_storage_space);
    printf("video:video_on_usb = %s\n", config.video.video_on_usb ? "true" : "false");
    printf("video:video_file_index = %d\n", config.video.video_file_index);
    printf("video:bitrate = %d\n", config.video.bitrate);
    printf("video:bitrate_ctrl_mode = %d\n", config.video.bitrate_ctrl_mode);
    printf("video:bitrate_storage = %d\n", config.video.bitrate_storage);
    printf("video:codec_fps = %d\n", config.video.codec_fps);
    printf("video:video_codec = %d\n", config.video.video_codec);
    printf("video:video_slices = %d\n", config.video.video_slices);
    printf("video:video_live_socket = %d\n", config.video.video_live_socket);
    printf("video:max_bitrate = %d\n", config.video.max_bitrate);
    printf("video:video_channel = %d\n", config.video.video_channel);
    printf("leds:leds_anim = %d,%d,%d\n", config.leds.leds_anim[0], config.leds.leds_anim[1], config.leds.leds_anim[2]);
    printf("detect:enemy_colors = %d\n", config.detect.enemy_colors);
    printf("detect:enemy_without_shell = %d\n", config.detect.enemy_without_shell);
    printf("detect:groundstripe_colors = %d\n", config.detect.groundstripe_colors);
    printf("detect:detect_type = %d\n", config.detect.detect_type);
    printf("detect:detections_select_h = %d\n", config.detect.detections_select_h);
    printf("detect:detections_select_v_hsync = %d\n", config.detect.detections_select_v_hsync);
    printf("detect:detections_select_v = %d\n", config.detect.detections_select_v);
    printf("syslog:output = %d\n", config.syslog.output);
    printf("syslog:max_size = %d\n", config.syslog.max_size);
    printf("syslog:nb_files = %d\n", config.syslog.nb_files);
    printf("custom:application_desc = %s\n", config.custom.application_desc);
    printf("custom:profile_desc = %s\n", config.custom.profile_desc);
    printf("custom:session_desc = %s\n", config.custom.session_desc);
    printf("custom:application_id = %s\n", config.custom.application_id);
    printf("custom:profile_id = %s\n", config.custom.profile_id);
    printf("custom:session_id = %s\n", config.custom.session_id);
    printf("userbox:userbox_cmd = %d\n", config.userbox.userbox_cmd);
    printf("gps:latitude = %f\n", config.gps.latitude);
    printf("gps:longitude = %f\n", config.gps.longitude);
    printf("gps:altitude = %f\n", config.gps.altitude);
    #endif

    // Finalize
    sockConfig.close();

    return 1;
}