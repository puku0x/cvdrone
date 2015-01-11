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
//! @file   video.cpp
//! @brief  Converting video into IplImage or cv::Mat
//
// -------------------------------------------------------------------------

#include "ardrone.h"
#include "uvlc.h"

// The code decoding H.264 video is based on the following sites.
// - An ffmpeg and SDL Tutorial - Tutorial 01: Making Screencaps -
//   http://dranger.com/ffmpeg/tutorial01.html
// - AR.Drone Development - 2.1.2 AR.Drone 2.0 Video Decording: FFMPEG + SDL2.0 -
//   http://ardrone-ailab-u-tokyo.blogspot.jp/2012/07/212-ardrone-20-video-decording-ffmpeg.html

// --------------------------------------------------------------------------
//! @brief   Initialize video.
//! @return  Result of initialization
//! @retval  1 Success
//! @retval  0 Failure
// --------------------------------------------------------------------------
int ARDrone::initVideo(void)
{
    // AR.Drone 2.0
    if (version.major == ARDRONE_VERSION_2) {
        // Open the IP address and port
        char filename[256];
        sprintf(filename, "tcp://%s:%d", ip, ARDRONE_VIDEO_PORT);
        if (avformat_open_input(&pFormatCtx, filename, NULL, NULL) < 0) {
            CVDRONE_ERROR("avformat_open_input() was failed. (%s, %d)\n", __FILE__, __LINE__);
            return 0;
        }

        // Retrive and dump stream information
        avformat_find_stream_info(pFormatCtx, NULL);
        av_dump_format(pFormatCtx, 0, filename, 0);

        // Find the decoder for the video stream
        pCodecCtx = pFormatCtx->streams[0]->codec;
        AVCodec *pCodec = avcodec_find_decoder(pCodecCtx->codec_id);
        if (pCodec == NULL) {
            CVDRONE_ERROR("avcodec_find_decoder() was failed. (%s, %d)\n", __FILE__, __LINE__);
            return 0;
        }

        // Open codec
        if (avcodec_open2(pCodecCtx, pCodec, NULL) < 0) {
            CVDRONE_ERROR("avcodec_open2() was failed. (%s, %d)\n", __FILE__, __LINE__);
            return 0;
        }

        // Allocate video frames and a buffer
        #if LIBAVCODEC_VERSION_INT >= AV_VERSION_INT(55,28,1)
        pFrame = av_frame_alloc();
        pFrameBGR = av_frame_alloc();
        #else
        pFrame = avcodec_alloc_frame();
        pFrameBGR = avcodec_alloc_frame();
        #endif
        bufferBGR = (uint8_t*)av_mallocz(avpicture_get_size(PIX_FMT_BGR24, pCodecCtx->width, pCodecCtx->height) * sizeof(uint8_t));

        // Assign appropriate parts of buffer to image planes in pFrameBGR
        avpicture_fill((AVPicture*)pFrameBGR, bufferBGR, PIX_FMT_BGR24, pCodecCtx->width, pCodecCtx->height);

        // Convert it to BGR
        pConvertCtx = sws_getContext(pCodecCtx->width, pCodecCtx->height, pCodecCtx->pix_fmt, pCodecCtx->width, pCodecCtx->height, PIX_FMT_BGR24, SWS_SPLINE, NULL, NULL, NULL);
    }
    // AR.Drone 1.0
    else {
        // Open the IP address and port
        if (!sockVideo.open(ip, ARDRONE_VIDEO_PORT)) {
            CVDRONE_ERROR("UDPSocket::open(port=%d) was failed. (%s, %d)\n", ARDRONE_VIDEO_PORT, __FILE__, __LINE__);
            return 0;
        }

        // Set codec
        pCodecCtx = avcodec_alloc_context3(NULL);
        pCodecCtx->width = 320;
        pCodecCtx->height = 240;

        // Allocate a buffer
        bufferBGR = (uint8_t*)av_mallocz(avpicture_get_size(PIX_FMT_BGR24, pCodecCtx->width, pCodecCtx->height));
    }

    // Allocate an IplImage
    img = cvCreateImage(cvSize(pCodecCtx->width, (pCodecCtx->height == 368) ? 360 : pCodecCtx->height), IPL_DEPTH_8U, 3);
    if (!img) {
        CVDRONE_ERROR("cvCreateImage() was failed. (%s, %d)\n", __FILE__, __LINE__);
        return 0;
    }

    // Clear the image
    cvZero(img);

    // Create a mutex
    mutexVideo = new pthread_mutex_t;
    pthread_mutex_init(mutexVideo, NULL);

    // Create a thread
    threadVideo = new pthread_t;
    if (pthread_create(threadVideo, NULL, runVideo, this) != 0) {
        CVDRONE_ERROR("pthread_create() was failed. (%s, %d)\n", __FILE__, __LINE__);
        return 0;
    }

    return 1;
}

// --------------------------------------------------------------------------
//! @brief   Thread function for video.
//! @return  None
// --------------------------------------------------------------------------
void ARDrone::loopVideo(void)
{
    while (1) {
        // Get video stream
        if (!getVideo()) break;
        pthread_testcancel();
        msleep(1);
    }
}

// --------------------------------------------------------------------------
//! @brief   Get AR.Drone's video stream.
//! @return  Result of this function
//! @retval  1 Success
//! @retval  0 Failure
// --------------------------------------------------------------------------
int ARDrone::getVideo(void)
{
    // AR.Drone 2.0
    if (version.major == ARDRONE_VERSION_2) {
        AVPacket packet;
        int frameFinished = 0;

        // Read all frames
        while (av_read_frame(pFormatCtx, &packet) >= 0) {
            // Decode the frame
            avcodec_decode_video2(pCodecCtx, pFrame, &frameFinished, &packet);

            // Decoded all frames
            if (frameFinished) {
                // Convert to BGR
                if (mutexVideo) pthread_mutex_lock(mutexVideo);
                sws_scale(pConvertCtx, (const uint8_t* const*)pFrame->data, pFrame->linesize, 0, pCodecCtx->height, pFrameBGR->data, pFrameBGR->linesize);
                newImage = true;
                if (mutexVideo) pthread_mutex_unlock(mutexVideo);

                // Free the packet and break immidiately
                av_free_packet(&packet);
                return 1;
                //break;
            }

            // Free the packet
            av_free_packet(&packet);
        }
        return 0;
    }
    // AR.Drone 1.0
    else {
        // Send request
        sockVideo.sendf("\x01\x00\x00\x00");

        // Receive data
        uint8_t buf[122880];
        int size = sockVideo.receive((void*)&buf, sizeof(buf));

        // Received something
        if (size > 0) {
            // Decode UVLC video
            if (mutexVideo) pthread_mutex_lock(mutexVideo);
            UVLC::DecodeVideo(buf, size, bufferBGR, &pCodecCtx->width, &pCodecCtx->height);
            if (mutexVideo) pthread_mutex_unlock(mutexVideo);
        }
    }

    return 1;
}

// --------------------------------------------------------------------------
//! @brief   Get an image from the AR.Drone's camera.
//! @return  An OpenCV image data (IplImage or cv::Mat)
//! @retval  NULL Failure
// --------------------------------------------------------------------------
ARDRONE_IMAGE ARDrone::getImage(void)
{
    // There is no image
    if (!img) return ARDRONE_IMAGE(NULL);

    // Enable mutex lock
    if (mutexVideo) pthread_mutex_lock(mutexVideo);

    // AR.Drone 2.0
    if (version.major == ARDRONE_VERSION_2) {
        // Copy current frame to an IplImage
        memcpy(img->imageData, pFrameBGR->data[0], pCodecCtx->width * ((pCodecCtx->height == 368) ? 360 : pCodecCtx->height) * sizeof(uint8_t) * 3);
    }
    // AR.Drone 1.0
    else {
        // If the sizes of the buffer and the IplImage are differnt
        if (pCodecCtx->width != img->width || pCodecCtx->height != img->height) {
            // Resize the image to 320x240
            IplImage *small_img = cvCreateImageHeader(cvSize(pCodecCtx->width, pCodecCtx->height), IPL_DEPTH_8U, 3);
            small_img->imageData = (char*)bufferBGR;
            cvResize(small_img, img, CV_INTER_CUBIC);
            cvReleaseImageHeader(&small_img);
        }
        // For 320x240 image, just copy it
        else memcpy(img->imageData, bufferBGR, pCodecCtx->width * pCodecCtx->height * sizeof(uint8_t) * 3);
    }
    
    // The latest image has been read, so change newImage accordingly
    newImage = false;

    // Disable mutex lock
    if (mutexVideo) pthread_mutex_unlock(mutexVideo);

    return ARDRONE_IMAGE(img);
}

// --------------------------------------------------------------------------
//! @brief   A variation of getImage() like cv::VideoCapture.
//! @return  An OpenCV image data (cv::Mat)
// --------------------------------------------------------------------------
ARDrone& ARDrone::operator >> (cv::Mat &image)
{
    image = getImage();
    return *this;
}

// --------------------------------------------------------------------------
//! @brief   Check whether we have received a new image since the last getImage().
//! @return  A bool that is true if we have received a new image and false if we have not
// --------------------------------------------------------------------------
bool ARDrone::willGetNewImage(void) {
    // Enable mutex lock
    if (mutexVideo) pthread_mutex_lock(mutexVideo);
    
    bool answer = newImage;
    
    // Disable mutex lock
    if (mutexVideo) pthread_mutex_unlock(mutexVideo);
    
    return answer;
}

// --------------------------------------------------------------------------
//! @brief   Finalize video.
//! @return  None
// --------------------------------------------------------------------------
void ARDrone::finalizeVideo(void)
{
    // Destroy the thread
    if (threadVideo) {
        pthread_cancel(*threadVideo);
        pthread_join(*threadVideo, NULL);
        delete threadVideo;
        threadVideo = NULL;
    }

    // Delete the mutex
    if (mutexVideo) {
        pthread_mutex_destroy(mutexVideo);
        delete mutexVideo;
        mutexVideo = NULL;
    }

    // Release the IplImage
    if (img) {
        cvReleaseImage(&img);
        img = NULL;
    }

    // AR.Drone 2.0
    if (version.major == ARDRONE_VERSION_2) {
        // Deallocate the frame
        if (pFrame) {
            #if LIBAVCODEC_VERSION_INT >= AV_VERSION_INT(55,28,1)
            av_frame_free(&pFrame);
            #else
            avcodec_free_frame(&pFrame);
            #endif
            pFrame = NULL;
        }

        // Deallocate the frame
        if (pFrameBGR) {
            #if LIBAVCODEC_VERSION_INT >= AV_VERSION_INT(55,28,1)
            av_frame_free(&pFrameBGR);
            #else
            avcodec_free_frame(&pFrameBGR);
            #endif
            pFrameBGR = NULL;
        }

        // Deallocate the buffer
        if (bufferBGR) {
            av_free(bufferBGR);
            bufferBGR = NULL;
        }

        // Deallocate the convert context
        if (pConvertCtx) {
            sws_freeContext(pConvertCtx);
            pConvertCtx = NULL;
        }

        // Deallocate the codec
        if (pCodecCtx) {
            avcodec_close(pCodecCtx);
            pCodecCtx = NULL;
        }

        // Deallocate the format context
        if (pFormatCtx) {
            avformat_close_input(&pFormatCtx);
            pFormatCtx = NULL;
        }
    }
    // AR.Drone 1.0
    else {
        // Deallocate the buffer
        if (bufferBGR) {
            av_free(bufferBGR);
            bufferBGR = NULL;
        }

        // Deallocate the codec
        if (pCodecCtx) {
            avcodec_close(pCodecCtx);
            pCodecCtx = NULL;
        }

        // Close the socket
        sockVideo.close();
    }
}