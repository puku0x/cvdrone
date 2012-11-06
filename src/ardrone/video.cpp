#include "ardrone.h"
#include "uvlc.h"

// Following site is useful to understand AR.Drone+FFmpeg.
// AR.Drone Development - 2.1.2 AR.Drone 2.0 Video Decording: FFMPEG + SDL2.0 -
// http://ardrone-ailab-u-tokyo.blogspot.jp/2012/07/212-ardrone-20-video-decording-ffmpeg.html

// --------------------------------------------------------------------------
// ARDrone::initVideo()
// Initialize video
// Return value SUCCESS: 1  FAILED: 0
// --------------------------------------------------------------------------
int ARDrone::initVideo(void)
{
    // AR.Drone 2.0
    if (version.major == ARDRONE_VERSION_2) {
        // Open the IP address and port
        char filename[256];
        sprintf(filename, "http://%s:%d", ip, ARDRONE_VIDEO_PORT);
        if (avformat_open_input(&pFormatCtx, filename, NULL, NULL) < 0) {
            printf("ERROR: avformat_open_input() failed. (%s, %d)\n", __FILE__, __LINE__);
            return 0;
        }

        // Dump video information
        avformat_find_stream_info(pFormatCtx, NULL);
        av_dump_format(pFormatCtx, 0, filename, 0);

        // Find codec
        pCodecCtx = pFormatCtx->streams[0]->codec;
        AVCodec *pCodec = avcodec_find_decoder(pCodecCtx->codec_id);

        // Open codec
        if (avcodec_open2(pCodecCtx, pCodec, NULL) < 0) {
            printf("ERROR: avcodec_open2() failed. (%s, %d)\n", __FILE__, __LINE__);
            return 0;
        }

        // Allocate frames and buffer
        pFrame = avcodec_alloc_frame();
        pFrame_BGR24 = avcodec_alloc_frame();
        buffer_BGR24 = (uint8_t*)av_malloc(avpicture_get_size(PIX_FMT_BGR24, pCodecCtx->width, pCodecCtx->height));

        // Associate frame and buffer
        avpicture_fill((AVPicture*)pFrame_BGR24, buffer_BGR24, PIX_FMT_BGR24, pCodecCtx->width, pCodecCtx->height);

        // Convert to BGR
        pConvertCtx_BGR24 = sws_getContext(pCodecCtx->width, pCodecCtx->height, pCodecCtx->pix_fmt, pCodecCtx->width, pCodecCtx->height, PIX_FMT_BGR24, SWS_SPLINE, NULL, NULL, NULL);
    }
    // AR.Drone 1.0
    else {
        // Open the socket
        if (!sockVideo.open(ip, ARDRONE_VIDEO_PORT)) {
            printf("ERROR: UDPSocket::open(port=%d) failed. (%s, %d)\n", ARDRONE_VIDEO_PORT, __FILE__, __LINE__);
            return 0;
        }

        // Set codec
        pCodecCtx = avcodec_alloc_context();
        pCodecCtx->width = 320;
        pCodecCtx->height = 240;

        // Allocate a buffer
        buffer_BGR24 = (uint8_t*)av_malloc(avpicture_get_size(PIX_FMT_BGR24, pCodecCtx->width, pCodecCtx->height));
    }

    // Allocate an IplImage
    img = cvCreateImage(cvSize(pCodecCtx->width, pCodecCtx->height), IPL_DEPTH_8U, 3);
    if (!img) return 0;
    cvZero(img);

    // Create a mutex
    mutexVideo = CreateMutex(NULL, FALSE, NULL);

    // Enable thread loop
    flagVideo = 1;

    // Create a thread
    UINT idVideo;
    threadVideo = (HANDLE)_beginthreadex(NULL, 0, runVideo, this, 0, &idVideo);
    if (threadVideo == INVALID_HANDLE_VALUE) {
        printf("ERROR: _beginthreadex() failed. (%s, %d)\n", __FILE__, __LINE__);
        return 0;
    }

    return 1;
}

// --------------------------------------------------------------------------
// ARDrone::loopVideo()
// Thread function.
// Return value 0
// --------------------------------------------------------------------------
UINT ARDrone::loopVideo(void)
{
    while (flagVideo) {
        // Get video stream
        if (!getVideo()) break;
        Sleep(1);
    }

    // Disable thread loop
    flagVideo = 0;

    return 0;
}

// --------------------------------------------------------------------------
// ARDrone::getVideo()
// Obtaining video stream.
// Return value SUCCESS: 1  FAILED: 0
// --------------------------------------------------------------------------
int ARDrone::getVideo(void)
{
    // AR.Drone 2.0
    if (version.major == ARDRONE_VERSION_2) {
        // Read a frame
        AVPacket packet;
        if (av_read_frame(pFormatCtx, &packet) < 0) {
            //printf("Could not read frame!\n");
            //return 0;
        }
        else {
            // Decode a frame
            int frameDecoded;
            if (avcodec_decode_video2(pCodecCtx, pFrame, &frameDecoded, &packet) < 0) {
                //printf("Could not decode frame!\n");
                //return 0;
            }
            else {
                // Convert to 24bit-BGR
                if (frameDecoded) {
                    WaitForSingleObject(mutexVideo, INFINITE);
                    sws_scale(pConvertCtx_BGR24, (const uint8_t* const*)pFrame->data, pFrame->linesize, 0, pCodecCtx->height, pFrame_BGR24->data, pFrame_BGR24->linesize);
                    ReleaseMutex(mutexVideo);
                }
            }
        }
    }
    // AR.Drone 1.0
    else {
        // Send request
        sockVideo.sendf("\x01\x00\x00\x00");

        // Receive data
        uint32_t buf[1024 * 8];
        int size = sockVideo.receive((void*)&buf, sizeof(buf));

        // Received something
        if (size > 0) {
            // Decode video
            int index;
            uint8_t remainingbits;
            WaitForSingleObject(mutexVideo, INFINITE);
            decodeVideo(buf, &index, &remainingbits, buffer_BGR24, &pCodecCtx->width, &pCodecCtx->height);
            ReleaseMutex(mutexVideo);
        }
    }

    return 1;
}

// --------------------------------------------------------------------------
// ARDrone::getImage()
// Obtaining a frame from your AR.Drone.
// Return value IplImage
// --------------------------------------------------------------------------
IplImage* ARDrone::getImage(void)
{
    // No image
    if (!img) return NULL;

    // AR.Drone 2.0
    if (version.major == ARDRONE_VERSION_2) {
        // Copy the frame to the IplImage
        WaitForSingleObject(mutexVideo, INFINITE);
        memcpy(img->imageData, pFrame_BGR24->data[0], pCodecCtx->width * pCodecCtx->height * sizeof(uint8_t) * 3);
        ReleaseMutex(mutexVideo);
    }
    // AR.Drone 1.0
    else {
        // Enable mutex lock
        WaitForSingleObject(mutexVideo, INFINITE);

        // If the sizes of buffer and IplImage are the same
        if (pCodecCtx->width == img->width && pCodecCtx->height == img->height) {
            // Copy the buffer to the IplImage
            memcpy(img->imageData, buffer_BGR24, pCodecCtx->width * pCodecCtx->height * sizeof(uint8_t) * 3);
        }
        // If the sizes are different
        else {
            // Resize the image to 320x240
            IplImage *small_img = cvCreateImage(cvSize(pCodecCtx->width, pCodecCtx->height), IPL_DEPTH_8U, 3);
            memcpy(small_img->imageData, buffer_BGR24, pCodecCtx->width * pCodecCtx->height * sizeof(uint8_t) * 3);
            cvResize(small_img, img);
            cvReleaseImage(&small_img);
        }

        // Disable mutex lock
        ReleaseMutex(mutexVideo);
    }

    return img;
}

// --------------------------------------------------------------------------
// ARDrone::finalizeVideo()
// Finalize video.
// Return value NONE
// --------------------------------------------------------------------------
void ARDrone::finalizeVideo(void)
{
    // Disable the loop
    flagVideo = 0;

    // Destroy the thread
    if (threadVideo != INVALID_HANDLE_VALUE) {
        WaitForSingleObject(threadVideo, INFINITE);
        CloseHandle(threadVideo);
        threadVideo = INVALID_HANDLE_VALUE;
    }

    // Delete the mutex
    if (mutexVideo != INVALID_HANDLE_VALUE) {
        CloseHandle(mutexVideo);
        mutexVideo = INVALID_HANDLE_VALUE;
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
            av_free(pFrame);
            pFrame = NULL;
        }

        // Deallocate the frame
        if (pFrame_BGR24) {
            av_free(pFrame_BGR24);
            pFrame_BGR24 = NULL;
        }

        // Deallocate the buffer
        if (buffer_BGR24) {
            av_free(buffer_BGR24);
            buffer_BGR24 = NULL;
        }

        // Deallocate the convert context
        if (pConvertCtx_BGR24) {
            sws_freeContext(pConvertCtx_BGR24);
            pConvertCtx_BGR24 = NULL;
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
        if (buffer_BGR24) {
            av_free(buffer_BGR24);
            buffer_BGR24 = NULL;
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