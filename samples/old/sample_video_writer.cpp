#include "ardrone/ardrone.h"

// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------
int main(int argc, char **argv)
{
    // AR.Drone class
    ARDrone ardrone;

    // Initialize
    if (!ardrone.open()) {
        printf("Failed to initialize.\n");
        return -1;
    }

    // Image of AR.Drone's camera
    IplImage *image = ardrone.getImage();

    // Name of video
    char filename[256];
    SYSTEMTIME st;
    GetLocalTime(&st);
    sprintf(filename, "cam%d%02d%02d%02d%02d%02d.avi", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);

    // Create a video writer
    CvVideoWriter *video = cvCreateVideoWriter(filename, CV_FOURCC('D','I','B',' '), 30, cvGetSize(image));

    // Main loop
    while (1) {
        // Key input
        int key = cvWaitKey(33);
        if (key == 0x1b) break;

        // Update
        if (!ardrone.update()) break;

        // Get an image
        image = ardrone.getImage();

        // Write a frame
        cvWriteFrame(video, image);

        // Display the image
        cvShowImage("camera", image);
    }

    // Save the video
    cvReleaseVideoWriter(&video);

    // See you
    ardrone.close();

    return 0;
}