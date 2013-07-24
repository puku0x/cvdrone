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

    // Recording flag
    bool rec = false;
    printf("Press 'R' to start/stop recording.");

    // Main loop
    while (1) {
        // Key input
        int key = cvWaitKey(1);
        if (key == 0x1b) break;

        // Update
        if (!ardrone.update()) break;

        // Get an image
        IplImage *image = ardrone.getImage();

        // Video recording start / stop
        if (key == 'r') {
            rec = !rec;
            ardrone.setVideoRecord(rec);
        }

        // Show recording state
        if (rec) {
            static CvFont font = cvFont(1.0);
            cvPutText(image, "REC", cvPoint(10, 20), &font, CV_RGB(255,0,0));
        }

        // Display the image
        cvShowImage("camera", image);
    }

    // See you
    ardrone.close();

    return 0;
}