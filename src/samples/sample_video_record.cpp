#include "ardrone/ardrone.h"

#define KEY_DOWN(key) (GetAsyncKeyState(key) & 0x8000)
#define KEY_PUSH(key) (GetAsyncKeyState(key) & 0x0001)

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
    int rec = 0;
    printf("Press 'R' to start/stop recording.");

    // Main loop
    while (!GetAsyncKeyState(VK_ESCAPE)) {
        // Update
        if (!ardrone.update()) break;

        // Get an image
        IplImage *image = ardrone.getImage();

        // Video recording start / stop
        if (KEY_PUSH('R')) {
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
        cvWaitKey(1);
    }

    // See you
    ardrone.close();

    return 0;
}