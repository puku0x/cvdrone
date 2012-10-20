#include "ardrone/ardrone.h"

// --------------------------------------------------------------------------
// main(Number of arguments, Value of arguments)
// This is the main function.
// Return value Success:0 Error:-1
// --------------------------------------------------------------------------
int main(int argc, char **argv)
{
    // AR.Drone class
    ARDrone ardrone("192.168.1.1");

    // Main loop
    while (1) {
        // Update your AR.Drone
        if (!ardrone.update()) break;

        // Getting an image
        IplImage *image = ardrone.getImage();

        // Display the image
        cvShowImage("camera", image);

        // Press Esc to exit
        if (cvWaitKey(1) == 0x1b) break;
    }

    return 0;
}