#include "ardrone/ardrone.h"

// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------
int main(int argc, char **argv)
{
    // AR.Drone class
    ARDrone ardrone("192.168.1.1");

    // Main loop
    while (1) {
        // Update
        if (!ardrone.update()) break;

        // Get an image
        IplImage *image = ardrone.getImage();

        // Display the image
        cvShowImage("camera", image);

        // Press Esc to exit
        if (cvWaitKey(1) == 0x1b) break;
    }

    return 0;
}