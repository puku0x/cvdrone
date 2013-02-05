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

    // Battery
    int battery = ardrone.getBatteryPercentage();
    printf("ardrone.battery = %d [%%]\n", battery);

    // Instructions
    printf("***************************************\n");
    printf("*       CV Drone sample program       *\n");
    printf("*           - Haw To Play -           *\n");
    printf("***************************************\n");
    printf("*                                     *\n");
    printf("* - Controls -                        *\n");
    printf("*    'Space' -- Takeoff/Landing       *\n");
    printf("*    'Up'    -- Move forward          *\n");
    printf("*    'Down'  -- Move backward         *\n");
    printf("*    'Left'  -- Turn left             *\n");
    printf("*    'Right' -- Turn right            *\n");
    printf("*    'Shift+Up'    -- Move upward     *\n");
    printf("*    'Shift+Down'  -- Move downward   *\n");
    printf("*    'Shift+Left'  -- Move left       *\n");
    printf("*    'Shift+Right' -- Move right      *\n");
    printf("*                                     *\n");
    printf("* - Others -                          *\n");
    printf("*    'C'     -- Change camera         *\n");
    printf("*    'Esc'   -- Exit                  *\n");
    printf("*                                     *\n");
    printf("***************************************\n\n");

    // Main loop
    while (!GetAsyncKeyState(VK_ESCAPE)) {
        // Update
        if (!ardrone.update()) break;

        // Get an image
        IplImage *image = ardrone.getImage();

        // Take off / Landing
        if (KEY_PUSH(VK_SPACE)) {
            if (ardrone.onGround()) ardrone.takeoff();
            else                    ardrone.landing();
        }

        // Move
        double vx = 0.0, vy = 0.0, vz = 0.0, vr = 0.0;
        if (KEY_DOWN(VK_SHIFT)) {
            if (KEY_DOWN(VK_UP))    vz =  1.0;
            if (KEY_DOWN(VK_DOWN))  vz = -1.0;
            if (KEY_DOWN(VK_LEFT))  vy =  1.0;
            if (KEY_DOWN(VK_RIGHT)) vy = -1.0;
        }
        else {
            if (KEY_DOWN(VK_UP))    vx =  1.0;
            if (KEY_DOWN(VK_DOWN))  vx = -1.0;
            if (KEY_DOWN(VK_LEFT))  vr =  1.0;
            if (KEY_DOWN(VK_RIGHT)) vr = -1.0;
        }
        ardrone.move3D(vx, vy, vz, vr);

        // Change camera
        static int mode = 0;
        if (KEY_PUSH('C')) ardrone.setCamera(++mode%4);

        // Display the image
        cvShowImage("camera", image);
        cvWaitKey(1);
    }

    // See you
    ardrone.close();

    return 0;
}