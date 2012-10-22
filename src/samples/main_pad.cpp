#include "ardrone/ardrone.h"

#define KEY_DOWN(key) (GetAsyncKeyState(key) & 0x8000)
#define KEY_PUSH(key) (GetAsyncKeyState(key) & 0x0001)

// --------------------------------------------------------------------------
// main(Number of arguments, Value of arguments)
// This is the main function.
// Return value Success:0 Error:-1
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

    // Main loop
    while (!GetAsyncKeyState(VK_ESCAPE)) {
        // Update your AR.Drone
        if (!ardrone.update()) break;

        // Get an image
        IplImage *image = ardrone.getImage();

        // Battery
        printf("ardrone.battery = %d [Åì] (écÇËñÒ%dï™)\n", battery, 12*battery/100);

        // Take off / Landing
        if (KEY_PUSH(VK_SPACE)) {
            if (ardrone.onGround()) ardrone.takeoff();
            else                    ardrone.landing();
        }

        // Emergency stop
        if (KEY_PUSH(VK_RETURN)) ardrone.emergency();

        // AR.Drone is flying
        if (!ardrone.onGround()) {
            double x = 0.0, y = 0.0, z = 0.0, r = 0.0;

            // Keyboard
            if (KEY_DOWN(VK_UP))    x =  0.5;
            if (KEY_DOWN(VK_DOWN))  x = -0.5;
            if (KEY_DOWN(VK_LEFT))  r =  0.5;
            if (KEY_DOWN(VK_RIGHT)) r = -0.5;
            if (KEY_DOWN('Q'))      z =  0.5;
            if (KEY_DOWN('A'))      z = -0.5;

            // Joypad
            JOYINFOEX JoyInfoEx;
            JoyInfoEx.dwSize = sizeof(JOYINFOEX);
            JoyInfoEx.dwFlags = JOY_RETURNALL;

            // Get joypad infomations
            if (joyGetPosEx(0, &JoyInfoEx) == JOYERR_NOERROR) {
                int y_pad = -((int)JoyInfoEx.dwXpos - 0x7FFF) / 32512.0*100.0;
                int x_pad = -((int)JoyInfoEx.dwYpos - 0x7FFF) / 32512.0*100.0;
                int r_pad = -((int)JoyInfoEx.dwZpos - 0x7FFF) / 32512.0*100.0;
                int z_pad =  ((int)JoyInfoEx.dwRpos - 0x7FFF) / 32512.0*100.0;

                printf("X = %d  ", x_pad);
                printf("Y = %d  ", y_pad);
                printf("Z = %d  ", z_pad);
                printf("R = %d\n", r_pad);

                x = 0.5 * x_pad / 100;
                y = 0.5 * y_pad / 100;
                z = 0.5 * z_pad / 100;
                r = 0.5 * r_pad / 100;

                if (JoyInfoEx.dwButtons & JOY_BUTTON1) ardrone.takeoff();
                if (JoyInfoEx.dwButtons & JOY_BUTTON2) ardrone.landing();
            }

            // Move
		    ardrone.move3D(x, y, z, r);
        }


        // Display the image
        cvShowImage("camera", image);
        cvWaitKey(1);
    }

    // See you
    ardrone.close();

    return 0;
}

