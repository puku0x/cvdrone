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

        // Orientation
        double roll  = ardrone.getRoll();
        double pitch = ardrone.getPitch();
        double yaw   = ardrone.getYaw();
        printf("ardrone.roll  = %3.2f [deg]\n", roll  * RAD_TO_DEG);
        printf("ardrone.pitch = %3.2f [deg]\n", pitch * RAD_TO_DEG);
        printf("ardrone.yaw   = %3.2f [deg]\n", yaw   * RAD_TO_DEG);

        // Altitude
        double altitude = ardrone.getAltitude();
        printf("ardrone.altitude = %3.2f [m]\n", altitude);

        // Velocity
        double vx, vy, vz;
        double velocity = ardrone.getVelocity(&vx, &vy, &vz);
        printf("ardrone.vx = %3.2f [m/s]\n", vx);
        printf("ardrone.vy = %3.2f [m/s]\n", vy);
        printf("ardrone.vz = %3.2f [m/s]\n", vz);

        // Battery
        int battery = ardrone.getBatteryPercentage();
        printf("ardrone.battery = %d [%%]\n", battery);

        // Take off / Landing
        if (KEY_PUSH(VK_SPACE)) {
            if (ardrone.onGround()) ardrone.takeoff();
            else                    ardrone.landing();
        }

        // Emergency stop
        if (KEY_PUSH(VK_RETURN)) ardrone.emergency();

        // AR.Drone is flying
        if (!ardrone.onGround()) {
            // Move
            double x = 0.0, y = 0.0, z = 0.0, r = 0.0;
            if (KEY_DOWN(VK_UP))    x =  0.5;
            if (KEY_DOWN(VK_DOWN))  x = -0.5;
            if (KEY_DOWN(VK_LEFT))  r =  0.5;
            if (KEY_DOWN(VK_RIGHT)) r = -0.5;
            if (KEY_DOWN('Q'))      z =  0.5;
            if (KEY_DOWN('A'))      z = -0.5;
            ardrone.move3D(x, y, z, r);
        }

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