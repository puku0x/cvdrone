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

    // Main loop
    while (1) {
        // Key input
        int key = cvWaitKey(33);
        if (key == 0x1b) break;

        // Update
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
        if (key == ' ') {
            if (ardrone.onGround()) ardrone.takeoff();
            else                    ardrone.landing();
        }

        // Move
        double x = 0.0, y = 0.0, z = 0.0, r = 0.0;
        if (key == 0x260000) x =  1.0;
        if (key == 0x280000) x = -1.0;
        if (key == 0x250000) r =  1.0;
        if (key == 0x270000) r = -1.0;
        ardrone.move3D(x, y, z, r);

        // Change camera
        static int mode = 0;
        if (key == 'c') ardrone.setCamera(++mode%4);

        // Display the image
        cvShowImage("camera", image);
    }

    // See you
    ardrone.close();

    return 0;
}