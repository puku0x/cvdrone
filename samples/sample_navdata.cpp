#include "ardrone/ardrone.h"

// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    // AR.Drone class
    ARDrone ardrone;

    // Initialize
    if (!ardrone.open()) {
        std::cout << "Failed to initialize." << std::endl;
        return -1;
    }

    // Main loop
    while (1) {
        // Key input
        int key = cv::waitKey(33);
        if (key == 0x1b) break;

        // Get an image
        cv::Mat image= ardrone.getImage();

        // Orientation
        double roll  = ardrone.getRoll();
        double pitch = ardrone.getPitch();
        double yaw   = ardrone.getYaw();
        std::cout << "ardrone.roll  = " << roll  * RAD_TO_DEG << " [deg]" << std::endl;
        std::cout << "ardrone.pitch = " << pitch * RAD_TO_DEG << " [deg]" << std::endl;
        std::cout << "ardrone.yaw   = " << yaw   * RAD_TO_DEG << " [deg]" << std::endl;

        // Altitude
        double altitude = ardrone.getAltitude();
        std::cout << "ardrone.altitude = " << altitude << " [m]" << std::endl;

        // Velocity
        double vx, vy, vz;
        double velocity = ardrone.getVelocity(&vx, &vy, &vz);
        std::cout << "ardrone.vx = " << vx << " [m/s]" << std::endl;
        std::cout << "ardrone.vy = " << vy << " [m/s]" << std::endl;
        std::cout << "ardrone.vz = " << vz << " [m/s]" << std::endl;

        // Battery
        int battery = ardrone.getBatteryPercentage();
        std::cout << "ardrone.battery = " << battery << " [%%]" << std::endl;

        // Take off / Landing 
        if (key == ' ') {
            if (ardrone.onGround()) ardrone.takeoff();
            else                    ardrone.landing();
        }

        // Move
        double x = 0.0, y = 0.0, z = 0.0, r = 0.0;
        if (key == 'i' || key == CV_VK_UP)    vx =  1.0;
        if (key == 'k' || key == CV_VK_DOWN)  vx = -1.0;
        if (key == 'u' || key == CV_VK_LEFT)  vr =  1.0;
        if (key == 'o' || key == CV_VK_RIGHT) vr = -1.0;
        if (key == 'j') vy =  1.0;
        if (key == 'l') vy = -1.0;
        if (key == 'q') vz =  1.0;
        if (key == 'a') vz = -1.0;
        ardrone.move3D(x, y, z, r);

        // Change camera
        static int mode = 0;
        if (key == 'c') ardrone.setCamera(++mode%4);

        // Display the image
        cv::imshow("camera", image);
    }

    // See you
    ardrone.close();

    return 0;
}