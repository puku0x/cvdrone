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

    // Battery
    std::cout << "Battery = " << ardrone.getBatteryPercentage() << " [%]" << std::endl;

    // Map
    cv::Mat map = cv::Mat::zeros(500, 500, CV_8UC3);

    // Position matrix
    cv::Mat P = cv::Mat::zeros(3, 1, CV_64FC1);

    // Main loop
    while (1) {
        // Key input
        int key = cv::waitKey(33);
        if (key == 0x1b) break;

        // Get an image
        cv::Mat image = ardrone.getImage();

        // Altitude
        double altitude = ardrone.getAltitude();

        // Orientations
        double roll = ardrone.getRoll();
        double pitch = ardrone.getPitch();
        double yaw = ardrone.getYaw();

        // Velocities
        double vx, vy, vz;
        double velocity = ardrone.getVelocity(&vx, &vy, &vz);
        cv::Mat V = (cv::Mat1f(3, 1) << vx, vy, vz);

        // Rotation matrices
        cv::Mat RZ = (cv::Mat1f(3, 3) << cos(yaw), -sin(yaw), 0.0,
                                         sin(yaw),  cos(yaw), 0.0,
                                              0.0,       0.0, 1.0);
        cv::Mat RY = (cv::Mat1f(3, 3) << cos(pitch), 0.0, sin(pitch),
                                                0.0, 1.0,        0.0,
                                        -sin(pitch), 0.0, cos(pitch));
        cv::Mat RX = (cv::Mat1f(3, 3) << 1.0,       0.0,        0.0,
                                         0.0, cos(roll), -sin(roll),
                                         0.0, sin(roll),  cos(roll));

        // Time [s]
        static int64 last = cv::getTickCount();
        double dt = (cv::getTickCount() - last) / cv::getTickFrequency();
        last = cv::getTickCount();

        // Dead-reckoning
        P = P + RZ * RY * RX * V * dt;

        // Position (x, y, z)
        double pos[3] = { P.at<double>(0, 0), P.at<double>(1, 0), P.at<double>(2, 0) };
        std::cout << "x = " << pos[0] << "[m], " << "y = " << pos[1] << "[m], " << "z = " << pos[2] << "[m]" << std::endl;

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
        if (key == 'c') ardrone.setCamera(++mode % 4);

        // Display the image
        cv::circle(map, cv::Point(-pos[1] * 100.0 + map.cols / 2, -pos[0] * 100.0 + map.rows / 2), 2, CV_RGB(255, 0, 0));
        cv::imshow("map", map);
        cv::imshow("camera", image);
    }

    // See you
    ardrone.close();

    return 0;
}