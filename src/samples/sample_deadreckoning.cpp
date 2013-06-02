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

    // Map
    IplImage *map = cvCreateImage(cvSize(500, 500), IPL_DEPTH_8U, 3);
    cvZero(map);

    // Position matrix
    cv::Mat P = cv::Mat::zeros(3, 1, CV_64FC1);

    // Main loop
    while (1) {
        // Key input
        int key = cvWaitKey(30);
        if (key == 0x1b) break;

        // Update
        if (!ardrone.update()) break;

        // Get an image
        IplImage *image = ardrone.getImage();

        // Orientation
        double roll  = ardrone.getRoll();
        double pitch = ardrone.getPitch();
        double yaw   = ardrone.getYaw();

        // Velocity
        double vx, vy, vz;
        double velocity = ardrone.getVelocity(&vx, &vy, &vz);

        // Rotation matrices
        double _RX[] = {        1.0,       0.0,        0.0,
                                0.0, cos(roll), -sin(roll),
                                0.0, sin(roll),  cos(roll)};
        double _RY[] = { cos(pitch),       0.0,  sin(pitch),
                                0.0,       1.0,        0.0,
                        -sin(pitch),       0.0,  cos(pitch)};
        double _RZ[] = {   cos(yaw), -sin(yaw),        0.0,
                           sin(yaw),  cos(yaw),        0.0,
                                0.0,       0.0,        1.0};
        cv::Mat RX(3, 3, CV_64FC1, _RX);
        cv::Mat RY(3, 3, CV_64FC1, _RY);
        cv::Mat RZ(3, 3, CV_64FC1, _RZ);

        // Time
        static int64 last = cv::getTickCount();
        double dt = (cv::getTickCount() - last) / cv::getTickFrequency();
        last = cv::getTickCount();

        // Local movement
        double _M[] = {vx * dt, vy * dt, vz * dt};
        cv::Mat M(3, 1, CV_64FC1, _M);

        // Dead reckoning
        P = P + RZ * RY * RX * M;

        // Position (x, y, z)
        double pos[3] = {P.at<double>(0,0), P.at<double>(1,0), P.at<double>(2,0)};
        printf("x = %3.2f, y = %3.2f, z = %3.2f", pos[0], pos[1], pos[2]);

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
        if (key == 'q')      z =  1.0;
        if (key == 'a')      z = -1.0;
        ardrone.move3D(x, y, z, r);

        // Change camera
        static int mode = 0;
        if (key == 'c') ardrone.setCamera(++mode%4);

        // Display the image
        cvDrawCircle(map, cvPoint(-pos[1]*30.0 + map->width/2, -pos[0]*30.0 + map->height/2), 2, CV_RGB(255,0,0));
        cvShowImage("map", map);
        cvShowImage("camera", image);
    }

    // See you
    ardrone.close();
    cvReleaseImage(&map);

    return 0;
}