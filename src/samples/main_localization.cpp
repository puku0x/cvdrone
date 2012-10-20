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

    // Kalman filter
    CvKalman* kalman = cvCreateKalman(6, 3, 0);

    // Transition matrix
    const float A[] = {1, 0, 0, 1, 0, 0,
                       0, 1, 0, 0, 1, 0,
                       0, 0, 1, 0, 0, 1,
                       0, 0, 0, 1, 0, 0,
                       0, 0, 0, 0, 1, 0,
                       0, 0, 0, 0, 0, 1};
    memcpy(kalman->transition_matrix->data.fl, A, sizeof(A));

    // Other matrices
    cvSetIdentity(kalman->measurement_matrix,    cvRealScalar(1.0));
    cvSetIdentity(kalman->process_noise_cov,     cvRealScalar(1e-5));
    cvSetIdentity(kalman->measurement_noise_cov, cvRealScalar(1e-1));
    cvSetIdentity(kalman->error_cov_post,        cvRealScalar(1.0));

    // Map
    IplImage *map = cvCreateImage(cvSize(500, 500), IPL_DEPTH_8U, 3);
    cvZero(map);

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

        // Time
        static double last = ardGetTickCount();
        double dt = ardGetTickCount() - last;
        last = ardGetTickCount();

        // Measurements
        float m[] = {vx*dt*cos(yaw) - vy*dt*sin(yaw), vx*dt*sin(yaw) + vy*dt*cos(yaw), vz};
        CvMat measurement = cvMat(3, 1, CV_32FC1, m);

        // Prediction
        const CvMat* prediction = cvKalmanPredict(kalman);
        double x = kalman->state_pre->data.fl[0];
        double y = kalman->state_pre->data.fl[1];
        double z = kalman->state_pre->data.fl[2];
        cvDrawText(image, cvPoint(20, 20), "x = %3.2f, y = %3.2f, z = %3.2f", x, y, z);
        
        // Draw
        CvPoint pos = cvPoint(-y*10.0 + map->width/2, -x*10.0 + map->height/2);
        cvDrawCircle(map, pos, 2, CV_RGB(255,0,0));
        cvShowImage("map", map);

        // Correction
        const CvMat* correction = cvKalmanCorrect(kalman, &measurement);

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
        if (KEY_PUSH('C')) {
            // AR.Drone 2.0
            if (ardrone.getVersion() == ARDRONE_VERSION_2) {
                if (mode == 0) mode = 1;    // Vertical
                else           mode = 0;    // Horizontal
            }
            // AR.Drone 1.0
            else {
                if (mode == 0) mode = 2;    // Horizontal + Vertiacal
                else           mode = 0;
            }
            ardrone.setCamera(mode);
        }

        // Display the image
        cvShowImage("camera", image);
        cvWaitKey(1);
    }

    // Release
    cvReleaseImage(&map);
    cvReleaseKalman(&kalman);

    // See you
    ardrone.close();

    return 0;
}