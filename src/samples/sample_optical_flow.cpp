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

    // Image of AR.Drone's camera
    IplImage *image = ardrone.getImage();

    // Variables for optical flow
    int corner_count = 50;
    IplImage *gray = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);
    IplImage *prev = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);
    cvCvtColor(image, prev, CV_BGR2GRAY);
    IplImage *eig_img = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 1);
    IplImage *tmp_img = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 1);
    IplImage *prev_pyramid = cvCreateImage(cvSize(image->width+8, image->height/3), IPL_DEPTH_8U, 1);
    IplImage *curr_pyramid = cvCreateImage(cvSize(image->width+8, image->height/3), IPL_DEPTH_8U, 1);
    CvPoint2D32f *corners1 = (CvPoint2D32f*)malloc(corner_count * sizeof(CvPoint2D32f));
    CvPoint2D32f *corners2 = (CvPoint2D32f*)malloc(corner_count * sizeof(CvPoint2D32f));

    // Main loop
    while (1) {
        // Key input
        int key = cvWaitKey(30);
        if (key == 0x1b) break;

        // Update
        if (!ardrone.update()) break;

        // Get an image
        image = ardrone.getImage();

        // Take off / Landing 
        if (key == ' ') {
            if (ardrone.onGround()) ardrone.takeoff();
            else                    ardrone.landing();
        }

        // Move
        double vx = 0.0, vy = 0.0, vz = 0.0, vr = 0.0;
        if (key == 0x260000) vx =  0.5;
        if (key == 0x280000) vx = -0.5;
        if (key == 0x250000) vr =  0.5;
        if (key == 0x270000) vr = -0.5;
        if (key == 'q')      vz =  0.5;
        if (key == 'a')      vz = -0.5;
        ardrone.move3D(vx, vy, vz, vr);

        // Convert the camera image to grayscale
        cvCvtColor(image, gray, CV_BGR2GRAY);

        // Detect features
        int corner_count = 50;
        cvGoodFeaturesToTrack(prev, eig_img, tmp_img, corners1, &corner_count, 0.1, 5.0, NULL);

        // Corner detected
        if (corner_count > 0) {
            char *status = (char*)malloc(corner_count * sizeof(char));

            // Calicurate optical flows
            CvTermCriteria criteria = cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.3);
            cvCalcOpticalFlowPyrLK(prev, gray, prev_pyramid, curr_pyramid, corners1, corners2, corner_count, cvSize(10, 10), 3, status, NULL, criteria, 0);

            // Drow the optical flows
            for (int i = 0; i < corner_count; i++) {
                cvCircle(image, cvPointFrom32f(corners1[i]), 1, CV_RGB (255, 0, 0));
                if (status[i]) cvLine(image, cvPointFrom32f(corners1[i]), cvPointFrom32f(corners2[i]), CV_RGB (0, 0, 255), 1, CV_AA, 0);
            }

            // Release the memory
            free(status);
        }

        // Save the last frame
        cvCopy(gray, prev);

        // Display the image
        cvShowImage("camera", image);
    }

    // Release the images
    cvReleaseImage(&gray);
    cvReleaseImage(&prev);
    cvReleaseImage(&eig_img);
    cvReleaseImage(&tmp_img);
    cvReleaseImage(&prev_pyramid);
    cvReleaseImage(&curr_pyramid);
    free(corners1);
    free(corners2);

    // See you
    ardrone.close();

    return 0;
}