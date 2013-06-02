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

    // Kalman filter
    CvKalman *kalman = cvCreateKalman(4, 2);

    // Setup
    cvSetIdentity(kalman->measurement_matrix, cvRealScalar(1.0));
    cvSetIdentity(kalman->process_noise_cov, cvRealScalar(1e-5));
    cvSetIdentity(kalman->measurement_noise_cov, cvRealScalar(0.1));
    cvSetIdentity(kalman->error_cov_post, cvRealScalar(1.0));

    // Linear system
    kalman->DynamMatr[0]  = 1.0; kalman->DynamMatr[1]  = 0.0; kalman->DynamMatr[2]  = 1.0; kalman->DynamMatr[3]  = 0.0; 
    kalman->DynamMatr[4]  = 0.0; kalman->DynamMatr[5]  = 1.0; kalman->DynamMatr[6]  = 0.0; kalman->DynamMatr[7]  = 1.0; 
    kalman->DynamMatr[8]  = 0.0; kalman->DynamMatr[9]  = 0.0; kalman->DynamMatr[10] = 1.0; kalman->DynamMatr[11] = 0.0; 
    kalman->DynamMatr[12] = 0.0; kalman->DynamMatr[13] = 0.0; kalman->DynamMatr[14] = 0.0; kalman->DynamMatr[15] = 1.0; 

    // Thresholds
    int minH = 0, maxH = 255;
    int minS = 0, maxS = 255;
    int minV = 0, maxV = 255;

    // Create a window
    cvNamedWindow("binalized");
    cvCreateTrackbar("H max", "binalized", &maxH, 255);
    cvCreateTrackbar("H min", "binalized", &minH, 255);
    cvCreateTrackbar("S max", "binalized", &maxS, 255);
    cvCreateTrackbar("S min", "binalized", &minS, 255);
    cvCreateTrackbar("V max", "binalized", &maxV, 255);
    cvCreateTrackbar("V min", "binalized", &minV, 255);
    cvResizeWindow("binalized", 0, 0);

    // Main loop
    while (1) {
        // Key input
        int key = cvWaitKey(1);
        if (key == 0x1b) break;

        // Update
        if (!ardrone.update()) break;

        // Get an image
        IplImage *image = ardrone.getImage();

        // HSV image
        IplImage *hsv = cvCloneImage(image);
        cvCvtColor(image, hsv, CV_RGB2HSV_FULL);

        // Binalized image
        IplImage *binalized = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);

        // Binalize
        CvScalar lower = cvScalar(minH, minS, minV);
        CvScalar upper = cvScalar(maxH, maxS, maxV);
        cvInRangeS(image, lower, upper, binalized);

        // Show result
        cvShowImage("binalized", binalized);

        // De-noising
        cvMorphologyEx(binalized, binalized, NULL, NULL, CV_MOP_CLOSE);
 
        // Detect contours
        CvSeq *contour = NULL, *maxContour = NULL;
        CvMemStorage *contourStorage = cvCreateMemStorage();
        cvFindContours(binalized, contourStorage, &contour, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

        // Find largest contour
        double max_area = 0.0;
        while (contour) {
            double area = fabs(cvContourArea(contour));
            if ( area > max_area) {
                maxContour = contour;
                max_area = area;
            }
            contour = contour->h_next;
        }

        // Object detected
        if (maxContour) {
            // Draw a contour
            cvZero(binalized);
            cvDrawContours(binalized, maxContour, cvScalarAll(255), cvScalarAll(255), 0, CV_FILLED);

            // Calculate the moments
            CvMoments moments;
            cvMoments(binalized, &moments, 1);
            int my = (int)(moments.m01/moments.m00);
            int mx = (int)(moments.m10/moments.m00);

            // Measurements
            float m[] = {mx, my};
            CvMat measurement = cvMat(2, 1, CV_32FC1, m);

            // Correct phase
            const CvMat *correction = cvKalmanCorrect(kalman, &measurement);
        }

        // Prediction phase
        const CvMat *prediction = cvKalmanPredict(kalman);

        // Display the image
        cvCircle(image, cvPointFrom32f(cvPoint2D32f(prediction->data.fl[0], prediction->data.fl[1])), 10, CV_RGB(0,255,0));
        cvShowImage("camera", image);

        // Release the memories
        cvReleaseImage(&hsv);
        cvReleaseImage(&binalized);
        cvReleaseMemStorage(&contourStorage);
    }

    // Release the kalman filter
    cvReleaseKalman(&kalman);

    // See you
    ardrone.close();

    return 0;
}