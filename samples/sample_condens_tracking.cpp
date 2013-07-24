#include "ardrone/ardrone.h"
#include "opencv2/legacy/legacy.hpp"
#include "opencv2/legacy/compat.hpp"

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

    // Particle filter
    CvConDensation *con = cvCreateConDensation(4, 0, 3000);

    // Setup
    CvMat *lowerBound = cvCreateMat(4, 1, CV_32FC1);
    CvMat *upperBound = cvCreateMat(4, 1, CV_32FC1);
    cvmSet(lowerBound, 0, 0, 0);
    cvmSet(lowerBound, 1, 0, 0);
    cvmSet(lowerBound, 2, 0, -10);
    cvmSet(lowerBound, 3, 0, -10);
    cvmSet(upperBound, 0, 0, ardrone.getImage()->width);
    cvmSet(upperBound, 1, 0, ardrone.getImage()->height);
    cvmSet(upperBound, 2, 0, 10);
    cvmSet(upperBound, 3, 0, 10);

    // Initialize particle filter
    cvConDensInitSampleSet(con, lowerBound, upperBound);

    // Linear system
    con->DynamMatr[0]  = 1.0; con->DynamMatr[1]  = 0.0; con->DynamMatr[2]  = 1.0; con->DynamMatr[3]  = 0.0; 
    con->DynamMatr[4]  = 0.0; con->DynamMatr[5]  = 1.0; con->DynamMatr[6]  = 0.0; con->DynamMatr[7]  = 1.0; 
    con->DynamMatr[8]  = 0.0; con->DynamMatr[9]  = 0.0; con->DynamMatr[10] = 1.0; con->DynamMatr[11] = 0.0; 
    con->DynamMatr[12] = 0.0; con->DynamMatr[13] = 0.0; con->DynamMatr[14] = 0.0; con->DynamMatr[15] = 1.0; 

    // Noises
    cvRandInit(&(con->RandS[0]), -25, 25, (int)cvGetTickCount());
    cvRandInit(&(con->RandS[1]), -25, 25, (int)cvGetTickCount());
    cvRandInit(&(con->RandS[2]),  -5,  5, (int)cvGetTickCount());
    cvRandInit(&(con->RandS[3]),  -5,  5, (int)cvGetTickCount());

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
            cvCircle(image, cvPoint(mx, my), 10, CV_RGB(255,0,0));

            // Calculate confidences
            for (int i = 0; i < con->SamplesNum; i++) {
                // Sample points
                float x = (con->flSamples[i][0]);
                float y = (con->flSamples[i][1]);

                // Valid sample point
                if (x > 0 && x < image->width && y > 0 && y < image->height) {
                    // Assume as gauss distribution
                    double sigma = 50.0;
                    double dist = hypot(x - mx, y - my);    // Distance to moment
                    con->flConfidence[i] = 1.0 / (sqrt (2.0 * CV_PI) * sigma) * expf (-dist*dist / (2.0 * sigma*sigma));
                }
                else con->flConfidence[i] = 0.0;
                cvCircle(image, cvPointFrom32f(cvPoint2D32f(x, y)), 3, CV_RGB(0,128,con->flConfidence[i] * 50000));
            }
        }

        // Update phase
        cvConDensUpdateByTime(con);

        // Sum of positions and confidences for calcurate weighted mean value
        double sumX = 0, sumY = 0, sumConf = 0;
        for (int i = 0; i < con->SamplesNum; i++) {
            sumX += con->flConfidence[i] * con->flSamples[i][0];
            sumY += con->flConfidence[i] * con->flSamples[i][1];
            sumConf += con->flConfidence[i];
        }

        // Estimated value
        if (sumConf > 0.0) {
            float x = sumX / sumConf;
            float y = sumY / sumConf;
            cvCircle(image, cvPointFrom32f(cvPoint2D32f(x, y)), 10, CV_RGB(0,255,0));
        }

        // Display the image
        cvShowImage("camera", image);

        // Release memories
        cvReleaseImage(&hsv);
        cvReleaseImage(&binalized);
        cvReleaseMemStorage(&contourStorage);
    }

    // Release the particle filter
    cvReleaseMat(&lowerBound);
    cvReleaseMat(&upperBound);
    cvReleaseConDensation(&con);

    // See you
    ardrone.close();

    return 0;
}