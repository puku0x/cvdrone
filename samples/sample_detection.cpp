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
        int key = cvWaitKey(33);
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
            if (area > max_area) {
                maxContour = contour;
                max_area = area;
            }
            contour = contour->h_next;
        }

        // Object detected
        if (maxContour) {
            // Show result
            CvRect rect = cvBoundingRect(maxContour);
            CvPoint minPoint, maxPoint;
            minPoint.x = rect.x;
            minPoint.y = rect.y;
            maxPoint.x = rect.x + rect.width;
            maxPoint.y = rect.y + rect.height;
            cvRectangle(image, minPoint, maxPoint, CV_RGB(0,255,0));
        }

        // Release memory
        cvReleaseMemStorage(&contourStorage);

        // Display the image
        cvShowImage("camera", image);

        // Release images
        cvReleaseImage(&hsv);
        cvReleaseImage(&binalized);
    }

    // See you
    ardrone.close();

    return 0;
}