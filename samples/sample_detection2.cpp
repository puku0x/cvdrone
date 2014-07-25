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
        std::cout << "Failed to initialize." << std::endl;
        return -1;
    }

    // Thresholds
    int minH = 0, maxH = 255;
    int minS = 0, maxS = 255;
    int minV = 0, maxV = 255;

    // Create a window
    cv::namedWindow("binalized");
    cv::createTrackbar("H max", "binalized", &maxH, 255);
    cv::createTrackbar("H min", "binalized", &minH, 255);
    cv::createTrackbar("S max", "binalized", &maxS, 255);
    cv::createTrackbar("S min", "binalized", &minS, 255);
    cv::createTrackbar("V max", "binalized", &maxV, 255);
    cv::createTrackbar("V min", "binalized", &minV, 255);
    cv::resizeWindow("binalized", 0, 0);

    // Main loop
    while (1) {
        // Key input
        int key = cv::waitKey(33);
        if (key == 0x1b) break;

        // Update
        if (!ardrone.update()) break;

        // Get an image
        cv::Mat image = ardrone.getImage();

        // HSV image
        cv::Mat hsv;
        cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV_FULL);

        // Binalize
        cv::Mat binalized;
        cv::Scalar lower(minH, minS, minV);
        cv::Scalar upper(maxH, maxS, maxV);
        cv::inRange(hsv, lower, upper, binalized);

        // Show result
        cv::imshow("binalized", binalized);

        // De-noising
        cv::Mat kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(binalized, binalized, cv::MORPH_CLOSE, kernel);
        //cv::imshow("morphologyEx", binalized);

        // Detect contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binalized.clone(), contours, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

        // Find largest contour
        int contour_index = -1;
        double max_area = 0.0;
        for (int i = 0; i < contours.size(); i++) {
            double area = fabs(cv::contourArea(contours[i]));
            if (area > max_area) {
                contour_index = i;
                max_area = area;
            }
        }

        // Object detected
        if (contour_index >= 0) {
            // Show result
            cv::Rect rect = cv::boundingRect(contours[contour_index]);
            cv::rectangle(image, rect, cv::Scalar(0,255,0));
            //cv::drawContours(image, contours, contour_index, cv::Scalar(0,255,0));
        }

        // Display the image
        cv::imshow("camera", image);
    }

    // See you
    ardrone.close();

    return 0;
}