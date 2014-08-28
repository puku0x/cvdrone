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

    // Get an image
    cv::Mat prev_image = ardrone.getImage();

    while (1) {
        // Key input
        int key = cv::waitKey(33);
        if (key == 0x1b) break;

        // Get an image
        cv::Mat image = ardrone.getImage();

        // Convert the camera image to grayscale
        cv::Mat prev_gray, new_gray;
        cv::cvtColor(image, new_gray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(prev_image, prev_gray, cv::COLOR_BGR2GRAY);

        // Detect corners
        int max_corners = 50;
        std::vector<cv::Point2f> prev_corners;
        std::vector<cv::Point2f> new_corners;
        cv::goodFeaturesToTrack(prev_gray, prev_corners, max_corners, 0.1, 5.0);
        cv::goodFeaturesToTrack(new_gray, new_corners, max_corners, 0.1, 5.0);

        // Calclate optical flow
        std::vector<unsigned char> status;
        std::vector<float> errors;
        cv::calcOpticalFlowPyrLK(prev_gray, new_gray, prev_corners, new_corners, status, errors);

        // Save the last frame
        image.copyTo(prev_image);

        // Draw optical flow
        for (size_t i = 0; i < status.size(); i++) {
            cv::Point p0(ceil(prev_corners[i].x), ceil(prev_corners[i].y));
            cv::Point p1(ceil(new_corners[i].x), ceil(new_corners[i].y));
            cv::line(image, p0, p1, cv::Scalar(0, 255, 0), 2);
        }

        // Change camera
        static int mode = 0;
        if (key == 'c') ardrone.setCamera(++mode % 4);

        // Display the image
        cv::imshow("camera", image);
    }

    // See you
    ardrone.close();

    return 0;
}