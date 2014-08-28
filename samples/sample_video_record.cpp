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

    // Recording flag
    bool rec = false;
    std::cout << "Press 'R' to start/stop recording." << std::endl;

    // Main loop
    while (1) {
        // Key input
        int key = cv::waitKey(1);
        if (key == 0x1b) break;

        // Video recording start / stop
        if (key == 'r') {
            rec = !rec;
            ardrone.setVideoRecord(rec);
        }

        // Get an image
        cv::Mat image = ardrone.getImage();

        // Show recording state
        if (rec) cv::putText(image, "REC", cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

        // Display the image
        cv::imshow("camera", image);
    }

    // See you
    ardrone.close();

    return 0;
}