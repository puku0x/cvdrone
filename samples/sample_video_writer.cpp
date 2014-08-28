#include "ardrone/ardrone.h"

// For std::localtime();
#include <ctime>

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

    // Image of AR.Drone's camera
    cv::Mat image = ardrone.getImage();
    
    // Video name
    std::time_t t = std::time(NULL);
    std::tm *local = std::localtime(&t);
    std::ostringstream stream;
    stream << 1900 + local->tm_year << "-" << 1 + local->tm_mon << "-" << local->tm_mday << "-" << local->tm_hour << "-" << local->tm_min << "-" << local->tm_sec << ".avi";

    // Create a video writer
    cv::VideoWriter writer(stream.str(), cv::VideoWriter::fourcc('D', 'I', 'B', ' '), 30, cv::Size(image.cols, image.rows));

    // Main loop
    while (1) {
        // Key input
        int key = cv::waitKey(33);
        if (key == 0x1b) break;

        // Get an image
        image = ardrone.getImage();

        // Write a frame
        writer << image;

        // Display the image
        imshow("camera", image);
    }

    // Output the video
    writer.release();

    // See you
    ardrone.close();

    return 0;
}