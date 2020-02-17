#include "ardrone.h"

// The code decoding H.264 video is based on the following sites.
// - An ffmpeg and SDL Tutorial - Tutorial 01: Making Screencaps -
//   http://dranger.com/ffmpeg/tutorial01.html
// - AR.Drone Development - 2.1.2 AR.Drone 2.0 Video Decording: FFMPEG + SDL2.0 -
//   http://ardrone-ailab-u-tokyo.blogspot.jp/2012/07/212-ardrone-20-video-decording-ffmpeg.html

// --------------------------------------------------------------------------
//! @brief   Initialize video.
//! @return  Result of initialization
//! @retval  1 Success
//! @retval  0 Failure
// --------------------------------------------------------------------------
using namespace cv;
using namespace std;


pair<Mat, vector<Vec3f>> ARDrone::detectCircle(Mat image){
    //ref = http://opencv.jp/opencv-2svn/cpp/feature_detection.html

    Mat gray;
    cvtColor(image, gray, COLOR_BGR2GRAY);

    GaussianBlur(gray, gray, Size(9,9), 2, 2);
    vector<Vec3f> circles;
    HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 2, gray.rows / 4, 200, 100);

    for(size_t i = 0;i < circles.size();i++){
		
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radious = cvRound(circles[i][2]);

        circle(image, center, 3, Scalar(0, 255, 0), -1, 8, 0);

        circle(image, center, radious, Scalar(0, 0, 255), 3, 8, 0);

    }
    return std::make_pair(image,circles);
}
