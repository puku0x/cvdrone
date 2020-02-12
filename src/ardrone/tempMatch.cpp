// -------------------------------------------------------------------------
// CV Drone (= OpenCV + AR.Drone)
// Copyright(C) 2016 puku0x
// https://github.com/puku0x/cvdrone
//
// This source file is part of CV Drone library.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of EITHER:
// (1) The GNU Lesser General Public License as published by the Free
//     Software Foundation; either version 2.1 of the License, or (at
//     your option) any later version. The text of the GNU Lesser
//     General Public License is included with this library in the
//     file cvdrone-license-LGPL.txt.
// (2) The BSD-style license that is included with this library in
//     the file cvdrone-license-BSD.txt.
// 
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files
// cvdrone-license-LGPL.txt and cvdrone-license-BSD.txt for more details.
//
//! @file   video.cpp
//! @brief  Converting video into IplImage or cv::Mat
//
// -------------------------------------------------------------------------

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


Mat ARDrone::detectCircle(Mat image){
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
    return image;
}

// cv::Mat temp_match(cv::Mat image) {
// 	cv::Mat result_mat;
// 	cv::Mat gray_img;


// 	cv::cvtColor(image, gray_img, cv::COLOR_BGR2GRAY, 0);  //�J�����摜���O���[�X�P�[���ɕϊ�

// 	cv::matchTemplate(gray_img, tmpl, result_mat, CV_TM_CCORR_NORMED);
// 	cv::normalize(result_mat, result_mat, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

// 	double minVal; double maxVal;
// 	cv::Point minLoc, maxLoc, matchLoc;
// 	cv::minMaxLoc(result_mat, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());
// 	matchLoc = maxLoc;

// 	cv::rectangle(
// 		image,
// 		matchLoc,
// 		cv::Point(matchLoc.x + 0.7 * tmpl.cols, matchLoc.y + 0.7 * tmpl.rows),
// 		CV_RGB(255, 0, 0),
// 		3);

// 	return image;
// }