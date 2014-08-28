// C++ STL
#include <iostream>
#include <fstream>

// OpenCV
#include <opencv2/opencv.hpp>

// OpenGL
#include <GL/glut.h>

// AR.Drone
#include "ardrone/ardrone.h"

// Marker detector
#include ".\3rdparty\packtpub\MarkerDetector.hpp"

// Parameter for calibration pattern
#define PAT_ROWS   (7)                  // Rows of pattern
#define PAT_COLS   (10)                 // Columns of pattern
#define CHESS_SIZE (24.0)               // Size of a pattern [mm]

// Global variables
ARDrone ardrone;
cv::Mat mapx, mapy;
CameraCalibration calibration;

// --------------------------------------------------------------------------
// buildProjectionMatrix(Camera matrix, Screen width, Screen height)
// Description  : Calculate projection matrix from camera and screen paremeters.
// Return value : Projection matrix
// --------------------------------------------------------------------------
Matrix44 buildProjectionMatrix(Matrix33 cameraMatrix, int screen_width, int screen_height)
{
	float d_near = 0.01;  // Near clipping distance
	float d_far = 100;    // Far clipping distance

	// Camera parameters
	float f_x = cameraMatrix.data[0]; // Focal length in x axis
	float f_y = cameraMatrix.data[4]; // Focal length in y axis (usually the same?)
	float c_x = cameraMatrix.data[2]; // Camera primary point x
	float c_y = cameraMatrix.data[5]; // Camera primary point y

	Matrix44 projectionMatrix;
	projectionMatrix.data[0] = -2.0 * f_x / screen_width;
	projectionMatrix.data[1] = 0.0;
	projectionMatrix.data[2] = 0.0;
	projectionMatrix.data[3] = 0.0;

	projectionMatrix.data[4] = 0.0;
	projectionMatrix.data[5] = 2.0 * f_y / screen_height;
	projectionMatrix.data[6] = 0.0;
	projectionMatrix.data[7] = 0.0;

	projectionMatrix.data[8] = 2.0 * c_x / screen_width - 1.0;
	projectionMatrix.data[9] = 2.0 * c_y / screen_height - 1.0;
	projectionMatrix.data[10] = -(d_far + d_near) / (d_far - d_near);
	projectionMatrix.data[11] = -1.0;

	projectionMatrix.data[12] = 0.0;
	projectionMatrix.data[13] = 0.0;
	projectionMatrix.data[14] = -2.0 * d_far * d_near / (d_far - d_near);
	projectionMatrix.data[15] = 0.0;

	return projectionMatrix;
}

// --------------------------------------------------------------------------
// idle()
// Description  : Idle function.
// Return value : NONE
// --------------------------------------------------------------------------
void idle(void)
{
	// Redisplay
	glutPostRedisplay();
}

// --------------------------------------------------------------------------
// display()
// Description  : Displaying function.
// Return value : NONE
// --------------------------------------------------------------------------
void display(void)
{
	// Clear the buffers
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Get an image
	cv::Mat image_raw = ardrone.getImage();
	cv::Mat image;
	cv::remap(image_raw, image, mapx, mapy, cv::INTER_LINEAR);

	// Show the image
	cv::Mat rgb;
	cv::cvtColor(image, rgb, cv::COLOR_BGR2RGB);
	cv::flip(rgb, rgb, 0);
	glDepthMask(GL_FALSE);
	glDrawPixels(rgb.cols, rgb.rows, GL_RGB, GL_UNSIGNED_BYTE, rgb.data);

	// Convert to BGRA
	cv::Mat bgra;
	cv::cvtColor(image, bgra, cv::COLOR_BGR2BGRA);

	// Prepare for marker detection
	BGRAVideoFrame frame;
	frame.width = bgra.cols;
	frame.height = bgra.rows;
	frame.data = bgra.data;
	frame.stride = bgra.step;

	// Detect marker(s)
	MarkerDetector detector(calibration);
	detector.processFrame(frame);
	std::vector<Transformation> transformations = detector.getTransformations();

	// Calculate projection matrix
	Matrix44 projectionMatrix = buildProjectionMatrix(calibration.getIntrinsic(), frame.width, frame.height);

	// Apply the projection matrix
	glMatrixMode(GL_PROJECTION);
	glLoadMatrixf(projectionMatrix.data);

	// Change to model view matrix mode
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// Enable depth mask
	glDepthMask(GL_TRUE);

	// Enable vertex array
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);

	// Push current model view matrix
	glPushMatrix();

	// Set line width
	glLineWidth(3.0f);

	// Vertex arrays
	float lineX[] = { 0, 0, 0, 1, 0, 0 };
	float lineY[] = { 0, 0, 0, 0, 1, 0 };
	float lineZ[] = { 0, 0, 0, 0, 0, 1 };

	// 2D plane
	const GLfloat squareVertices[] = {-0.5f, -0.5f,
									   0.5f, -0.5f,
									  -0.5f,  0.5f,
									   0.5f,  0.5f};

	// 2D plane color (RGBA)
	const GLubyte squareColors[] = {255, 255,   0, 255,
								      0, 255, 255, 255,
									  0,   0,   0,   0,
									255,   0, 255, 255};

	// Draw AR
	for (size_t i = 0; i < transformations.size(); i++) {
		// Get transformation
		const Transformation &transformation = transformations[i];
		Matrix44 glMatrix = transformation.getMat44();

		// Load it
		glLoadMatrixf(reinterpret_cast<const GLfloat*>(&glMatrix.data[0]));

		// Draw 2D plane
		glEnableClientState(GL_COLOR_ARRAY);
		glVertexPointer(2, GL_FLOAT, 0, squareVertices);
		glColorPointer(4, GL_UNSIGNED_BYTE, 0, squareColors);
		glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
		glDisableClientState(GL_COLOR_ARRAY);

		// Scale of coordinate axes
		float scale = 0.5;
		glScalef(scale, scale, scale);

		// Move it a little
		glTranslatef(0, 0, 0.1f);

		// X axis
		glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
		glVertexPointer(3, GL_FLOAT, 0, lineX);
		glDrawArrays(GL_LINES, 0, 2);

		// Y axis
		glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
		glVertexPointer(3, GL_FLOAT, 0, lineY);
		glDrawArrays(GL_LINES, 0, 2);

		// Z axis
		glColor4f(0.0f, 0.0f, 1.0f, 1.0f);
		glVertexPointer(3, GL_FLOAT, 0, lineZ);
		glDrawArrays(GL_LINES, 0, 2);
	}

	// Disable vertex array
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);

	// Pop the model view matrix
	glPopMatrix();

	// Swap the buffer
	glutSwapBuffers();
}

// --------------------------------------------------------------------------
// key(Key pressed, X position of cursor, Y position of cursor)
// Description  : Key input function.
// Return value : NONE
// --------------------------------------------------------------------------
void key(unsigned char key, int x, int y) {
	switch (key) {
	case 0x1b:
		exit(1);
		break;
	default:
		break;
	}
}

// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    // Initialize
	if (!ardrone.open()) {
		std::cout << "Failed to initialize." << std::endl;
		return -1;
	}

    // Images
	cv::Mat frame = ardrone.getImage();

	// Open XML file
    std::string filename("camera.xml");
    std::fstream file(filename.c_str(), std::ios::in);

    // Not found
    if (!file.is_open()) {
        // Image buffer
		std::vector<cv::Mat> images;
		std::cout << "Press Space key to capture an image" << std::endl;
		std::cout << "Press Esc to exit" << std::endl;

		// Main loop
		while (1) {
			// Key iput
			int key = cv::waitKey(1);
			if (key == 0x1b) break;

			// Get an image
			frame = ardrone.getImage();

			// Convert to grayscale
			cv::Mat gray;
			cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

			// Detect a chessboard
			cv::Size size(PAT_COLS, PAT_ROWS);
			std::vector<cv::Point2f> corners;
			bool found = cv::findChessboardCorners(gray, size, corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);

			// Chessboard detected
			if (found) {
				// Draw it
				cv::drawChessboardCorners(frame, size, corners, found);

				// Space key was pressed
				if (key == ' ') {
					// Add to buffer
					images.push_back(gray);
				}
			}

			// Show the image
			std::ostringstream stream;
			stream << "Captured " << images.size() << " image(s).";
			cv::putText(frame, stream.str(), cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
			cv::imshow("Camera Calibration", frame);
		}

		// We have enough samples
		if (images.size() > 4) {
			cv::Size size(PAT_COLS, PAT_ROWS);
			std::vector<std::vector<cv::Point2f>> corners2D;
			std::vector<std::vector<cv::Point3f>> corners3D;

			for (size_t i = 0; i < images.size(); i++) {
				// Detect a chessboard
				std::vector<cv::Point2f> tmp_corners2D;
				bool found = cv::findChessboardCorners(images[i], size, tmp_corners2D);

				// Chessboard detected
				if (found) {
					// Convert the corners to sub-pixel
					cv::cornerSubPix(images[i], tmp_corners2D, cvSize(11, 11), cvSize(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.1));
					corners2D.push_back(tmp_corners2D);

					// Set the 3D position of patterns
					const float squareSize = CHESS_SIZE;
					std::vector<cv::Point3f> tmp_corners3D;
					for (int j = 0; j < size.height; j++) {
						for (int k = 0; k < size.width; k++) {
							tmp_corners3D.push_back(cv::Point3f((float)(k*squareSize), (float)(j*squareSize), 0.0));
						}
					}
					corners3D.push_back(tmp_corners3D);
				}
			}

			// Estimate camera parameters
			cv::Mat cameraMatrix, distCoeffs;
			std::vector<cv::Mat> rvec, tvec;
			cv::calibrateCamera(corners3D, corners2D, images[0].size(), cameraMatrix, distCoeffs, rvec, tvec);
			std::cout << cameraMatrix << std::endl;
			std::cout << distCoeffs << std::endl;

			// Save them
			cv::FileStorage fs(filename, cv::FileStorage::WRITE);
			fs << "intrinsic" << cameraMatrix;
			fs << "distortion" << distCoeffs;
		}

		// Destroy windows
		cv::destroyAllWindows();
	}

	// Open XML file
	cv::FileStorage rfs(filename, cv::FileStorage::READ);
	if (!rfs.isOpened()) {
		std::cout << "Failed to open the XML file" << std::endl;
		return -1;
	}

	// Load camera parameters
	cv::Mat cameraMatrix, distCoeffs;
	rfs["intrinsic"] >> cameraMatrix;
	rfs["distortion"] >> distCoeffs;

	// Create undistort map
	cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cameraMatrix, frame.size(), CV_32FC1, mapx, mapy);

	// Set camera parameters
	float fx = cameraMatrix.at<double>(0, 0);
	float fy = cameraMatrix.at<double>(1, 1);
	float cx = cameraMatrix.at<double>(0, 2);
	float cy = cameraMatrix.at<double>(1, 2);
	//calibration = CameraCalibration(fx, fy, cx, cy);
    calibration = CameraCalibration(fx, fy, frame.cols / 2, frame.rows / 2);

	// Initialize GLUT
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(frame.cols, frame.rows);
	glutCreateWindow("Mastering OpenCV with Practical Computer Vision Project");
	glutDisplayFunc(display);
	glutKeyboardFunc(key);
	glutIdleFunc(idle);

	// Clea scene
	glClearColor(0.0, 0.0, 1.0, 1.0);
	glEnable(GL_DEPTH_TEST);

	// Start main loop
	glutMainLoop();

	return 0;
}