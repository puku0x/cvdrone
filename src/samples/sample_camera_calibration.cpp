#include "ardrone/ardrone.h"

#define KEY_DOWN(key) (GetAsyncKeyState(key) & 0x8000)
#define KEY_PUSH(key) (GetAsyncKeyState(key) & 0x0001)

// Execute calibration
#define CALIB_MODE  1                   // ON:1 OFF:0

// Parameter for calibration pattern
#define PAT_ROW    (7)                  // Rows of pattern
#define PAT_COL    (10)                 // Columns of pattern
#define PAT_SIZE   (PAT_ROW*PAT_COL)
#define CHESS_SIZE (24.0)               // Size of a pattern [mm]

// --------------------------------------------------------------------------
// cvDrawText(Image, Drowing point, Messages)
// Description  : Draw the specified text.
// Return value : NONE
// --------------------------------------------------------------------------
inline void cvDrawText(IplImage *image, CvPoint point, const char *fmt, ...)
{
    // Font
    static CvFont font = cvFont(1.0);

    // Apply format
    char text[256];
    va_list ap;
    va_start(ap, fmt);
    vsprintf(text, fmt, ap);
    va_end(ap);

    // Draw the text
    cvPutText(image, text, point, &font, cvScalarAll(255));
}

// --------------------------------------------------------------------------
// cvAsk(Message)
// Description  : Show a question.
// Return value : NO:0 YES:1
// --------------------------------------------------------------------------
inline int cvAsk(const char *message, ...)
{
    char *arg;
    char str[256];

    // Apply format
    va_start(arg, message);
    vsprintf(str, message, arg);
    va_end(arg);

    // Show message box
    return (MessageBox(NULL, str, "QUESTION", MB_YESNO|MB_ICONQUESTION|MB_TOPMOST|MB_SETFOREGROUND) == IDYES);
}

#if CALIB_MODE
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

    // Images
    std::vector<IplImage*> images;
    printf("Press space key to take a sample picture !\n");

    // Main loop
    while (1) {
        // Key input
        int key = cvWaitKey(1);
        if (key == 0x1b) break;

        // Update
        if (!ardrone.update()) break;

        // Get an image
        IplImage *image = ardrone.getImage();

        // If you push Space key
        if (key == ' ') {
            // Convert the camera image to grayscale
            IplImage *gray = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);
            cvCvtColor(image, gray, CV_BGR2GRAY);

            // Detect the chessboard
            int corner_count = 0;
            CvSize size = cvSize(PAT_COL, PAT_ROW);
            CvPoint2D32f corners[PAT_SIZE];
            int found = cvFindChessboardCorners(gray, size, corners, &corner_count);

            // Detected
            if (found) {
                // Draw corners.
                cvDrawChessboardCorners(image, size, corners, corner_count, found);

                // Add to buffer
                images.push_back(gray);
                //Beep(3000, 100);
            }
            // Failed to detect
            else {
                // Release the image
                cvReleaseImage(&gray);
                //Beep(100, 100);
            }
        }

        // Display the image
        cvDrawText(image, cvPoint(15, 20), "NUM = %d", (int)images.size());
        cvShowImage("camera", image);
    }

    // Destroy the window
    cvDestroyWindow("camera");

    // At least one image was taken
    if (!images.empty()) {
        // Total number of images
        const int num = (int)images.size();

        //// For debug
        //for (int i = 0; i < num; i++) {
        //    char name[256];
        //    sprintf(name, "images[%d/%d]", i+1, num);
        //    cvShowImage(name, images[i]);
        //    cvWaitKey(0);
        //    cvDestroyWindow(name);
        //}

        // Ask save parameters or not
        if (cvAsk("Do you save the camera parameters ?\n")) {
            // Detect coners
            int *p_count = (int*)malloc(sizeof(int) * num);
            CvPoint2D32f *corners = (CvPoint2D32f*)cvAlloc(sizeof(CvPoint2D32f) * num * PAT_SIZE);
            for (int i = 0; i < num; i++) {
                // Detect chessboard
                int corner_count = 0;
                CvSize size = cvSize(PAT_COL, PAT_ROW);
                int found = cvFindChessboardCorners(images[i], size, &corners[i * PAT_SIZE], &corner_count);

                // Convert the corners to sub-pixel
                cvFindCornerSubPix(images[i], &corners[i * PAT_SIZE], corner_count, cvSize(3, 3), cvSize(-1, -1), cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03));
                p_count[i] = corner_count;
            }

            // Set the 3D position of patterns
            CvPoint3D32f *objects = (CvPoint3D32f*)cvAlloc(sizeof(CvPoint3D32f) * num * PAT_SIZE);
            for (int i = 0; i < num; i++) {
                for (int j = 0; j < PAT_ROW; j++) {
                    for (int k = 0; k < PAT_COL; k++) {
                        objects[i * PAT_SIZE + j * PAT_COL + k].x = j * CHESS_SIZE;
                        objects[i * PAT_SIZE + j * PAT_COL + k].y = k * CHESS_SIZE;
                        objects[i * PAT_SIZE + j * PAT_COL + k].z = 0.0;
                    }
                }
            }

            // Create matrices
            CvMat object_points, image_points, point_counts;
            cvInitMatHeader(&object_points, num * PAT_SIZE, 3, CV_32FC1, objects);
            cvInitMatHeader(&image_points,  num * PAT_SIZE, 1, CV_32FC2, corners);
            cvInitMatHeader(&point_counts,  num,            1, CV_32SC1, p_count);

            // Estimate intrinsic parameters and distortion coefficients
            printf("Calicurating parameters...");
            CvMat *intrinsic   = cvCreateMat(3, 3, CV_32FC1);
            CvMat *distortion  = cvCreateMat(1, 4, CV_32FC1);
            cvCalibrateCamera2(&object_points, &image_points, &point_counts, cvGetSize(images[0]), intrinsic, distortion);
            printf("Finished !\n");

            // Output a file
            printf("Generating a XML file...");
            CvFileStorage *fs = cvOpenFileStorage("camera.xml", 0, CV_STORAGE_WRITE);
            cvWrite(fs, "intrinsic", intrinsic);
            cvWrite(fs, "distortion", distortion);
            cvReleaseFileStorage(&fs);    
            printf("Finished !\n");

            // Release the matrices
            free(p_count);
            cvFree(&corners);
            cvFree(&objects);
            cvReleaseMat(&intrinsic);
            cvReleaseMat(&distortion);
        }

        // Release the images
        for (int i = 0; i < num; i++) cvReleaseImage(&images[i]);
    }

    // See you
    ardrone.close();

    return 0;
}
#else
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

    // Image of AR.Drone's camera
    IplImage *image = ardrone.getImage();

    // Read intrincis camera parameters
    CvFileStorage *fs = cvOpenFileStorage("camera.xml", 0, CV_STORAGE_READ);
    CvMat *intrinsic = (CvMat*)cvRead(fs, cvGetFileNodeByName(fs, NULL, "intrinsic"));
    CvMat *distortion = (CvMat*)cvRead(fs, cvGetFileNodeByName(fs, NULL, "distortion"));

    // Initialize undistortion maps
    CvMat *mapx = cvCreateMat(image->height, image->width, CV_32FC1);
    CvMat *mapy = cvCreateMat(image->height, image->width, CV_32FC1);
    cvInitUndistortMap(intrinsic, distortion, mapx, mapy);

    // Main loop
    while (1) {
        // Key input
        int key = cvWaitKey(1);
        if (key == 0x1b) break;

        // Update
        if (!ardrone.update()) break;

        // Get an image
        image = ardrone.getImage();

        // Remap the image
        cvRemap(image, image, mapx, mapy);

        // Display the image
        cvShowImage("camera", image);
    }

    // Release the matrices
    cvReleaseMat(&mapx);
    cvReleaseMat(&mapy);
    cvReleaseFileStorage(&fs);

    // See you
    ardrone.close();

    return 0;
}
#endif