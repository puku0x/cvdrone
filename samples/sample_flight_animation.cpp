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

    // Battery
    std::cout << "Battery = " << ardrone.getBatteryPercentage() << "%" << std::endl;

    // Instructions
    std::cout << "  Q - ARDRONE_ANIM_PHI_M30_DEG            " << std::endl;
    std::cout << "  A - ARDRONE_ANIM_PHI_30_DEG             " << std::endl;
    std::cout << "  Z - ARDRONE_ANIM_THETA_M30_DEG          " << std::endl;
    std::cout << "  W - ARDRONE_ANIM_THETA_30_DEG           " << std::endl;
    std::cout << "  S - ARDRONE_ANIM_THETA_20DEG_YAW_200DEG " << std::endl;
    std::cout << "  X - ARDRONE_ANIM_THETA_20DEG_YAW_M200DEG" << std::endl;
    std::cout << "  E - ARDRONE_ANIM_TURNAROUND             " << std::endl;
    std::cout << "  D - ARDRONE_ANIM_TURNAROUND_GODOWN      " << std::endl;
    std::cout << "  C - ARDRONE_ANIM_YAW_SHAKE              " << std::endl;
    std::cout << "  R - ARDRONE_ANIM_YAW_DANCE              " << std::endl;
    std::cout << "  F - ARDRONE_ANIM_PHI_DANCE              " << std::endl;
    std::cout << "  V - ARDRONE_ANIM_THETA_DANCE            " << std::endl;
    std::cout << "  T - ARDRONE_ANIM_VZ_DANCE               " << std::endl;
    std::cout << "  G - ARDRONE_ANIM_WAVE                   " << std::endl;
    std::cout << "  B - ARDRONE_ANIM_PHI_THETA_MIXED        " << std::endl;
    std::cout << "  Y - ARDRONE_ANIM_DOUBLE_PHI_THETA_MIXED " << std::endl;
    std::cout << "  H - ARDRONE_ANIM_FLIP_AHEAD             " << std::endl;
    std::cout << "  N - ARDRONE_ANIM_FLIP_BEHIND            " << std::endl;
    std::cout << "  U - ARDRONE_ANIM_FLIP_LEFT              " << std::endl;
    std::cout << "  J - ARDRONE_ANIM_FLIP_RIGHT             " << std::endl;

    // Main loop
    while (1) {
        // Key input
        int key = cv::waitKey(33);
        if (key == 0x1b) break;

        // Get an image
        cv::Mat image = ardrone.getImage();

        // Take off / Landing 
        if (key == ' ') {
            if (ardrone.onGround()) ardrone.takeoff();
            else                    ardrone.landing();
        }

        // Flight animations
        if (key == 'q') ardrone.setAnimation(ARDRONE_ANIM_PHI_M30_DEG);
        if (key == 'a') ardrone.setAnimation(ARDRONE_ANIM_PHI_30_DEG);
        if (key == 'z') ardrone.setAnimation(ARDRONE_ANIM_THETA_M30_DEG);
        if (key == 'w') ardrone.setAnimation(ARDRONE_ANIM_THETA_30_DEG);
        if (key == 's') ardrone.setAnimation(ARDRONE_ANIM_THETA_20DEG_YAW_200DEG);
        if (key == 'x') ardrone.setAnimation(ARDRONE_ANIM_THETA_20DEG_YAW_M200DEG);
        if (key == 'e') ardrone.setAnimation(ARDRONE_ANIM_TURNAROUND);
        if (key == 'd') ardrone.setAnimation(ARDRONE_ANIM_TURNAROUND_GODOWN);
        if (key == 'c') ardrone.setAnimation(ARDRONE_ANIM_YAW_SHAKE);
        if (key == 'r') ardrone.setAnimation(ARDRONE_ANIM_YAW_DANCE);
        if (key == 'f') ardrone.setAnimation(ARDRONE_ANIM_PHI_DANCE);
        if (key == 'v') ardrone.setAnimation(ARDRONE_ANIM_THETA_DANCE);
        if (key == 't') ardrone.setAnimation(ARDRONE_ANIM_VZ_DANCE);
        if (key == 'g') ardrone.setAnimation(ARDRONE_ANIM_WAVE);
        if (key == 'b') ardrone.setAnimation(ARDRONE_ANIM_PHI_THETA_MIXED);
        if (key == 'y') ardrone.setAnimation(ARDRONE_ANIM_DOUBLE_PHI_THETA_MIXED);
        if (key == 'h') ardrone.setAnimation(ARDRONE_ANIM_FLIP_AHEAD);
        if (key == 'n') ardrone.setAnimation(ARDRONE_ANIM_FLIP_BEHIND);
        if (key == 'u') ardrone.setAnimation(ARDRONE_ANIM_FLIP_LEFT);
        if (key == 'j') ardrone.setAnimation(ARDRONE_ANIM_FLIP_RIGHT);

        // Display the image
        cv::imshow("camera", image);
    }

    // See you
    ardrone.close();

    return 0;
}