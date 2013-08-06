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

    // Battery
    printf("Battery = %d%%\n", ardrone.getBatteryPercentage());

    // Instructions
    printf("  Q - ARDRONE_ANIM_PHI_M30_DEG\n");
    printf("  A - ARDRONE_ANIM_PHI_30_DEG\n");
    printf("  Z - ARDRONE_ANIM_THETA_M30_DEG\n");
    printf("  W - ARDRONE_ANIM_THETA_30_DEG\n");
    printf("  S - ARDRONE_ANIM_THETA_20DEG_YAW_200DEG\n");
    printf("  X - ARDRONE_ANIM_THETA_20DEG_YAW_M200DEG\n");
    printf("  E - ARDRONE_ANIM_TURNAROUND\n");
    printf("  D - ARDRONE_ANIM_TURNAROUND_GODOWN\n");
    printf("  C - ARDRONE_ANIM_YAW_SHAKE\n");
    printf("  R - ARDRONE_ANIM_YAW_DANCE\n");
    printf("  F - ARDRONE_ANIM_PHI_DANCE\n");
    printf("  V - ARDRONE_ANIM_THETA_DANCE\n");
    printf("  T - ARDRONE_ANIM_VZ_DANCE\n");
    printf("  G - ARDRONE_ANIM_WAVE\n");
    printf("  B - ARDRONE_ANIM_PHI_THETA_MIXED\n");
    printf("  Y - ARDRONE_ANIM_DOUBLE_PHI_THETA_MIXED\n");
    printf("  H - ARDRONE_ANIM_FLIP_AHEAD\n");
    printf("  N - ARDRONE_ANIM_FLIP_BEHIND\n");
    printf("  U - ARDRONE_ANIM_FLIP_LEFT\n");
    printf("  J - ARDRONE_ANIM_FLIP_RIGHT\n");

    // Main loop
    while (1) {
        // Key input
        int key = cvWaitKey(33);
        if (key == 0x1b) break;

        // Update
        if (!ardrone.update()) break;

        // Get an image
        IplImage *image = ardrone.getImage();

        // Take off / Landing 
        if (key == ' ') {
            if (ardrone.onGround()) ardrone.takeoff();
            else                    ardrone.landing();
        }

        // Flight animations
        if (key == 'q') ardrone.setAnimation(ARDRONE_ANIM_PHI_M30_DEG,             1000);
        if (key == 'a') ardrone.setAnimation(ARDRONE_ANIM_PHI_30_DEG,              1000);
        if (key == 'z') ardrone.setAnimation(ARDRONE_ANIM_THETA_M30_DEG,           1000);
        if (key == 'w') ardrone.setAnimation(ARDRONE_ANIM_THETA_30_DEG,            1000);
        if (key == 's') ardrone.setAnimation(ARDRONE_ANIM_THETA_20DEG_YAW_200DEG,  1000);
        if (key == 'x') ardrone.setAnimation(ARDRONE_ANIM_THETA_20DEG_YAW_M200DEG, 1000);
        if (key == 'e') ardrone.setAnimation(ARDRONE_ANIM_TURNAROUND,              5000);
        if (key == 'd') ardrone.setAnimation(ARDRONE_ANIM_TURNAROUND_GODOWN,       5000);
        if (key == 'c') ardrone.setAnimation(ARDRONE_ANIM_YAW_SHAKE,               2000);
        if (key == 'r') ardrone.setAnimation(ARDRONE_ANIM_YAW_DANCE,               5000);
        if (key == 'f') ardrone.setAnimation(ARDRONE_ANIM_PHI_DANCE,               5000);
        if (key == 'v') ardrone.setAnimation(ARDRONE_ANIM_THETA_DANCE,             5000);
        if (key == 't') ardrone.setAnimation(ARDRONE_ANIM_VZ_DANCE,                5000);
        if (key == 'g') ardrone.setAnimation(ARDRONE_ANIM_WAVE,                    5000);
        if (key == 'b') ardrone.setAnimation(ARDRONE_ANIM_PHI_THETA_MIXED,         5000);
        if (key == 'y') ardrone.setAnimation(ARDRONE_ANIM_DOUBLE_PHI_THETA_MIXED,  5000);
        if (key == 'h') ardrone.setAnimation(ARDRONE_ANIM_FLIP_AHEAD,                15);
        if (key == 'n') ardrone.setAnimation(ARDRONE_ANIM_FLIP_BEHIND,               15);
        if (key == 'u') ardrone.setAnimation(ARDRONE_ANIM_FLIP_LEFT,                 15);
        if (key == 'j') ardrone.setAnimation(ARDRONE_ANIM_FLIP_RIGHT,                15);

        // Display the image
        cvShowImage("camera", image);
    }

    // See you
    ardrone.close();

    return 0;
}