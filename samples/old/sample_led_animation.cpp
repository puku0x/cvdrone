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

    // Instructions
    printf("  Q - BLINK_GREEN_RED\n");
    printf("  A - BLINK_GREEN\n");
    printf("  Z - BLINK_RED\n");
    printf("  W - BLINK_ORANGE\n");
    printf("  S - SNAKE_GREEN_RED\n");
    printf("  X - FIRE\n");
    printf("  E - STANDARD\n");
    printf("  D - RED\n");
    printf("  C - GREEN\n");
    printf("  R - RED_SNAKE\n");
    printf("  F - BLANK\n");
    printf("  V - RIGHT_MISSILE\n");
    printf("  T - LEFT_MISSILE\n");
    printf("  G - DOUBLE_MISSILE\n");
    printf("  B - FRONT_LEFT_GREEN_OTHERS_RED\n");
    printf("  Y - FRONT_RIGHT_GREEN_OTHERS_RED\n");
    printf("  H - REAR_RIGHT_GREEN_OTHERS_RED\n");
    printf("  N - REAR_LEFT_GREEN_OTHERS_RED\n");
    printf("  U - LEFT_GREEN_RIGHT_RED\n");
    printf("  J - LEFT_RED_RIGHT_GREEN\n");
    printf("  M - BLINK_STANDARD\n");

    // Main loop
    while (1) {
        // Key input
        int key = cvWaitKey(100);
        if (key == 0x1b) break;

        // Update
        if (!ardrone.update()) break;

        // Get an image
        IplImage *image = ardrone.getImage();

        // LED animations
        if (key == 'q') ardrone.setLED(ARDRONE_LED_ANIM_BLINK_GREEN_RED,              0.5, 5);
        if (key == 'a') ardrone.setLED(ARDRONE_LED_ANIM_BLINK_GREEN,                  0.5, 5);
        if (key == 'z') ardrone.setLED(ARDRONE_LED_ANIM_BLINK_RED,                    0.5, 5);
        if (key == 'w') ardrone.setLED(ARDRONE_LED_ANIM_BLINK_ORANGE,                 0.5, 5);
        if (key == 's') ardrone.setLED(ARDRONE_LED_ANIM_SNAKE_GREEN_RED,              0.5, 5);
        if (key == 'x') ardrone.setLED(ARDRONE_LED_ANIM_FIRE,                         0.5, 5);
        if (key == 'e') ardrone.setLED(ARDRONE_LED_ANIM_STANDARD,                     0.5, 5);
        if (key == 'd') ardrone.setLED(ARDRONE_LED_ANIM_RED,                          0.5, 5);
        if (key == 'c') ardrone.setLED(ARDRONE_LED_ANIM_GREEN,                        0.5, 5);
        if (key == 'r') ardrone.setLED(ARDRONE_LED_ANIM_RED_SNAKE,                    0.5, 5);
        if (key == 'f') ardrone.setLED(ARDRONE_LED_ANIM_BLANK,                        0.5, 5);
        if (key == 'v') ardrone.setLED(ARDRONE_LED_ANIM_RIGHT_MISSILE,                0.5, 5);
        if (key == 't') ardrone.setLED(ARDRONE_LED_ANIM_LEFT_MISSILE,                 0.5, 5);
        if (key == 'g') ardrone.setLED(ARDRONE_LED_ANIM_DOUBLE_MISSILE,               0.5, 5);
        if (key == 'b') ardrone.setLED(ARDRONE_LED_ANIM_FRONT_LEFT_GREEN_OTHERS_RED,  0.5, 5);
        if (key == 'y') ardrone.setLED(ARDRONE_LED_ANIM_FRONT_RIGHT_GREEN_OTHERS_RED, 0.5, 5);
        if (key == 'h') ardrone.setLED(ARDRONE_LED_ANIM_REAR_RIGHT_GREEN_OTHERS_RED,  0.5, 5);
        if (key == 'n') ardrone.setLED(ARDRONE_LED_ANIM_REAR_LEFT_GREEN_OTHERS_RED,   0.5, 5);
        if (key == 'u') ardrone.setLED(ARDRONE_LED_ANIM_LEFT_GREEN_RIGHT_RED,         0.5, 5);
        if (key == 'j') ardrone.setLED(ARDRONE_LED_ANIM_LEFT_RED_RIGHT_GREEN,         0.5, 5);
        if (key == 'm') ardrone.setLED(ARDRONE_LED_ANIM_BLINK_STANDARD,               0.5, 5);

        // Display the image
        cvShowImage("camera", image);
    }

    // See you
    ardrone.close();

    return 0;
}