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

    // Instructions
    std::cout << "  Q - BLINK_GREEN_RED             " << std::endl;
    std::cout << "  A - BLINK_GREEN                 " << std::endl;
    std::cout << "  Z - BLINK_RED                   " << std::endl;
    std::cout << "  W - BLINK_ORANGE                " << std::endl;
    std::cout << "  S - SNAKE_GREEN_RED             " << std::endl;
    std::cout << "  X - FIRE                        " << std::endl;
    std::cout << "  E - STANDARD                    " << std::endl;
    std::cout << "  D - RED                         " << std::endl;
    std::cout << "  C - GREEN                       " << std::endl;
    std::cout << "  R - RED_SNAKE                   " << std::endl;
    std::cout << "  F - BLANK                       " << std::endl;
    std::cout << "  V - RIGHT_MISSILE               " << std::endl;
    std::cout << "  T - LEFT_MISSILE                " << std::endl;
    std::cout << "  G - DOUBLE_MISSILE              " << std::endl;
    std::cout << "  B - FRONT_LEFT_GREEN_OTHERS_RED " << std::endl;
    std::cout << "  Y - FRONT_RIGHT_GREEN_OTHERS_RED" << std::endl;
    std::cout << "  H - REAR_RIGHT_GREEN_OTHERS_RED " << std::endl;
    std::cout << "  N - REAR_LEFT_GREEN_OTHERS_RED  " << std::endl;
    std::cout << "  U - LEFT_GREEN_RIGHT_RED        " << std::endl;
    std::cout << "  J - LEFT_RED_RIGHT_GREEN        " << std::endl;
    std::cout << "  M - BLINK_STANDARD              " << std::endl;

    // Main loop
    while (1) {
        // Key input
        int key = cv::waitKey(33);
        if (key == 0x1b) break;

        // Get an image
        cv::Mat image = ardrone.getImage();

        // LED animations
        if (key == 'q') ardrone.setLED(ARDRONE_LED_ANIM_BLINK_GREEN_RED);
        if (key == 'a') ardrone.setLED(ARDRONE_LED_ANIM_BLINK_GREEN);
        if (key == 'z') ardrone.setLED(ARDRONE_LED_ANIM_BLINK_RED);
        if (key == 'w') ardrone.setLED(ARDRONE_LED_ANIM_BLINK_ORANGE);
        if (key == 's') ardrone.setLED(ARDRONE_LED_ANIM_SNAKE_GREEN_RED);
        if (key == 'x') ardrone.setLED(ARDRONE_LED_ANIM_FIRE);
        if (key == 'e') ardrone.setLED(ARDRONE_LED_ANIM_STANDARD);
        if (key == 'd') ardrone.setLED(ARDRONE_LED_ANIM_RED);
        if (key == 'c') ardrone.setLED(ARDRONE_LED_ANIM_GREEN);
        if (key == 'r') ardrone.setLED(ARDRONE_LED_ANIM_RED_SNAKE);
        if (key == 'f') ardrone.setLED(ARDRONE_LED_ANIM_BLANK);
        if (key == 'v') ardrone.setLED(ARDRONE_LED_ANIM_RIGHT_MISSILE);
        if (key == 't') ardrone.setLED(ARDRONE_LED_ANIM_LEFT_MISSILE);
        if (key == 'g') ardrone.setLED(ARDRONE_LED_ANIM_DOUBLE_MISSILE);
        if (key == 'b') ardrone.setLED(ARDRONE_LED_ANIM_FRONT_LEFT_GREEN_OTHERS_RED);
        if (key == 'y') ardrone.setLED(ARDRONE_LED_ANIM_FRONT_RIGHT_GREEN_OTHERS_RED);
        if (key == 'h') ardrone.setLED(ARDRONE_LED_ANIM_REAR_RIGHT_GREEN_OTHERS_RED);
        if (key == 'n') ardrone.setLED(ARDRONE_LED_ANIM_REAR_LEFT_GREEN_OTHERS_RED);
        if (key == 'u') ardrone.setLED(ARDRONE_LED_ANIM_LEFT_GREEN_RIGHT_RED);
        if (key == 'j') ardrone.setLED(ARDRONE_LED_ANIM_LEFT_RED_RIGHT_GREEN);
        if (key == 'm') ardrone.setLED(ARDRONE_LED_ANIM_BLINK_STANDARD);

        // Display the image
        cv::imshow("camera", image);
    }

    // See you
    ardrone.close();

    return 0;
}