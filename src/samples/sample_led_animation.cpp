#include "ardrone/ardrone.h"

#define KEY_DOWN(key) (GetAsyncKeyState(key) & 0x8000)
#define KEY_PUSH(key) (GetAsyncKeyState(key) & 0x0001)

// --------------------------------------------------------------------------
// main(Number of arguments, Value of arguments)
// Description  : This is the main function.
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
    printf("Usages.\n");
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
    while (!GetAsyncKeyState(VK_ESCAPE)) {
        // Update
        if (!ardrone.update()) break;

        // LED animations
        if (KEY_PUSH('Q')) ardrone.setLED(BLINK_GREEN_RED,              0.5, 5);
        if (KEY_PUSH('A')) ardrone.setLED(BLINK_GREEN,                  0.5, 5);
        if (KEY_PUSH('Z')) ardrone.setLED(BLINK_RED,                    0.5, 5);
        if (KEY_PUSH('W')) ardrone.setLED(BLINK_ORANGE,                 0.5, 5);
        if (KEY_PUSH('S')) ardrone.setLED(SNAKE_GREEN_RED,              0.5, 5);
        if (KEY_PUSH('X')) ardrone.setLED(FIRE,                         0.5, 5);
        if (KEY_PUSH('E')) ardrone.setLED(STANDARD,                     0.5, 5);
        if (KEY_PUSH('D')) ardrone.setLED(RED,                          0.5, 5);
        if (KEY_PUSH('C')) ardrone.setLED(GREEN,                        0.5, 5);
        if (KEY_PUSH('R')) ardrone.setLED(RED_SNAKE,                    0.5, 5);
        if (KEY_PUSH('F')) ardrone.setLED(BLANK,                        0.5, 5);
        if (KEY_PUSH('V')) ardrone.setLED(RIGHT_MISSILE,                0.5, 5);
        if (KEY_PUSH('T')) ardrone.setLED(LEFT_MISSILE,                 0.5, 5);
        if (KEY_PUSH('G')) ardrone.setLED(DOUBLE_MISSILE,               0.5, 5);
        if (KEY_PUSH('B')) ardrone.setLED(FRONT_LEFT_GREEN_OTHERS_RED,  0.5, 5);
        if (KEY_PUSH('Y')) ardrone.setLED(FRONT_RIGHT_GREEN_OTHERS_RED, 0.5, 5);
        if (KEY_PUSH('H')) ardrone.setLED(REAR_RIGHT_GREEN_OTHERS_RED,  0.5, 5);
        if (KEY_PUSH('N')) ardrone.setLED(REAR_LEFT_GREEN_OTHERS_RED,   0.5, 5);
        if (KEY_PUSH('U')) ardrone.setLED(LEFT_GREEN_RIGHT_RED,         0.5, 5);
        if (KEY_PUSH('J')) ardrone.setLED(LEFT_RED_RIGHT_GREEN,         0.5, 5);
        if (KEY_PUSH('M')) ardrone.setLED(BLINK_STANDARD,               0.5, 5);

        Sleep(100);
    }

    // See you
    ardrone.close();

    return 0;
}