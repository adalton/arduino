/*
 * Blink.cpp
 *
 * Blinks the onboard LED on most Arduino boards.
 *
 * Andy Dalton
 * December 1, 2013
 */
#include "Arduino.h"                                                            

enum {
    // Pin number for the LED
    LED_PIN     =   13,

    // Time (in ms) between LED state changes
    BLINK_DELAY = 1000,
};

/*
 * setup
 *
 * Perform one-time setup for this program.
 */
void setup() {
    // No setup needed
}

/*
 * loop
 *
 * Called repeatedly from the main loop.
 */
void loop() {
    // Is the LED currently on or off?
    static boolean onOff = false;

    // Set the LED to the desired state.
    digitalWrite(LED_PIN, onOff);

    // Get ready for the next invocation of this functions
    onOff = !onOff;

    // Wait a while before returning
    delay(BLINK_DELAY);
}
