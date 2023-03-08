#include "platform.h"
#include <xc.h>

// whether the leds turn on when the pin is set to high or low
#define LED_ON 0

/*
    // LEDs
    TRISC5 = 0; // set C5 as an output for the white LED
    ANSELC5 = 0; // Enable digital input buffer (Useful for reading the LED state)
    LATC5 = 1; // turn the white LED off
*/
void pin_init(void) {
    // LEDS
    TRISC5 = 0; // set C5 as an output for red LED
    ANSELC5 = 0; // enable digital input buffer (Useful for reading the LED state)
    LATC5 = !LED_ON;

    TRISC6 = 0; // set C6 as an output for blue LED
    ANSELC6 = 0; // enable digital input buffer (Useful for reading the LED state)
    LATC6 = !LED_ON;

    TRISC7 = 0; // set C7 as an output for white LED
    ANSELC7 = 0; // enable digital input buffer (Useful for reading the LED state)
    LATC7 = !LED_ON;

    // Battery charger
}

void RED_LED_SET(bool value) {
    LATC5 = !value ^ LED_ON;
}
void BLUE_LED_SET(bool value) {
    LATC6 = !value ^ LED_ON;
}
void WHITE_LED_SET(bool value) {
    LATC7 = !value ^ LED_ON;
}
