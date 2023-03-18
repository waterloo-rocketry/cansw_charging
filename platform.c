#include "platform.h"
#include <xc.h>

// whether the leds turn on when the pin is set to high or low
#define LED_ON 0
#define LINE_5V_ON 1 //not sure if these are correct values => powers rest of rocket, should be default on?
#define BATT_ON 1 //not sure if these are correct values

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

    // Rocket power
    TRISA2 = 0; // allow 5V current line to be toggleable (output)
    LATA2 = LINE_5V_ON;

    TRISA1 = 1; // set 5V current draw to be input
    ANSELA1 = 1; // enable analog reading

    TRISA0 = 1; //set 13V current draw to be input
    ANSELA0 = 1; // enable analog reading

    // Battery charger
    TRISA5 = 0; // allow battery charging to be toggleable (output)
    LATA5 = BATT_ON;

    TRISA4 = 1; // set battery charging current to be input
    ANSELA4 = 1; //enable analog reading

    // VSENSE
    TRISC2 = 1; //set BATT_VSENSE to be input
    ANSELC2 = 1; //enable analog reading

    TRISC3 = 1; //set VSW_VSENSE to be input
    ANSELC3 = 1; //enable analog reading
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
void LINE_5V_SET(bool value) {
    LATA2 = !value ^ LINE_5V_ON;
}