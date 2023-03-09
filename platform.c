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

    // Rocket power
    TRISA2 = 1; // set EN_5V to be an input
    LATA2 = 1; //not sure if its 1 or 0 to enable, but should be default on

    TRISA1 = 0; // set CURR_5V to be output
    ANSELA1 = 1; // enable analog reading

    TRISA0 = 0; //set CURR_13V to be output
    ANSELA0 = 1; // enable analog reading

    // Battery charger
    TRISA5 = 1; // set A5 as an input to toggle battery charging
    LATA5 = 1; //either 1 or 0 to enable it, not sure which, but should default be on

    TRISA4 = 0; // set A4 to be an output for battery output voltage
    ANSELA4 = 1; //enable analog reading

    // VSENSE
    TRISC2 = 1; //set BATT_VSENSE to be output
    ANSELC2 = 1; //enable analog reading

    TRISC3 = 1; //set VSW_VSENSE to be output
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
