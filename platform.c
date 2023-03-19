#include <xc.h>
#include "platform.h"

#define LED_ON 0
#define LINE_5V_ON 1
#define BATT_ON 0

void pin_init(void) {
    // LEDS
    TRISC5 = 0; // set red LED pin as output
    ANSELC5 = 0; // enable digital input buffer (Useful for reading the LED state)
    LATC5 = !LED_ON;

    TRISC6 = 0; // set blue LED pin as output
    ANSELC6 = 0; // enable digital input buffer (Useful for reading the LED state)
    LATC6 = !LED_ON;

    TRISC7 = 0; // set white LED pin as output
    ANSELC7 = 0; // enable digital input buffer (Useful for reading the LED state)
    LATC7 = !LED_ON;

    // Rocket power lines
    TRISA2 = 0; // allow 5V current line to be toggle-able
    LATA2 = LINE_5V_ON;

    TRISA1 = 1; // set 5V current draw to be input
    ANSELA1 = 1; // enable analog reading

    TRISA0 = 1; //set 13V current draw to be input
    ANSELA0 = 1; // enable analog reading

    // Battery charger
    TRISA5 = 0; // allow battery charging to be toggle-able
    LATA5 = BATT_ON;

    TRISA4 = 1; // set battery charging current to be input
    ANSELA4 = 1; //enable analog reading

    // Voltage health
    TRISC2 = 1; //set rocket voltage to be input
    ANSELC2 = 1; //enable analog reading

    TRISC3 = 1; //set 13V battery voltage to be input
    ANSELC3 = 1; //enable analog reading
}

void LINE_5V_SET(bool value) {
    LATA2 = !value ^ LINE_5V_ON;
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
