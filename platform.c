#include "platform.h"
#include <xc.h>

// whether the leds turn on when the pin is set to high or low
#define LED_ON 0

void gpio_init(void) {
    // set as outputs
    TRISB0 = 0;
    TRISB1 = 0;
    TRISB2 = 0;
    
    // turn LEDs off
    LATB0 = !LED_ON;
    LATB1 = !LED_ON;
    LATB2 = !LED_ON;
}

void RED_LED_SET(bool value) {
    LATB0 = !value ^ LED_ON;
}
void BLUE_LED_SET(bool value) {
    LATB1 = !value ^ LED_ON;
}
void WHITE_LED_SET(bool value) {
    LATB2 = !value ^ LED_ON;
}
