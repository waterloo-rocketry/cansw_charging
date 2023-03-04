#include "platform.h"
#include <xc.h>

// whether the leds turn on when the pin is set to high or low
#define LED_ON 0

void gpio_init(void) {
    // set as outputs
    TRISC5 = 0;
    TRISC6 = 0;
    TRISC7 = 0;
    
    // turn LEDs off
    LATC5 = !LED_ON;
    LATC6 = !LED_ON;
    LATC7 = !LED_ON;
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
