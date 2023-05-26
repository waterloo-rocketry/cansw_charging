#include <xc.h>
#include "platform.h"
#include "mcc_generated_files/adcc.h"

// LEDs and switches
#define LED_ON 0
#define CAN_5V_ON 1
#define CHG_BATT_ON 0

void pin_init(void) {
    // LEDS
    TRISC5 = 0; // set red LED pin as output
    ANSELC5 = 0; // enable digital input buffer (Useful for reading the LED state)
    LATC5 = LED_ON; // start with 5V power enabled

    TRISC6 = 0; // set blue LED pin as output
    ANSELC6 = 0; // enable digital input buffer (Useful for reading the LED state)
    LATC6 = !LED_ON; // start with charging disabled

    TRISC7 = 0; // set white LED pin as output
    ANSELC7 = 0; // enable digital input buffer (Useful for reading the LED state)
    LATC7 = !LED_ON;

    // Rocket power lines
    LATA2 = CAN_5V_ON;
    TRISA2 = 0; // allow 5V current line to be toggle-able

    TRISA1 = 1; // set 5V current draw (can 5V bus) to be input
    ANSELA1 = 1; // enable analog reading

    TRISA0 = 1; //set 13V current draw (battery) to be input
    ANSELA0 = 1; // enable analog reading

    // Battery charger
    LATA5 = !CHG_BATT_ON; // start with charging disabled
    TRISA5 = 0; // allow battery charging to be toggle-able

    TRISA4 = 1; // set battery charging current to be input
    ANSELA4 = 1; //enable analog reading

    // Voltage health
    TRISC2 = 1; //set battery voltage to be input
    ANSELC2 = 1; //enable analog reading

    TRISC3 = 1; //set rocket voltage to be input
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

void CAN_5V_SET(bool value) {
    LATA2 = !value ^ CAN_5V_ON;
}

void CHARGE_CURR_SET(bool value) {
    LATA5 = !value ^ CHG_BATT_ON;
}

// the following code was yoinked from cansw_arming

// zach derived the equation alpha = (Fs*T/5)/ 1 + (Fs*T/5)
// where Fs = sampling frequency and T = response time
// response time is equivalent to 5*tau or 5/2pi*Fc, where Fc is cutoff frequency

#define SAMPLE_FREQ (1000.0 / MAX_SENSOR_LOOP_TIME_DIFF_ms)
#define LOW_PASS_ALPHA(TR) ((SAMPLE_FREQ * TR / 5.0) / (1 + SAMPLE_FREQ * TR / 5.0))
#define LOW_PASS_RESPONSE_TIME 10 // seconds
double alpha_low = LOW_PASS_ALPHA(LOW_PASS_RESPONSE_TIME);
double low_pass_curr = 0;
void update_batt_curr_low_pass(void){
    double new_curr_reading = ADCC_GetSingleConversion(channel_POWER_V13) / CURR_13V_RESISTOR;

    low_pass_curr = alpha_low*low_pass_curr + (1.0 - alpha_low)*new_curr_reading;
}

uint16_t get_batt_curr_low_pass(void){
    return (uint16_t)low_pass_curr;
}
