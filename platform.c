#include <xc.h>

#include "mcc_generated_files/adcc.h"
#include "canlib/canlib.h"
#include "platform.h"

void pin_init(void) {
    // LEDS
    TRISA2 = 0; // set red LED pin as output
    ANSELA2 = 0; // enable digital input buffer (Useful for reading the LED state)
    LATA2 = LED_ON; // start with 5V power enabled

    TRISA1 = 0; // set blue LED pin as output
    ANSELA1 = 0; // enable digital input buffer (Useful for reading the LED state)
    LATA1 = !LED_ON; // start with charging disabled

    TRISA0 = 0; // set white LED pin as output
    ANSELA0 = 0; // enable digital input buffer (Useful for reading the LED state)
    LATA0 = !LED_ON;

    // Rocket power lines
    #if (BOARD_UNIQUE_ID == BOARD_ID_CHARGING_CAN)
    LATA3 = CAN_5V_ON;
    #elif (BOARD_UNIQUE_ID == BOARD_ID_CHARGING_AIRBRAKE || BOARD_UNIQUE_ID == BOARD_ID_CHARGING_PAYLOAD)
    LATA3 = !CAN_5V_ON;
    #endif
    TRISA3 = 0; // allow 5V current line to be toggle-able
    
    #if (BOARD_UNIQUE_ID == BOARD_ID_CHARGING_CAN)
    TRISB1 = 1; // set 13V current draw (battery) to be input
    ANSELB1 = 1; // enable analog reading
    #endif

    #if (BOARD_UNIQUE_ID == BOARD_ID_CHARGING_CAN || BOARD_UNIQUE_ID == BOARD_ID_CHARGING_PAYLOAD )
    TRISB0 = 1; // set 5V current draw (payload logic + motor) to be input
    ANSELB0 = 1; // enable analog reading
    #endif

    TRISC7 = 1; // set +BATT current draw (battery) to be input
    ANSELC7 = 1; // enable analog reading

    // Battery charger
    LATA5 = !CHG_BATT_ON; // start with charging disabled
    TRISA5 = 0; // allow battery charging to be toggle-able

    TRISA4 = 1; // set battery charging current to be input
    ANSELA4 = 1; // enable analog reading

    // Voltage health
    TRISC2 = 1; // set +BATT voltage to be input
    ANSELC2 = 1; // enable analog reading

    TRISC3 = 1; // set +13V voltage to be input
    ANSELC3 = 1; // enable analog reading
#if (BOARD_UNIQUE_ID == BOARD_ID_CHARGING_AIRBRAKE || BOARD_UNIQUE_ID == BOARD_ID_CHARGING_PAYLOAD)    
    //setup motor pins
    LATB3 = !MOTOR_ON; // start with motor disabled
    TRISB3 = 0; // allow motor to be toggled
    
    TRISB5 = 0; // set motor input to be output
    
    TRISB2 = 1; // set motor current draw (battery/5V) to be input
    ANSELB2 = 1; // enable analog reading
#endif
}
void RED_LED_SET(bool value) {
    LATA2 = !value ^ LED_ON;
}

void BLUE_LED_SET(bool value) {
    LATA1 = !value ^ LED_ON;
}

void WHITE_LED_SET(bool value) {
    LATA0 = !value ^ LED_ON;
}
#if (BOARD_UNIQUE_ID == BOARD_ID_CHARGING_CAN)
void CAN_5V_SET(bool value) {
    LATA3 = !value ^ CAN_5V_ON;
}
#endif
void BATTERY_CHARGER_EN(bool value) {
    LATA5 = !value ^ CHG_BATT_ON;
}



// this code is for low pass filter stuff for current
// the following code was yoinked from cansw_arming
// zach derived the equation alpha = (Fs*T/5)/ 1 + (Fs*T/5)
// where Fs = sampling frequency and T = response time
// response time is equivalent to 5*tau or 5/2pi*Fc, where Fc is cutoff frequency

double alpha_low = LOW_PASS_ALPHA(LOW_PASS_RESPONSE_TIME);
double low_pass_curr_batt = 0;
double low_pass_curr_motor = 0;
double low_pass_curr_13v = 0;
double low_pass_curr_5v = 0;
//i think this is needed for 13V BATT Motor and 5V current readings? not sure tho
void update_batt_curr_low_pass(void) {
    double new_curr_reading = ADCC_GetSingleConversion(channel_BATT_CURR) / CURR_BATT_RESISTOR;
    low_pass_curr_batt = alpha_low * low_pass_curr_batt + (1.0 - alpha_low) * new_curr_reading;
}

uint16_t get_batt_curr_low_pass(void) {
    return (uint16_t)low_pass_curr_batt;
}

#if (BOARD_UNIQUE_ID == BOARD_ID_CHARGING_PAYLOAD || BOARD_UNIQUE_ID == BOARD_ID_CHARGING_AIRBRAKE)
void update_motor_curr_low_pass(void) {
    double new_curr_reading = ADCC_GetSingleConversion(channel_MOTOR_CURR) / CURR_MOTOR_RESISTOR;
    low_pass_curr_motor = alpha_low * low_pass_curr_motor + (1.0 - alpha_low) * new_curr_reading;
}

uint16_t get_motor_curr_low_pass(void) {
    return (uint16_t)low_pass_curr_motor;
}    
#endif

void update_13v_curr_low_pass(void) {
    double new_curr_reading = ADCC_GetSingleConversion(channel_POWER_V13) * CONVERSION_ADC_TO_V / CURR_13V_RESISTOR;
    low_pass_curr_13v = alpha_low * low_pass_curr_13v + (1.0 - alpha_low) * new_curr_reading;
}

uint16_t get_13v_curr_low_pass(void) {
    return (uint16_t)low_pass_curr_13v;
}

void update_5v_curr_low_pass(void) {
    double new_curr_reading = ADCC_GetSingleConversion(channel_POWER_V5) * CONVERSION_ADC_TO_V / CURR_5V_RESISTOR;
    low_pass_curr_5v = alpha_low * low_pass_curr_5v + (1.0 - alpha_low) * new_curr_reading;
}

uint16_t get_5v_curr_low_pass(void) {
    return (uint16_t)low_pass_curr_5v;
}