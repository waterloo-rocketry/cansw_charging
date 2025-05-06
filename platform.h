#ifndef BOARD_H
#define BOARD_H

#include <stdbool.h>
#include <stdint.h>

#define LED_ON 0
#define CAN_5V_ON 1
#define CHG_BATT_ON 0
#define MOTOR_ON 1
#define SAMPLE_FREQ (1000.0 / MAX_SENSOR_LOOP_TIME_DIFF_ms)
#define LOW_PASS_ALPHA(TR) ((SAMPLE_FREQ * TR / 5.0) / (1 + SAMPLE_FREQ * TR / 5.0))
#define LOW_PASS_RESPONSE_TIME 10 // seconds
#define MOTOR_POWER LATB3
#define MOTOR_PWM LATB5 

// Time between main loop code execution
#define MAX_LOOP_TIME_DIFF_ms 500
// Time between "high speed" sensor checks
#define MAX_SENSOR_LOOP_TIME_DIFF_ms 25
// Reset if we go this long without seeing any CAN messages (including our own)
#define MAX_BUS_DEAD_TIME_ms 10000

// Voltage monitoring in 100k Ohms
#define BATT_RESISTANCE_DIVIDER 5.6
#define GROUND_RESISTANCE_DIVIDER 5.6
// Current monitoring in units of 10 milliohms
#define CURR_5V_RESISTOR 2.0
#define CURR_13V_RESISTOR 1.0   
#define CHG_CURR_RESISTOR 2.0
#define CURR_BATT_RESISTOR 1.0
#define CURR_MOTOR_RESISTOR 1.0

#define VREF 4.096
#define MAX_COUNTS 4096
#define CONVERSION_ADC_TO_V VREF * 1000 / MAX_COUNTS

void pin_init(void);

void RED_LED_SET(bool value);
void BLUE_LED_SET(bool value);
void WHITE_LED_SET(bool value);
void CAN_5V_SET(bool value);
void BATTERY_CHARGER_EN(bool value);

void update_batt_curr_low_pass(void);
// returns the value from the lower cut off frequency filter
uint16_t get_batt_curr_low_pass(void);

void update_13v_curr_low_pass(void);
uint16_t get_13v_curr_low_pass(void);
void update_5v_curr_low_pass(void);
uint16_t get_5v_curr_low_pass(void);

#endif /* BOARD_H */
