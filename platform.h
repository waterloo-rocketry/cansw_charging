#ifndef BOARD_H
#define	BOARD_H

#include <stdbool.h>

// Time between main loop code execution
#define MAX_LOOP_TIME_DIFF_ms 250
// Time between "high speed" sensor checks
#define MAX_SENSOR_LOOP_TIME_DIFF_ms 25

// Voltage monitoring
#define RESISTANCE_DIVIDER_SCALAR 3.2
// Current monioring
#define CURR_5V_SCALAR 1.0
#define CURR_13V_SCALAR 0.5
#define CHG_CURR_SCALAR 2.0

void pin_init(void);

void RED_LED_SET(bool value);
void BLUE_LED_SET(bool value);
void WHITE_LED_SET(bool value);

void CAN_5V_SET(bool value);

//returns the value from the lower cut off frequency filter
double get_batt_curr_low_low_pass(void);
#endif	/* BOARD_H */

