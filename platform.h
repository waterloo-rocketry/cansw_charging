#ifndef BOARD_H
#define BOARD_H

#include <stdbool.h>
#include <stdint.h>

// Time between main loop code execution
#define MAX_LOOP_TIME_DIFF_ms 250
// Time between "high speed" sensor checks
#define MAX_SENSOR_LOOP_TIME_DIFF_ms 25
// Reset if we go this long without seeing any CAN messages (including our own)
#define MAX_BUS_DEAD_TIME_ms 1000

// Voltage monitoring
#define BATT_RESISTANCE_DIVIDER 3.2
#define GROUND_RESISTANCE_DIVIDER 3.2
// Current monioring
#define CURR_5V_RESISTOR 2.0
#define CURR_13V_RESISTOR 1.0
#define CHG_CURR_RESISTOR 2.0

void pin_init(void);

void RED_LED_SET(bool value);
void BLUE_LED_SET(bool value);
void WHITE_LED_SET(bool value);

void CAN_5V_SET(bool value);
void CHARGE_CURR_SET(bool value);

void update_batt_curr_low_pass(void);

// returns the value from the lower cut off frequency filter
uint16_t get_batt_curr_low_pass(void);

#endif /* BOARD_H */
