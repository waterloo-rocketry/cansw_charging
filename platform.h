#ifndef BOARD_H
#define	BOARD_H

#include <stdbool.h>

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
#endif	/* BOARD_H */

