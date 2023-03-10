#ifndef BOARD_H
#define	BOARD_H

#include <stdbool.h>

#define CURR13_DRAW_FACTOR 0.5
#define BATT_CURR_FACTOR 2
#define RESISTANCE_DIVIDER_FACTOR 3.2


void pin_init(void);

void RED_LED_SET(bool value);
void BLUE_LED_SET(bool value);
void WHITE_LED_SET(bool value);

#endif	/* BOARD_H */

