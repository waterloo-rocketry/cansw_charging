#ifndef BOARD_H
#define	BOARD_H

#include <stdbool.h>

// Voltage Monitoring
#define RESISTANCE_FACTOR 3.2
#define SCALE 1.0/1024.0

void pin_init(void);

void RED_LED_SET(bool value);
void BLUE_LED_SET(bool value);
void WHITE_LED_SET(bool value);

#endif	/* BOARD_H */

