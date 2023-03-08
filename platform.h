#ifndef BOARD_H
#define	BOARD_H

#include <stdbool.h>

// Voltage Monitoring
#define ANALOG_SCALAR 4.0960 // the magic F constant (?)
#define SCALE 1.0/1024.0

void pin_init(void);

void RED_LED_SET(bool value);
void BLUE_LED_SET(bool value);
void WHITE_LED_SET(bool value);

#endif	/* BOARD_H */

