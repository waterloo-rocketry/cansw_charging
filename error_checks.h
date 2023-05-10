#ifndef ERROR_CHECKS_H
#define	ERROR_CHECKS_H

#include "canlib/message_types.h"

#include <stdbool.h>

// for voltages within these ranges, a warning will be sent out over CAN
// values were determined by the given voltage rail range
#define BATT_UNDERVOLTAGE_THRESHOLD_mV 9000
#define BATT_OVERVOLTAGE_THRESHOLD_mV 12600

// at this current, a warning will be sent out over CAN
#define BATT_OVERCURRENT_THRESHOLD_mA 600 //TODO: CHECK VALUE WITH JACK

// From bus line. At this current, a warning will be sent out over CAN
#define BUS_OVERCURRENT_THRESHOLD_mA 300

// General board status checkers
bool check_battery_voltage_error(void);
bool check_battery_current_error(void);
bool check_bus_current_error(void);

#endif	/* ERROR_CHECKS_H */

