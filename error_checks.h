#ifndef ERROR_CHECKS_H
#define ERROR_CHECKS_H

#include <stdbool.h>

#include "canlib/message_types.h"

// for voltages within these ranges, a warning will be sent out over CAN
// values were determined by the given voltage rail range
#define UNDERVOLTAGE_THRESHOLD_BATT_mV 9000
#define OVERVOLTAGE_THRESHOLD_BATT_mV 12600

// at this current, a warning will be sent out over CAN
#define OVERCURRENT_THRESHOLD_BATT_mA 5700 //fuse goes at 6300

// From bus line. At this current, a warning will be sent out over CAN
#define OVERCURRENT_THRESHOLD_5V_mA 1800 //buck rated for 2A max
#define OVERCURRENT_THRESHOLD_13V_mA 2700 //buck rated for 3A max

// General board status checkers
bool check_battery_voltage_error(void);
bool check_battery_current_error(void);
bool check_5v_current_error(void);
bool check_13v_current_error(void);

#endif /* ERROR_CHECKS_H */
