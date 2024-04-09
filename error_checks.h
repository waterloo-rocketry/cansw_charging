#ifndef ERROR_CHECKS_H
#define ERROR_CHECKS_H

#include <stdbool.h>

#include "canlib/message_types.h"

// for voltages within these ranges, a warning will be sent out over CAN
// values were determined by the given voltage rail range
#define BATT_UNDERVOLTAGE_THRESHOLD_mV 9000
#define BATT_OVERVOLTAGE_THRESHOLD_mV 12600

// at this current, a warning will be sent out over CAN
#define BATT_OVERCURRENT_THRESHOLD_mA 6000 //fuse goes at 6300

#if (BOARD_UNIQUE_ID == BOARD_ID_CHARGING_CAN)
// From bus line. At this current, a warning will be sent out over CAN
#define 5V_OVERCURRENT_THRESHOLD_mA 3000 //buck rated for 3A max


#define 13V_OVERCURRENT_THRESHOLD_mA 3000 //buck rated for 3A max
#endif

// General board status checkers
bool check_battery_voltage_error(void);
bool check_battery_current_error(void);
bool check_batt_current_error(void);

#if (BOARD_UNIQUE_ID == BOARD_ID_CHARGING_CAN)
bool check_5v_current_error(void);
bool check_13v_current_error(void);
#endif

#endif /* ERROR_CHECKS_H */
