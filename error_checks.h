#ifndef ERROR_CHECKS_H
#define ERROR_CHECKS_H

#include <stdbool.h>

#include "canlib/message_types.h"

// for voltages within these ranges, a warning will be sent out over CAN
// values were determined by the given voltage rail range
#define UNDERVOLTAGE_THRESHOLD_BATT_mV 9000
#define OVERVOLTAGE_THRESHOLD_BATT_mV 12600

// at this current, a warning will be sent out over CAN
#define OVERCURRENT_THRESHOLD_BATT_mA 6000 //fuse goes at 6300

#if (BOARD_UNIQUE_ID == BOARD_ID_CHARGING_CAN)
// From bus line. At this current, a warning will be sent out over CAN
#define OVERCURRENT_THRESHOLD_5V_mA 1900 //buck rated for 2A max
#define OVERCURRENT_THRESHOLD_13V_mA 2900 //buck rated for 3A max

#endif

#if (BOARD_UNIQUE_ID == BOARD_ID_CHARGING_AIRBRAKE)
#define OVERCURRENT_THRESHOLD_MOTOR_mA 4500
#endif
#if (BOARD_UNIQUE_ID == BOARD_ID_CHARGING_PAYLOAD)
#define OVERCURRENT_THRESHOLD_MOTOR_mA 680
#endif

// General board status checkers
bool check_battery_voltage_error(void);
bool check_battery_current_error(void);


#if (BOARD_UNIQUE_ID == BOARD_ID_CHARGING_CAN)
bool check_5v_current_error(void);
bool check_13v_current_error(void);
#endif
#if (BOARD_UNIQUE_ID == BOARD_ID_CHARGING_AIRBRAKE || BOARD_UNIQUE_ID == BOARD_ID_CHARGING_PAYLOAD)
bool check_motor_current_error(void);
#endif
#endif /* ERROR_CHECKS_H */
