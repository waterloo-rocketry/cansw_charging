#include "canlib/can.h"
#include "canlib/can_common.h"
#include "canlib/message_types.h"
#include "canlib/pic18f26k83/pic18f26k83_can.h"
#include "canlib/pic18f26k83/pic18f26k83_timer.h"
#include "canlib/util/can_tx_buffer.h"

#include "mcc_generated_files/adcc.h"
#include "mcc_generated_files/fvr.h"

#include "error_checks.h"
#include "platform.h"

//******************************************************************************
//                              STATUS CHECKS                                 //
//******************************************************************************

bool check_battery_voltage_error(void) {
    uint16_t batt_voltage_mV =
        (uint16_t)(ADCC_GetSingleConversion(channel_BATT_VOLT) * BATT_RESISTANCE_DIVIDER);

    if (batt_voltage_mV < BATT_UNDERVOLTAGE_THRESHOLD_mV ||
        batt_voltage_mV > BATT_OVERVOLTAGE_THRESHOLD_mV) {

        uint32_t timestamp = millis();
        uint8_t batt_data[2] = {0};
        batt_data[0] = (batt_voltage_mV >> 8) & 0xff;
        batt_data[1] = (batt_voltage_mV >> 0) & 0xff;
        enum BOARD_STATUS error_code = batt_voltage_mV < BATT_UNDERVOLTAGE_THRESHOLD_mV
                                           ? E_BATT_UNDER_VOLTAGE
                                           : E_BATT_OVER_VOLTAGE;

        can_msg_t error_msg;
        build_board_stat_msg(timestamp, error_code, batt_data, 2, &error_msg);
        txb_enqueue(&error_msg);

        // shit's bad yo
        return false;
    }
    // things look ok
    return true;
}

bool check_battery_current_error(void) {
    uint16_t curr_draw_mA = ADCC_GetSingleConversion(channel_POWER_V13) / CURR_13V_RESISTOR;

    if (curr_draw_mA > BATT_OVERCURRENT_THRESHOLD_mA) {
        uint32_t timestamp = millis();
        uint8_t curr_data[2] = {0};
        curr_data[0] = (curr_draw_mA >> 8) & 0xff;
        curr_data[1] = (curr_draw_mA >> 0) & 0xff;

        can_msg_t error_msg;
        build_board_stat_msg(timestamp, E_BUS_OVER_CURRENT, curr_data, 2, &error_msg);
        txb_enqueue(&error_msg);
        return false;
    }

    // things look ok
    return true;
}

bool check_bus_current_error(void) {
    uint16_t curr_draw_mA = ADCC_GetSingleConversion(channel_POWER_V5) / CURR_5V_RESISTOR;

    if (curr_draw_mA > BUS_OVERCURRENT_THRESHOLD_mA) {
        uint32_t timestamp = millis();
        uint8_t curr_data[2] = {0};
        curr_data[0] = (curr_draw_mA >> 8) & 0xff;
        curr_data[1] = (curr_draw_mA >> 0) & 0xff;

        can_msg_t error_msg;
        build_board_stat_msg(timestamp, E_BUS_OVER_CURRENT, curr_data, 2, &error_msg);
        txb_enqueue(&error_msg);
        return false;
    }

    // things look ok
    return true;
}
