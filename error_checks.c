#include "canlib/canlib.h"

#include "mcc_generated_files/adcc.h"
#include "mcc_generated_files/fvr.h"

#include "error_checks.h"
#include "platform.h"

#include "timer.h"

//******************************************************************************
//                              STATUS CHECKS                                 //
//******************************************************************************

bool check_battery_voltage_error(void) {
    uint16_t adc_result = ADCC_GetSingleConversion(channel_BATT_VOLT);
    uint16_t batt_voltage_mV =
        (uint16_t)(adc_result * BATT_RESISTANCE_DIVIDER);

    if (batt_voltage_mV < UNDERVOLTAGE_THRESHOLD_BATT_mV ||
        batt_voltage_mV > OVERVOLTAGE_THRESHOLD_BATT_mV) {

        uint32_t timestamp = millis();
        uint8_t batt_data[2] = {0};
        batt_data[0] = (batt_voltage_mV >> 8) & 0xff;
        batt_data[1] = (batt_voltage_mV >> 0) & 0xff;
        enum BOARD_STATUS error_code = batt_voltage_mV < UNDERVOLTAGE_THRESHOLD_BATT_mV
                                           ? E_BATT_UNDER_VOLTAGE
                                           : E_BATT_OVER_VOLTAGE;

        can_msg_t error_msg;
        build_board_stat_msg(timestamp, error_code, batt_data, 2, &error_msg);
        txb_enqueue(&error_msg);

        // shit's bad yo
        return false;
    }
    else
    {
        can_msg_t batt_volt_msg; // lipo battery voltage
        build_analog_data_msg(
            millis(),
            SENSOR_BATT_VOLT,
            (uint16_t)(ADCC_GetSingleConversion(channel_BATT_VOLT) * BATT_RESISTANCE_DIVIDER),
            &batt_volt_msg
            );
        txb_enqueue(&batt_volt_msg);
    }
    // things look ok
    return true;
}

bool check_battery_current_error(void) {
    uint16_t curr_draw_mA = get_batt_curr_low_pass();

    if (curr_draw_mA > OVERCURRENT_THRESHOLD_BATT_mA) {
        uint32_t timestamp = millis();
        uint8_t curr_data[2] = {0};
        curr_data[0] = (curr_draw_mA >> 8) & 0xff;
        curr_data[1] = (curr_draw_mA >> 0) & 0xff;

        can_msg_t error_msg;
        build_board_stat_msg(timestamp, E_BATT_OVER_CURRENT, curr_data, 2, &error_msg);
        txb_enqueue(&error_msg);
        return false;
    }

    // things look ok
    return true;
}

bool check_5v_current_error(void) {
    uint16_t curr_draw_mA = get_5v_curr_low_pass();

    if (curr_draw_mA > OVERCURRENT_THRESHOLD_5V_mA) {
        uint32_t timestamp = millis();
        uint8_t curr_data[2] = {0};
        curr_data[0] = (curr_draw_mA >> 8) & 0xff;
        curr_data[1] = (curr_draw_mA >> 0) & 0xff;

        can_msg_t error_msg;
        build_board_stat_msg(timestamp, E_5V_OVER_CURRENT, curr_data, 2, &error_msg);
        txb_enqueue(&error_msg);
        return false;
    }

    // things look ok
    return true;
}

bool check_13v_current_error(void) {
    uint16_t curr_draw_mA = get_13v_curr_low_pass();

    if (curr_draw_mA > OVERCURRENT_THRESHOLD_13V_mA) {
        uint32_t timestamp = millis();
        uint8_t curr_data[2] = {0};
        curr_data[0] = (curr_draw_mA >> 8) & 0xff;
        curr_data[1] = (curr_draw_mA >> 0) & 0xff;

        can_msg_t error_msg;
        build_board_stat_msg(timestamp, E_13V_OVER_CURRENT, curr_data, 2, &error_msg);
        txb_enqueue(&error_msg);
        return false;
    }

    // things look ok
    return true;
}
