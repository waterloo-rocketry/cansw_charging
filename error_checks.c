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

		uint32_t gen_err_bitfield = 0;
		if(batt_voltage_mV < UNDERVOLTAGE_THRESHOLD_BATT_mV){
		    gen_err_bitfield |= (1 << E_12V_UNDER_VOLTAGE_OFFSET);
		}else{
		    gen_err_bitfield |= (1 << E_12V_OVER_VOLTAGE_OFFSET);
		}
        can_msg_t error_msg;
        build_general_board_status_msg(PRIO_MEDIUM, timestamp, gen_err_bitfield, 0, &error_msg);
        txb_enqueue(&error_msg);

        // shit's bad yo
        return false;
    }

        can_msg_t batt_volt_msg; // lipo battery voltage
        build_analog_data_msg(
			PRIO_LOW, millis(),
            SENSOR_BATT_VOLT,
            (uint16_t)(ADCC_GetSingleConversion(channel_BATT_VOLT) * BATT_RESISTANCE_DIVIDER),
            &batt_volt_msg
            );
        txb_enqueue(&batt_volt_msg);

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
        build_general_board_status_msg(PRIO_MEDIUM, timestamp, (1 << E_12V_OVER_CURRENT_OFFSET), 0, &error_msg);
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
        build_general_board_status_msg(PRIO_MEDIUM, timestamp, (1 << E_5V_OVER_CURRENT_OFFSET), 0, &error_msg);
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
        build_general_board_status_msg(PRIO_MEDIUM, timestamp, 0, 1, &error_msg);
        txb_enqueue(&error_msg);
        return false;
    }

    // things look ok
    return true;
}
