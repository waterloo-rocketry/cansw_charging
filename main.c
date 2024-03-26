#include <xc.h>

#include "canlib.h"
#include "canlib/can.h"
#include "canlib/can_common.h"
#include "canlib/message_types.h"
#include "canlib/pic18f26k83/pic18f26k83_can.h"
#include "canlib/pic18f26k83/pic18f26k83_timer.h"
#include "canlib/util/can_tx_buffer.h"
#include "canlib/util/timing_util.h"

#include "mcc_generated_files/adcc.h"
#include "mcc_generated_files/fvr.h"

#include "device_config.h"
#include "error_checks.h"
#include "platform.h"

static void can_msg_handler(const can_msg_t *msg);
static void send_status_ok(void);

// memory pool for the CAN tx buffer
uint8_t tx_pool[100];

volatile bool seen_can_message = false;

int main(void) {
    // initialize mcc functions
    ADCC_Initialize();
    FVR_Initialize();

    pin_init(); // init pins
    oscillator_init(); // init the external oscillator
    timer0_init(); // init our millis() function

    // Enable global interrupts
    INTCON0bits.GIE = 1;

    // Set up CAN TX
    TRISC0 = 0;
    RC0PPS = 0x33; // make C0 transmit CAN TX (page 267)

    // Set up CAN RX
    TRISC1 = 1;
    ANSELC1 = 0;
    CANRXPPS = 0x11; // make CAN read from C1 (page 264-265)

    // set up CAN module
    can_timing_t can_setup;
    can_generate_timing_params(_XTAL_FREQ, &can_setup);
    can_init(&can_setup, can_msg_handler);
    // set up CAN tx buffer
    txb_init(tx_pool, sizeof(tx_pool), can_send, can_send_rdy);

    // loop timer
    uint32_t last_millis = millis();
    uint32_t sensor_last_millis = millis();
    uint32_t last_message_millis = millis();

    bool heartbeat = false;
    while (1) {
        CLRWDT(); // feed the watchdog, which is set for 256ms

        if (OSCCON2 != 0x70) { // If the fail-safe clock monitor has triggered
            oscillator_init();
        }

        if (seen_can_message) {
            seen_can_message = false;
            last_message_millis = millis();
        }

        if (millis() - last_message_millis > MAX_BUS_DEAD_TIME_ms) {
            // We've got too long without seeing a valid CAN message (including
            // one of ours)
            RESET();
        }

        if (millis() - last_millis > MAX_LOOP_TIME_DIFF_ms) {
            // update our loop counter
            last_millis = millis();

            // visual heartbeat indicator
            WHITE_LED_SET(heartbeat);
            heartbeat = !heartbeat;

            // check for general board status
            bool status_ok = true;
            status_ok &= check_battery_voltage_error();
            status_ok &= check_battery_current_error();
            status_ok &= check_bus_current_error();

            // if there was an issue, a message would already have been sent out
            if (status_ok) {
                send_status_ok();
            }

            // Current draws
            can_msg_t vcc_curr_msg; // measures the VCC current (mosfit decides
                                    // groundside or lipo battery)
            // implements cansw_arming's rolling average to act as a low-pass
            // voltage filter
            build_analog_data_msg(
                millis(), SENSOR_BATT_CURR, get_batt_curr_low_pass(), &vcc_curr_msg);
            txb_enqueue(&vcc_curr_msg);

            can_msg_t bus_curr_msg; // measures current going into CAN 5V
            build_analog_data_msg(
                millis(),
                SENSOR_BUS_CURR,
                (uint16_t)(ADCC_GetSingleConversion(channel_POWER_V5) / CURR_5V_RESISTOR),
                &bus_curr_msg);
            txb_enqueue(&bus_curr_msg);

            // Battery charing current
            can_msg_t chg_curr_msg; // measures charing current going into lipo
            build_analog_data_msg(
                millis(),
                SENSOR_CHARGE_CURR,
                (uint16_t)(ADCC_GetSingleConversion(channel_CHARGE_CURR) / CHG_CURR_RESISTOR),
                &chg_curr_msg);
            txb_enqueue(&chg_curr_msg);

            // Voltage health
            can_msg_t batt_volt_msg; // measures the lipo battery voltage
            build_analog_data_msg(
                millis(),
                SENSOR_BATT_VOLT,
                (uint16_t)(ADCC_GetSingleConversion(channel_BATT_VOLT) * BATT_RESISTANCE_DIVIDER),
                &batt_volt_msg);
            txb_enqueue(&batt_volt_msg);

            can_msg_t ground_volt_msg; // measures the groundside battery voltage
            build_analog_data_msg(millis(),
                                  SENSOR_GROUND_VOLT,
                                  (uint16_t)(ADCC_GetSingleConversion(channel_GROUND_VOLT) *
                                             GROUND_RESISTANCE_DIVIDER),
                                  &ground_volt_msg);
            txb_enqueue(&ground_volt_msg);
        }
        // send any queued CAN messages
        txb_heartbeat();

        // high speed sensor checking
        if (millis() - sensor_last_millis > MAX_SENSOR_LOOP_TIME_DIFF_ms) {
            sensor_last_millis = millis();
            update_batt_curr_low_pass();
        }
    }
}

static void can_msg_handler(const can_msg_t *msg) {
    seen_can_message = true;
    uint16_t msg_type = get_message_type(msg);

    // ignore messages that were sent from this board
    if (get_board_unique_id(msg) == BOARD_UNIQUE_ID) {
        return;
    }

    int act_id;
    int act_state;
    int dest_id;
    switch (msg_type) {
        case MSG_ACTUATOR_CMD:
            act_id = get_actuator_id(msg);
            act_state = get_req_actuator_state(msg);
            if (act_id == ACTUATOR_CANBUS) {
                if (act_state == ACTUATOR_ON) {
                    CAN_5V_SET(true);
                    RED_LED_SET(true);
                } else if (act_state == ACTUATOR_OFF) {
                    CAN_5V_SET(false);
                    RED_LED_SET(false);
                }
            } else if (act_id == ACTUATOR_CHARGE) {
                if (act_state == ACTUATOR_ON) {
                    CHARGE_CURR_SET(true);
                    BLUE_LED_SET(true);
                } else if (act_state == ACTUATOR_OFF) {
                    CHARGE_CURR_SET(false);
                    BLUE_LED_SET(false);
                }
            }
            break;

        case MSG_LEDS_ON:
            RED_LED_SET(true);
            BLUE_LED_SET(true);
            WHITE_LED_SET(true);
            break;

        case MSG_LEDS_OFF:
            RED_LED_SET(false);
            BLUE_LED_SET(false);
            WHITE_LED_SET(false);
            break;

        case MSG_RESET_CMD:
            dest_id = get_reset_board_id(msg);
            if (dest_id == BOARD_UNIQUE_ID || dest_id == 0) {
                RESET();
            }
            break;

        // all the other ones - do nothing
        default:
            break;
    }
}

// Send a CAN message with nominal status
static void send_status_ok(void) {
    can_msg_t board_stat_msg;
    build_board_stat_msg(millis(), E_NOMINAL, NULL, 0, &board_stat_msg);

    // send it off at low priority
    txb_enqueue(&board_stat_msg);
}

static void __interrupt() interrupt_handler(void) {
    if (PIR5) {
        can_handle_interrupt();
    }

    // Timer0 has overflowed - update millis() function
    // This happens approximately every 500us
    if (PIE3bits.TMR0IE == 1 && PIR3bits.TMR0IF == 1) {
        timer0_handle_interrupt();
        PIR3bits.TMR0IF = 0;
    }
}
