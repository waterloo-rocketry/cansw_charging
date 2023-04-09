#include <xc.h>
#include "canlib.h"
#include "canlib/can.h"
#include "canlib/can_common.h"
#include "canlib/pic18f26k83/pic18f26k83_can.h"
#include "canlib/message_types.h"
#include "canlib/util/timing_util.h"
#include "canlib/util/can_tx_buffer.h"
#include "canlib/pic18f26k83/pic18f26k83_timer.h"

#include "mcc_generated_files/adcc.h"
#include "mcc_generated_files/fvr.h"

#include "device_config.h"
#include "platform.h"

static void can_msg_handler(const can_msg_t *msg);

//memory pool for the CAN tx buffer
uint8_t tx_pool[100];

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
    
    bool heartbeat = false; //TODO: I think this needs to start true
    while (1) {
        if (millis() - last_millis > MAX_LOOP_TIME_DIFF_ms) {
            // update our loop counter
            last_millis = millis();

            // visual heartbeat indicator
            BLUE_LED_SET(heartbeat);
            heartbeat = !heartbeat;
            
            // Current draws
            can_msg_t batt_curr_msg; // measures the car battery current (pre-respin)
            // implements cansw_arming's rolling average to act as a low-pass voltage filter
            build_analog_data_msg(millis(),
                                    SENSOR_BATT_CURR,
                                    get_batt_curr_low_pass(),
                                    &batt_curr_msg);
            txb_enqueue(&batt_curr_msg);

            can_msg_t bus_curr_msg; // measures current going into CAN 5V
            build_analog_data_msg(millis(),
                                    SENSOR_BUS_CURR,
                                    (uint16_t)(ADCC_GetSingleConversion(channel_POWER_V5)*CURR_5V_SCALAR),
                                    &bus_curr_msg);
            txb_enqueue(&bus_curr_msg);

            // Battery charing current
            can_msg_t chg_curr_msg; // measures current being charged into lipo
            build_analog_data_msg(millis(),
                                    SENSOR_CHARGE_CURR,
                                    (uint16_t)(ADCC_GetSingleConversion(channel_BATT_CURR)/CHG_CURR_SCALAR),
                                    &chg_curr_msg);
            txb_enqueue(&chg_curr_msg);

            // Voltage health
            can_msg_t chg_volt_msg; // measures voltage of lipo
            build_analog_data_msg(millis(),
                                    SENSOR_CHARGE_VOLT,
                                    (uint16_t)(ADCC_GetSingleConversion(channel_CAN_VOLT)*RESISTANCE_DIVIDER_SCALAR),
                                    &chg_volt_msg);
            txb_enqueue(&chg_volt_msg);

            can_msg_t batt_volt_msg; // measures the car battery voltage (pre-respin)
            build_analog_data_msg(millis(),
                                    SENSOR_BATT_VOLT,
                                    (uint16_t)(ADCC_GetSingleConversion(channel_ROCKET_VOLT)*RESISTANCE_DIVIDER_SCALAR),
                                    &batt_volt_msg);
            txb_enqueue(&batt_volt_msg);

        }
        //send any queued CAN messages
        txb_heartbeat();

        // high speed sensor checking
        if (millis() - sensor_last_millis > MAX_SENSOR_LOOP_TIME_DIFF_ms) {
            sensor_last_millis = millis();
            update_batt_curr_low_pass();
        }
    }
}

static void can_msg_handler(const can_msg_t *msg) {
    uint16_t msg_type = get_message_type(msg);

    // ignore messages that were sent from this board
    if (get_board_unique_id(msg) == BOARD_UNIQUE_ID) {
        return;
    }

    int act_state;
    switch (msg_type) {
        case MSG_ACTUATOR_CMD:
            act_state = get_req_actuator_state(msg);
            if (act_state==ACTUATOR_OPEN) {
                CAN_5V_SET(true);
            } else if (act_state==ACTUATOR_CLOSED) {
                CAN_5V_SET(false);
            }
            break;

        // all the other ones - do nothing
        default:
            break;
    }
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
