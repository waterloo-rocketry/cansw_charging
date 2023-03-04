#include <xc.h>
#include "canlib.h"

#include "mcc_generated_files/adcc.h"
#include "mcc_generated_files/fvr.h"

#include "device_config.h"
#include "platform.h"

#define MAX_LOOP_TIME_DIFF_ms 250

static void can_msg_handler(const can_msg_t *msg);

//memory pool for the CAN tx buffer
uint8_t tx_pool[100];

int main(void) {
    // set up pins
    gpio_init();
    
    // initialize mcc functions
    ADCC_Initialize();
    FVR_Initialize();

    // intiialize the external oscillator
    oscillator_init();

    // init our millis() function
    timer0_init();

    // Enable global interrupts
    INTCON0bits.GIE = 1;

    // Set up CAN TX
    TRISC0 = 0; // set as output
    RC0PPS = 0x33; // make C0 transmit CAN TX (page 267)

    // Set up CAN RX
    TRISC1 = 1; // set as input
    ANSELC1 = 0; // not analog
    CANRXPPS = 0x11; // make CAN read from C1 (page 264-265)

    // set up CAN module
    can_timing_t can_setup;
    can_generate_timing_params(_XTAL_FREQ, &can_setup);
    can_init(&can_setup, can_msg_handler);
    // set up CAN tx buffer
    txb_init(tx_pool, sizeof(tx_pool), can_send, can_send_rdy);

    // loop timer
    uint32_t last_millis = millis();
    
    bool heartbeat = false;
    while (1) {
        if (millis() - last_millis > MAX_LOOP_TIME_DIFF_ms) {
            // update our loop counter
            last_millis = millis();

            // visual heartbeat indicator
            BLUE_LED_SET(heartbeat);
            heartbeat = !heartbeat;
            
            // We're alive, let's tell the world!
            can_msg_t board_stat_msg;
            // based off other examples (cansw_arming)
            // just go and send all the can messages here
            // making sure to txb_enqueue said message
            // build_analog_data_msg(millis(), )
            // build_board_stat_msg(millis(), E_NOMINAL, NULL, 0, &board_stat_msg);
            txb_enqueue(&board_stat_msg);
        }
        //send any queued CAN messages
        txb_heartbeat();
    }
}

static void can_msg_handler(const can_msg_t *msg) {
    uint16_t msg_type = get_message_type(msg);

    // ignore messages that were sent from this board
    if (get_board_unique_id(msg) == BOARD_UNIQUE_ID) {
        return;
    }

    switch (msg_type) {
        case MSG_ACTUATOR_CMD:
            int act_state = get_req_actuator_state(msg)
            // jack said these actuator states might change so yeah
            if (act_state==ACTUATOR_OPEN) {
                // turn on EN_5V
            } else if (act_state==ACTUATOR_CLOSED) {
                // turn off EN_5V
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
