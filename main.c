#include <xc.h>

#include "canlib.h"

#include "mcc_generated_files/adcc.h"
#include "mcc_generated_files/fvr.h"

#include "device_config.h"
#include "error_checks.h"
#include "platform.h"
#include "pwm.h"

#define MOTOR_ON 1;

static void can_msg_handler(const can_msg_t *msg);
static void send_status_ok(void);
void actuate_airbrakes(float extension);
extern void timer2_handle_interrupt(void);

// memory pool for the CAN tx buffer
uint8_t tx_pool[100];

volatile bool seen_can_message = false;

// setup airbrakes variables
#if (BOARD_UNIQUE_ID == BOARD_ID_CHARGING_AIRBRAKE)
uint32_t inj_open_time = 0;

enum FLIGHT_PHASE {
    PRE_FLIGHT = 0,
    BOOST,
    COAST,
    DESCENT,
};

enum FLIGHT_PHASE state = PRE_FLIGHT;
const uint32_t BOOST_LENGTH_MS = 10000; // 10000ms = 10s - CHANGE THIS
const uint32_t COAST_LENGTH_MS = 10000;
volatile bool debug_en = false;

//Commanded extension is 0-1 as fraction of full extension
volatile float cmd_airbrakes_ext = 0;
volatile float debug_cmd_airbrakes_ext = 0;
float curr_airbrakes_ext = 0;
uint32_t airbrakes_act_time = 0;

//motor constants
const uint32_t MOTOR_ACT_TIME_MS = 500; //Motor guaranteed to fully actuate in this time
const uint16_t MOTOR_MIN_PULSE_WIDTH_US = 50; // corresponds to 0 deg
const uint16_t MOTOR_MAX_PULSE_WIDTH_US = 2500; // corresponds to 180 deg
const uint16_t MOTOR_MIN_EXT_DEG = 0.0;
const uint16_t MOTOR_MAX_EXT_DEG = 180.0;
const uint16_t AB_MIN_EXT_DEG = 20; //under this the servo will stall - 20 deg
const uint16_t AB_MAX_EXT_DEG = 140; //over this the servo will stall -  140 deg
const uint16_t PWM_PERIOD = 500; // 500*10us = 5ms period - 50us at 1% DC, 2500us at 50% DC 

#define PERCENT_TO_DEG(percent) ( (float) percent * (AB_MAX_EXT_DEG - AB_MIN_EXT_DEG) + AB_MIN_EXT_DEG)
#define DEG_TO_PULSEWIDTH(deg) ( (uint16_t) ((deg/MOTOR_MAX_EXT_DEG) * (MOTOR_MAX_PULSE_WIDTH_US - MOTOR_MIN_PULSE_WIDTH_US) + MOTOR_MIN_PULSE_WIDTH_US) )
#define PERCENT_TO_PULSEWIDTH(percent) ( DEG_TO_PULSEWIDTH(PERCENT_TO_DEG(percent)) )

//setup payload variables
#elif (BOARD_UNIQUE_ID == BOARD_ID_CHARGING_PAYLOAD)
const uint16_t MIN_PULSE_WIDTH_US = 0;
const uint16_t MAX_PULSE_WIDTH_US = 1000; 
uint16_t cmd_payload_rpm = 0; //payload is just running at one speed? make const?
#endif

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

#if (BOARD_UNIQUE_ID == BOARD_ID_CHARGING_AIRBRAKE)
    // set up PWM
    pwm_init(PWM_PERIOD);
    pwm_set_duty_cycle(0);
    pwm_enable();
#endif

    // loop timer
    uint32_t last_millis = 0;
    uint32_t sensor_last_millis = millis();
    uint32_t last_message_millis = millis();

    bool heartbeat = false;
    while (1) {
        CLRWDT(); // feed the watchdog, which is set for 256ms
        
        pwm_set_duty_cycle(10); // 1% of 5ms = 50 us

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

        uint32_t mls = millis();
        if ((mls - last_millis) > MAX_LOOP_TIME_DIFF_ms) {
            // update our loop counter
            last_millis = millis();

            // visual heartbeat indicator
            WHITE_LED_SET(heartbeat);
            heartbeat = !heartbeat;

            // check for general board status
            bool status_ok = true;
            status_ok &= check_battery_voltage_error();
            status_ok &= check_battery_current_error();
            status_ok &= check_battery_current_error();
#if (BOARD_UNIQUE_ID == BOARD_ID_CHARGING_CAN)
            status_ok &= check_5v_current_error();
            status_ok &= check_13v_current_error();
#endif
            // if there was an issue, a message would already have been sent out
            if (status_ok) {
                send_status_ok();
            }
#if (BOARD_UNIQUE_ID == BOARD_ID_CHARGING_CAN)
            /*            // Current draws
                        can_msg_t vcc_curr_msg; // measures the VCC current (mosfit decides
                                                // groundside or lipo battery)
                        // implements cansw_arming's rolling average to act as a low-pass
                        // voltage filter
                        build_analog_data_msg(
                            millis(), SENSOR_BATT_CURR, get_batt_curr_low_pass(), &vcc_curr_msg);
                        txb_enqueue(&vcc_curr_msg); //COPY THIS FOR OTHER CURRENT SENSING
            */
            can_msg_t curr_msg_5v; // measures current going into CAN 5V
            build_analog_data_msg(millis(), SENSOR_5V_CURR, get_5v_curr_low_pass(), &curr_msg_5v);
            txb_enqueue(&curr_msg_5v);

            can_msg_t curr_msg_13v; // measures 13V current
            build_analog_data_msg(
                millis(), SENSOR_13V_CURR, get_13v_curr_low_pass(), &curr_msg_13v);
            txb_enqueue(&curr_msg_13v);
#endif
            // Battery charging current
            can_msg_t curr_msg_chg; // charging current going into lipo
            build_analog_data_msg(
                millis(),
                SENSOR_CHARGE_CURR,
                (uint16_t)(ADCC_GetSingleConversion(channel_CHARGE_CURR) / CHG_CURR_RESISTOR),
                &curr_msg_chg);
            txb_enqueue(&curr_msg_chg);

            can_msg_t curr_msg_batt; // current draw from lipo
            build_analog_data_msg(
                millis(), SENSOR_BATT_CURR, get_batt_curr_low_pass(), &curr_msg_batt);
            txb_enqueue(&curr_msg_batt);

            // measure motor current            
#if (BOARD_UNIQUE_ID == BOARD_ID_CHARGING_PAYLOAD || BOARD_UNIQUE_ID == BOARD_ID_CHARGING_AIRBRAKE)
            can_msg_t curr_msg_motor;
            build_analog_data_msg(
                millis(), SENSOR_MOTOR_CURR, get_motor_curr_low_pass(), &curr_msg_motor);
            txb_enqueue(&curr_msg_motor);
#endif
            // Voltage health
            can_msg_t batt_volt_msg; // lipo battery voltage
            build_analog_data_msg(
                millis(),
                SENSOR_BATT_VOLT,
                (uint16_t)(ADCC_GetSingleConversion(channel_BATT_VOLT) * BATT_RESISTANCE_DIVIDER),
                &batt_volt_msg);
            txb_enqueue(&batt_volt_msg);

            can_msg_t ground_volt_msg; // groundside battery voltage
            build_analog_data_msg(millis(),
                SENSOR_GROUND_VOLT,
                (uint16_t)(ADCC_GetSingleConversion(channel_GROUND_VOLT) * GROUND_RESISTANCE_DIVIDER),
                &ground_volt_msg);
            txb_enqueue(&ground_volt_msg);
        }
        // send any queued CAN messages
        txb_heartbeat();

        // update high speed sensor lowpass
        if (millis() - sensor_last_millis > MAX_SENSOR_LOOP_TIME_DIFF_ms) {
            sensor_last_millis = millis();
            update_batt_curr_low_pass();
        }
#if (BOARD_UNIQUE_ID == BOARD_ID_CHARGING_AIRBRAKE)
        // change flight state
        if ((millis() - inj_open_time) > BOOST_LENGTH_MS && state == BOOST) {
            state = COAST;
    #if (IS_KETO)
            LATB0 = 0; // Keto D7
    #else
            LATB3 = MOTOR_ON;
    #endif
        }
        if ((millis() - inj_open_time) > (BOOST_LENGTH_MS + COAST_LENGTH_MS) && state == COAST) {
            state = DESCENT;
            
            //close the airbrakes
            //actuate_airbrakes(0);
            airbrakes_act_time = millis();
        }
        if ((millis() - MOTOR_ACT_TIME_MS) > airbrakes_act_time &&
            (state == PRE_FLIGHT || state == DESCENT)) {
    #if (IS_KETO)
            LATB0 = 1; // Keto D7
    #else
            LATB3 = !MOTOR_ON;
    #endif
        }
        if (cmd_airbrakes_ext != curr_airbrakes_ext) {
            //actuate_airbrakes(cmd_airbrakes_ext);
            curr_airbrakes_ext = cmd_airbrakes_ext;
        }
    #if (IS_KETO)
        if (curr_airbrakes_ext != 0) {
            LATB1 = MOTOR_ON; // Keto D6
        } else {
            LATB1 = !MOTOR_ON;
        }
    #endif
#endif
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
        case MSG_ACTUATOR_CMD: // this will toggle *all* battery chargers, not just CHARGING_CAN
            act_id = get_actuator_id(msg);
            act_state = get_req_actuator_state(msg);
            
            //Battery Charger On/Off
            if (act_id == ACTUATOR_CHARGE) {
                if (act_state == ACTUATOR_ON) {
                    BATTERY_CHARGER_EN(true);
                    BLUE_LED_SET(true);
                } else if (act_state == ACTUATOR_OFF) {
                    BATTERY_CHARGER_EN(false);
                    BLUE_LED_SET(false);
                }
            }
            
            //RocketCAN 5V Line On/Off
#if (BOARD_UNIQUE_ID == BOARD_ID_CHARGING_CAN)
            else if (act_id == ACTUATOR_CANBUS) {
                if (act_state == ACTUATOR_ON) {
                    // CAN_5V_SET(true);
                    RED_LED_SET(true);
                } else if (act_state == ACTUATOR_OFF) {
                    // CAN_5V_SET(false);
                    RED_LED_SET(false);
                }
            }
#endif
            //Catch injector valve open command to signal boost phase
#if (BOARD_UNIQUE_ID == BOARD_ID_CHARGING_AIRBRAKE)
            else if (act_id == ACTUATOR_INJECTOR_VALVE && act_state == ACTUATOR_ON) {
                // inj open -> we're launching
                inj_open_time = millis();
                state = BOOST;
            }
#endif
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
            
            //Airbrakes servo command logic
#if (BOARD_UNIQUE_ID == BOARD_ID_CHARGING_AIRBRAKE)
        case MSG_ACT_ANALOG_CMD:
            act_id = get_actuator_id(msg);
            if (act_id == ACTUATOR_AIRBRAKES_SERVO &&
                (state == COAST ||
                 (debug_en && debug_cmd_airbrakes_ext == get_req_actuator_state_analog(msg) &&
                  state == PRE_FLIGHT))) {
                cmd_airbrakes_ext = debug_cmd_airbrakes_ext;
            } 
            else if (act_id == ACTUATOR_AIRBRAKES_ENABLE) {
                debug_cmd_airbrakes_ext = get_req_actuator_state_analog(msg);
#if (IS_KETO)
                LATB0 = 0; //Keto D7
#else
                LATB3 = MOTOR_ON;
#endif
                debug_en = true;
            }
            
            //Payload servo command logic
#elif (BOARD_UNIQUE_ID == BOARD_ID_CHARGING_PAYLOAD)
        case MSG_ACT_ANALOG_CMD:
            act_id = get_actuator_id(msg);
            if (act_id == ACTUATOR_PAYLOAD_SERVO) {
                cmd_payload_rpm = get_req_actuator_state_analog(msg);
            }
#endif
        // all the other ones - do nothing
        default:
            break;
    }
}

// Send a CAN message with nominal status
static void send_status_ok(void) {
    can_msg_t board_stat_msg;
    build_board_stat_msg(millis(), E_NOMINAL, NULL, 0, &board_stat_msg);
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

    // Timer2 has overflowed
    // This happens approximately every 100us
    if (PIE4bits.TMR2IE == 1 && PIR4bits.TMR2IF == 1) {
        timer2_handle_interrupt();
        PIR4bits.TMR2IF = 0;
    }
}

//Motor control functions
#if (BOARD_UNIQUE_ID == BOARD_ID_CHARGING_AIRBRAKE)
void actuate_airbrakes(float extension) {
    pwm_set_duty_cycle( (uint16_t) PERCENT_TO_PULSEWIDTH(extension) / (PWM_PERIOD / 100) );
    if (debug_en) {
        debug_en = false;
        airbrakes_act_time = millis();
    }
}

#elif (BOARD_UNIQUE_ID == BOARD_ID_CHARGING_PAYLOAD)
float percent2Cycle(float percent) {
    return (MIN_PULSE_WIDTH_US + percent * (MAX_PULSE_WIDTH_US - MIN_PULSE_WIDTH_US));
}

void actuate_payload(float extension) {
    // percent2Cycle(extension);
    // do another thing (write a value via PWM to B5)
}
#endif
