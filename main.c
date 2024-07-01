#include <xc.h>
#include "canlib.h"
#include "mcc_generated_files/adcc.h"
#include "mcc_generated_files/fvr.h"
#include "device_config.h"
#include "error_checks.h"
#include "platform.h"
#include "stdint.h"
#if (IS_KETO)
    #define MOTOR_POWER LATB1 //Keto D7
    #define MOTOR_PWM LATB0   //Keto D6
    #define MOTOR_ON 0
#else
    #define MOTOR_POWER LATB3
    #define MOTOR_PWM LATB5
    #define MOTOR_ON 1             
#endif

static void can_msg_handler(const can_msg_t *msg);
static void send_status_ok(void);
void actuate_airbrakes(float extension);
//extern void timer2_handle_interrupt(void);
void pwm_init(void);
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
//const uint32_t BOOST_LENGTH_MS = 15000; // 15000ms = 15s - CHANGE THIS
//const uint32_t COAST_LENGTH_MS = 20000;
const uint32_t BOOST_LENGTH_MS = 1000; // for the purposes of debugging
const uint32_t COAST_LENGTH_MS = 2000000;
volatile bool debug_en = false;

//Commanded extension is 0-1 as fraction of full extension
volatile float cmd_airbrakes_ext = 0;
volatile float debug_cmd_airbrakes_ext = 0;
float curr_airbrakes_ext = 0;
uint32_t airbrakes_act_time = 0;

//motor constants
const uint32_t MOTOR_ACT_TIME_MS = 5000; //Motor guaranteed to fully actuate in this time
const uint16_t MOTOR_MIN_PULSE_WIDTH_US = 500; // corresponds to 0 deg
const uint16_t MOTOR_MAX_PULSE_WIDTH_US = 2500; // corresponds to 180 deg
const float MOTOR_MIN_EXT_DEG = 0.0;
const float MOTOR_MAX_EXT_DEG = 180.0;
const uint16_t AB_MIN_EXT_DEG = 0; //under this the servo will stall - 20 deg
const uint16_t AB_MAX_EXT_DEG = 180; //over this the servo will stall -  140 deg
//const uint16_t PWM_PERIOD = 500; // 1000*10us = 10ms period - 50us at 0.5% DC, 2500us at 25% DC
void updatePulseWidth(float percent);
#define PERCENT_TO_DEG(percent) ( (float) percent * (AB_MAX_EXT_DEG - AB_MIN_EXT_DEG) + AB_MIN_EXT_DEG)
#define DEG_TO_PULSEWIDTH(deg) ( (uint16_t) ((deg/MOTOR_MAX_EXT_DEG) * (MOTOR_MAX_PULSE_WIDTH_US - MOTOR_MIN_PULSE_WIDTH_US) + MOTOR_MIN_PULSE_WIDTH_US) )
#define PERCENT_TO_PULSEWIDTH(percent) ( DEG_TO_PULSEWIDTH(PERCENT_TO_DEG(percent)) )

//setup payload variables
#elif (BOARD_UNIQUE_ID == BOARD_ID_CHARGING_PAYLOAD)
const uint16_t MOTOR_MIN_PULSE_WIDTH_US = 0;
const uint16_t MOTOR_MAX_PULSE_WIDTH_US = 1000; 
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

    #if (BOARD_UNIQUE_ID != BOARD_ID_CHARGING_CAN)
    pwm_init();
    //for 1.5 ms (90 deg))
    //CCPR3H = 0b00000010;
    //CCPR3L = 0b00110011;
    //for 2.4 ms (171 deg)
    CCPR3H = 0b00000010;
    CCPR3L = 0b01110011;
    MOTOR_POWER = MOTOR_ON;
    #endif

    // loop timer
    uint32_t last_millis = 0;
    uint32_t sensor_last_millis = millis();
    uint32_t last_message_millis = millis();
    
    //BATTERY_CHARGER_EN(true);

    bool heartbeat = false;
    while (1) {
        CLRWDT(); // feed the watchdog, which is set for 256ms
        //RED_LED_SET(state == BOOST); //Keto stuff
        //BLUE_LED_SET(state == COAST);
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
            
            //power on/off indicator
            BLUE_LED_SET(MOTOR_POWER == MOTOR_ON);

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
            /* 
            // Current draws
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

            bool result;
            // Battery charging current
            can_msg_t curr_msg_chg; // charging current going into lipo
            build_analog_data_msg(
                millis(),
                SENSOR_CHARGE_CURR,
                (uint16_t)(ADCC_GetSingleConversion(channel_CHARGE_CURR) / CHG_CURR_RESISTOR), 
                &curr_msg_chg
            );
            result = txb_enqueue(&curr_msg_chg);

            can_msg_t curr_msg_batt; // current draw from lipo
            build_analog_data_msg(
                millis(), SENSOR_BATT_CURR, get_batt_curr_low_pass(), &curr_msg_batt);
            result = txb_enqueue(&curr_msg_batt);

            // measure motor current            
            #if (BOARD_UNIQUE_ID == BOARD_ID_CHARGING_PAYLOAD || BOARD_UNIQUE_ID == BOARD_ID_CHARGING_AIRBRAKE)
            can_msg_t curr_msg_motor;
            build_analog_data_msg(millis(), SENSOR_MOTOR_CURR, get_motor_curr_low_pass(), &curr_msg_motor);
            result = txb_enqueue(&curr_msg_motor);
            #endif
        
            // Voltage health
            
            //battery voltage msg is constructed in check_battery_voltage_error if no error

            can_msg_t ground_volt_msg; // groundside battery voltage
            build_analog_data_msg(
                millis(),
                SENSOR_GROUND_VOLT,
                (uint16_t)(ADCC_GetSingleConversion(channel_GROUND_VOLT) * GROUND_RESISTANCE_DIVIDER),
                &ground_volt_msg
            );
            result = txb_enqueue(&ground_volt_msg);
        }//ended here
        
        // send any queued CAN messages
        txb_heartbeat();

        // update high speed sensor lowpass
        if (millis() - sensor_last_millis > MAX_SENSOR_LOOP_TIME_DIFF_ms) {
            sensor_last_millis = millis();
            update_batt_curr_low_pass();
#if (BOARD_UNIQUE_ID == BOARD_ID_CHARGING_AIRBRAKE || BOARD_UNIQUE_ID == BOARD_ID_CHARGING_PAYLOAD)
            update_motor_curr_low_pass();
#elif (BOARD_UNIQUE_ID == BOARD_ID_CHARGING_CAN)
            update_5v_curr_low_pass();
            update_13v_curr_low_pass();
#endif            
        }

        #if (BOARD_UNIQUE_ID == BOARD_ID_CHARGING_AIRBRAKE)
            // state transition from boost to coast, enable motor
            if (state == BOOST && ((millis() - inj_open_time) > BOOST_LENGTH_MS)) {
                state = COAST;
                MOTOR_POWER = MOTOR_ON;
            }
        
        
            //state transition from coast to descent, enable motor for MOTOR_ACT_TIME_MS
            if (state == COAST && ((millis() - inj_open_time) > (BOOST_LENGTH_MS + COAST_LENGTH_MS))) {
                state = DESCENT;
            
                MOTOR_POWER = MOTOR_ON;
                cmd_airbrakes_ext = 0;
                airbrakes_act_time = millis();
            }
        
            //If we are on the ground or in descent, cut motor power after a certain period of time
            if ((state == PRE_FLIGHT || state == DESCENT) 
                && ((millis() - airbrakes_act_time) > MOTOR_ACT_TIME_MS)) {
            
                MOTOR_POWER = !MOTOR_ON;
                cmd_airbrakes_ext = 0;
            }
            
            //test code: run pwm command
            //pwm_init();
            if (cmd_airbrakes_ext != curr_airbrakes_ext) {
                actuate_airbrakes(cmd_airbrakes_ext);
                curr_airbrakes_ext = cmd_airbrakes_ext;
                updatePulseWidth(cmd_airbrakes_ext);
            }
        #endif
    }
}

static void can_msg_handler(const can_msg_t *msg) {
    seen_can_message = true;
    uint16_t msg_type = get_message_type(msg);
    
    //echo -n "m0C0,00,00,00,01,00;" > /dev/tty.usbmodem12401                   //injector valve open message
    //echo -n "m220,00,00,00,09,00,00,80,3F;" > /dev/tty.usbmodem21401          //actuate to 1
    //echo -n "m220,00,00,00,09,66,66,66,3F;" > /dev/tty.usbmodem21401          //actuate to 0.9
    //echo -n "m220,00,00,00,09,00,00,00,3F;" > /dev/tty.usbmodem21401          //actuate to 0.5
    //echo -n "m220,00,00,00,09,DC,CC,CC,3E;" > /dev/tty.usbmodem21401          //actuate to 0.4
    //echo -n "m220,00,00,00,09,00,00,00,00;" > /dev/tty.usbmodem21401          //actuate to 0.0

    // ignore messages that were sent from this board
    if (get_board_unique_id(msg) == BOARD_UNIQUE_ID || get_board_unique_id(msg)==BOARD_ID_LOGGER) {
        return;
    }
    //RED_LED_SET(true);

    int act_id;
    int act_state;
    int dest_id;
    int percent_act;
    switch (msg_type) {
        case MSG_ACTUATOR_CMD: // this will toggle *all* battery chargers, not just CHARGING_CAN
            act_id = get_actuator_id(msg);
            act_state = get_req_actuator_state(msg);
            
            //Battery Charger On/Off
            if (act_id == ACTUATOR_CHARGE) {
                if (act_state == ACTUATOR_ON) {
                    BATTERY_CHARGER_EN(true);
                    BLUE_LED_SET(true); //temporarily commented out
                } else if (act_state == ACTUATOR_OFF) {
                    BATTERY_CHARGER_EN(false);
                    BLUE_LED_SET(false); //temporarily bye
                }
            }
            
            //RocketCAN 5V Line On/Off
    #if (BOARD_UNIQUE_ID == BOARD_ID_CHARGING_CAN)
            else if (act_id == ACTUATOR_CANBUS) {
                if (act_state == ACTUATOR_ON) {
                    CAN_5V_SET(true);
                    RED_LED_SET(true);
                } else if (act_state == ACTUATOR_OFF) {
                    CAN_5V_SET(false);
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
            RED_LED_SET(true); //Keto D3
            BLUE_LED_SET(true); //Keto D4
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
            float act_state = get_req_actuator_state_analog(msg);
            
            if(act_state == -1.0) break; //invalid message field or message type
            if(act_state > 1.0) act_state = 1.0; //bounds checking
            if(act_state < 0.0) act_state = 0.0;
            
            if (act_id == ACTUATOR_AIRBRAKES_SERVO) {
                if(state == COAST || (debug_en && debug_cmd_airbrakes_ext == act_state && state == PRE_FLIGHT))
                {
                    airbrakes_act_time = millis();
                    cmd_airbrakes_ext = act_state;
                    MOTOR_POWER = MOTOR_ON;
                    updatePulseWidth(act_state);
                }
            }
            
            else if (act_id == ACTUATOR_AIRBRAKES_ENABLE) {
                debug_cmd_airbrakes_ext = act_state;
                debug_en = true;
            }
             break;
             
            //Payload servo command logic
#elif (BOARD_UNIQUE_ID == BOARD_ID_CHARGING_PAYLOAD)
        case MSG_ACT_ANALOG_CMD:
            act_id = get_actuator_id(msg);
            if (act_id == ACTUATOR_PAYLOAD_SERVO) {
                cmd_payload_rpm = get_req_actuator_state_analog(msg);
            }
            break;
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
}

//Motor control functions
#if (BOARD_UNIQUE_ID == BOARD_ID_CHARGING_AIRBRAKE)
void actuate_airbrakes(float extension) {
    
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


void pwm_init(void){
    //1. Use the desired output pin RxyPPS control to select CCPx as the source and 
    //   disable the CCPx pin output driver by setting the associated TRIS bit.
    RB5PPS = 0b001011;
    TRISB5 = 1;
    
    //2. Load the T2PR register with the PWM period value.
    T2PR = 251;

    //3. Configure the CCP module for the PWM mode by loading the CCPxCON register with the appropriate values.
    CCP3CONbits.EN = 0b1; // enable CCP3
    CCP3CONbits.FMT = 0b0; // Set to right-aligned
    CCP3CONbits.MODE = 0b1100; // set to PWM operation
    

    //4. Load the CCPRxL register, and the CCPRxH register with the PWM duty cycle value and configure the FMT bit of the CCPxCON register
    //to set the proper register alignment.
    
    //for 500us (0 deg)
    CCPR3H = 0b00000000;
    CCPR3L = 0b10111100;
    //for 1.5 ms (90 deg))
    //CCPR3H = 0b00000010;
    //CCPR3L = 0b00110011;
    //for 2.4 ms (171 deg)
    //CCPR3H = 0b00000011;
    //CCPR3L = 0b10000100;
    //for 2.5 ms (180 deg))
    //CCPR3H = 0b00000011;
    //CCPR3L = 0b10101010;

    //5. Configure and start Timer2:
    //- Clear the TMR2IF interrupt flag bit of the respective PIR register. See Note below.
    PIR4bits.TMR2IF = 0;
    //- Select the timer clock source to be as FOSC/4 using the T2CLK register. 
    T2CLK = 0b0001; //(pg 321)
    //- Configure the CKPS bits of the T2CON register with the Timer prescale value.
    T2CONbits.CKPS = 0b111; //prescale of 128
    //- Enable the Timer by setting the ON bit of the T2CON register.
    T2CONbits.ON = 1; //enables timer
    //CCPTMRS1bits.P5TSEL = 0b00; //joes magic line (not magic))
    
    //6. Enable PWM output pin:
    //- Wait until the Timer overflows and the TMR2IF bit of the PIR4 register is set. See Note below.
    while (PIR4bits.TMR2IF == 0)
    {}
    //- Enable the CCPx pin output driver by clearing the associated TRIS bit.
    TRISB5 = 0;
}


/*
void pwm_init(void)
{
    
    //1. Disable PWMx by setting TRISx
    TRISB5 = 1; // motor PWM output (visualized on D6) //this would have set D7??? 
    
    //2. Clear PWMxCON
   
    PWM5CON = 0;
    
    //3. Load T2PR with period
    T2PR = 251; //initialize with 2700us -> 252.13 with a prescale of 128 (rounds to 252)
    
    //4. Load PWMxDCH and PWMxDCL<7:6> with duty cycle (ben says 2.5ms)
    //Loading min pulse width (500us) needs 187.5 in 10-bit combined register (rounds to 188) 
    //19 -> 00101111:00 (8bit * 4 + 2bit)
    PWM5DCH = 0b00101111;
    PWM5DCLbits.DC = 0b00;
    
    //5. Initialize timer
    
    PIR4bits.TMR2IF = 0;
    T2CLK = 0b0001; //selects Fosc/4 (pg 321)
    T2CONbits.CKPS = 0b111; //prescale of 128
    T2CONbits.ON = 1; //enables timer
    CCPTMRS1bits.P5TSEL = 0b00; //joes magic line (not magic))
    
    //6. Enable PWM output and wait for timer overflow, TMRxIF of PIR is set
    
    PWM5CONbits.EN = 1;
    while (PIR4bits.TMR2IF == 0)
    {}
    
    //7. Enable PWM pin output drivers
    TRISB5 = 0;
    RB5PPS = PWM5OUT; //map PWM5OUT to B5
    
    //8. Load appropriate values to PWMxCON
    
    PWM5CONbits.POL = 1; //set to non-inverting
    
    return;
}
 */

//function to take %extension and turn into bits to write to PWMxDCH and PWMxDCL

//void updatePulseWidth(float percent)
//{
//    uint16_t pulseWidth = (uint16_t)(percent * 4 * (T2PR + 1));
//    PWM5DCH = (pulseWidth >> 2) & 0xff;
//    PWM5DCLbits.DC = pulseWidth & 0x03;
//    return;
//}

#if (BOARD_UNIQUE_ID == BOARD_ID_CHARGING_AIRBRAKE)
 
void updatePulseWidth(float percent)
{
    uint32_t pulseWidth_us = (uint32_t) (MOTOR_MIN_PULSE_WIDTH_US + (percent) * (MOTOR_MAX_PULSE_WIDTH_US - MOTOR_MIN_PULSE_WIDTH_US));
    uint32_t bitWrite = (uint32_t) ((pulseWidth_us * 48) / 128); //48 is Fosc in MHz, 128 is prescaler
    //write PW/(Tosc * prescale value)
    CCPR3L = bitWrite & 0xFF;
    CCPR3H = (bitWrite >> 8) & 0x03; //honestly not sure abt this either this is like a very rough guess but as long as the servo wiggles its fine
    //CCPR3H = (bitWrite >> 8);
    //CCPR3L = (bitWrite - (CCPR3L << 8)); //this is sus idk how to bitwise operator //help this is really funny
}

#endif
