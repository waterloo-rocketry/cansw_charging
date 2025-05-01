#include "canlib/canlib.h"
#include "mcc_generated_files/adcc.h"
#include "mcc_generated_files/fvr.h"

#include "error_checks.h"
#include "platform.h"

/*
 * 
 */

#if (BOARD_UNIQUE_ID == BOARD_ID_CHARGING_AIRBRAKE)
const uint16_t MOTOR_MIN_PULSE_WIDTH_US = 500; // corresponds to 0 deg
const uint16_t MOTOR_MAX_PULSE_WIDTH_US = 2500; // corresponds to 117 deg
#elif (BOARD_UNIQUE_ID == BOARD_ID_CHARGING_PAYLOAD)
const uint16_t MOTOR_MIN_PULSE_WIDTH_US = 1500;
const uint16_t MOTOR_MAX_PULSE_WIDTH_US = 1900; 
#endif

//Motor control functions
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
    
    #if (BOARD_UNIQUE_ID == BOARD_ID_CHARGING_AIRBRAKE)
    CCPR3H = 0b00000000;
    CCPR3L = 0b10111100;
    #elif (BOARD_UNIQUE_ID == BOARD_ID_CHARGING_PAYLOAD)
    CCPR3H = 0b00000010;
    CCPR3L = 0b00110011;
    #endif

    //5. Configure and start Timer2:
    //- Clear the TMR2IF interrupt flag bit of the respective PIR register. See Note below.
    PIR4bits.TMR2IF = 0;
    //- Select the timer clock source to be as FOSC/4 using the T2CLK register. 
    T2CLK = 0b0001; //(pg 321)
    //- Configure the CKPS bits of the T2CON register with the Timer prescale value.
    T2CONbits.CKPS = 0b111; //prescale of 128
    //- Enable the Timer by setting the ON bit of the T2CON register.
    T2CONbits.ON = 1; //enables timer
    
    //6. Enable PWM output pin:
    //- Wait until the Timer overflows and the TMR2IF bit of the PIR4 register is set. See Note below.
    while (PIR4bits.TMR2IF == 0)
    {}
    
    //- Enable the CCPx pin output driver by clearing the associated TRIS bit.
    TRISB5 = 0;
}

#if (BOARD_UNIQUE_ID == BOARD_ID_CHARGING_AIRBRAKE || BOARD_UNIQUE_ID == BOARD_ID_CHARGING_PAYLOAD)
 
void updatePulseWidth(uint8_t percent)
{
    uint32_t pulseWidth_us = (uint32_t) (MOTOR_MIN_PULSE_WIDTH_US + ((float)percent / 100.0) * (MOTOR_MAX_PULSE_WIDTH_US - MOTOR_MIN_PULSE_WIDTH_US));
    uint32_t bitWrite = (uint32_t) ((pulseWidth_us * 48) / 128); //48 is Fosc in MHz, 128 is prescaler
    //write PW/(Tosc * prescale value)
    CCPR3L = bitWrite & 0xFF;
    CCPR3H = (bitWrite >> 8) & 0x03; //honestly not sure abt this either this is like a very rough guess but as long as the servo wiggles its fine
}

#endif

