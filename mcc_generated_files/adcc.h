/**
  ADCC Generated Driver API Header File

  @Company
    Microchip Technology Inc.

  @File Name
    adcc.h

  @Summary
    This is the generated header file for the ADCC driver using PIC10 / PIC12 / PIC16 / PIC18 MCUs 

  @Description
    This header file provides APIs for driver for ADCC.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.65.2
        Device            :  PIC18F26K83
        Driver Version    :  2.01
    The generated drivers are tested against the following:
        Compiler          :  XC8 1.45
        MPLAB             :  MPLAB X 4.15
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

#ifndef ADCC_H
#define ADCC_H

/**
  Section: Included Files
*/

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif

/**
  Section: Data Types Definitions
*/

/**
 *  result size of an A/D conversion
 */

typedef uint16_t adc_result_t;

/** ADC Channel Definition

 @Summary
   Defines the channels available for conversion.

 @Description
   This routine defines the channels that are available for the module to use.

 Remarks:
   None
 */

typedef enum {
    channel_POWER_V5  = 0x8,
    channel_POWER_V13 = 0x9,
    channel_CHARGE_CURR = 0x4,
    channel_MOTOR_CURR = 0xa,
    channel_BATT_CURR = 0x17,
    channel_BATT_VOLT = 0x12,
    channel_GROUND_VOLT  = 0x13,
} adcc_channel_t; //page 677

/**
  Section: ADC Module APIs
*/

/**
  @Summary
    Initializes the ADCC

  @Description
    This routine initializes the Initializes the ADCC.
    This routine must be called before any other ADCC routine is called.
    This routine should only be called once during system initialization.

  @Preconditions
    None

  @Param
    None

  @Returns
    None

  @Comment
    

  @Example
    <code>
    uint16_t convertedValue;    

    ADCC_Initialize();
    convertedValue = ADCC_GetSingleConversion(AN1_Channel);
    </code>
*/
void ADCC_Initialize(void);

/**
  @Summary
     Allows selection of a channel for conversion

  @Description
    This routine is used to select desired channel for conversion.
    available
    
  @Preconditions
    ADCC_Initialize() function should have been called before calling this function.

  @Returns
    None

  @Param
    Pass in required channel number
    "For available channel refer to enum under adcc.h file"

  @Example
    <code>
    uint16_t convertedValue; 

    ADCC_Initialize();   
    ADCC_StartConversion(AN1_Channel);
    convertedValue = ADCC_GetConversionResult();
    </code>
*/
void ADCC_StartConversion(adcc_channel_t channel);

/**
  @Summary
    Returns true when the conversion is completed otherwise false.

  @Description
    This routine is used to determine if conversion is completed.
    When conversion is complete routine returns true. It returns false otherwise.

  @Preconditions
    ADCC_Initialize() and ADCC_StartConversion(adcc_channel_t channel)
    function should have been called before calling this function.

  @Returns
    true  - If conversion is complete
    false - If conversion is not completed

  @Param
    None

  @Example
    <code>
    uint16_t convertedValue;    

    ADCC_Initialize();    
    ADCC_StartConversion(AN1_Channel);
    while(!ADCC_IsConversionDone());
    convertedValue = ADCC_GetConversionResult();
    </code>
 */
bool ADCC_IsConversionDone();

/**
  @Summary
    Returns the ADCC conversion value.

  @Description
    This routine is used to get the analog to digital converted value. This
    routine gets converted values from the channel specified.

  @Preconditions
    This routine returns the conversion value only after the conversion is complete.
    Completion status can be checked using
    ADCC_IsConversionDone() routine.

  @Returns
    Returns the converted value.

  @Param
    None

  @Example
    <code>
    uint16_t convertedValue;    

    ADCC_Initialize();    
    ADCC_StartConversion(AN1_Channel);
    while(!ADCC_IsConversionDone());
    convertedValue = ADCC_GetConversionResult();
    </code>
 */
adc_result_t ADCC_GetConversionResult(void);

/**
  @Summary
    Returns the ADCC conversion value
    also allows selection of a channel for conversion.

  @Description
    This routine is used to select desired channel for conversion
    and to get the analog to digital converted value after a single conversion.

  @Preconditions
    ADCC_Initialize() and ADCC_DisableContinuousConversion() functions should have been called before calling this function.

  @Returns
    Returns the converted value.

  @Param
    Pass in required channel number.
    "For available channel refer to enum under adcc.h file"

  @Example
    <code>
    uint16_t convertedValue;

    ADCC_Initialize();
    ADCC_DisableContinuousConversion();
	
    convertedValue = ADCC_GetSingleConversion(AN1_Channel);
    </code>
*/
adc_result_t ADCC_GetSingleConversion(adcc_channel_t channel);

/**
  @Summary
    Stops the ADCC conversion by resetting the ADGO bit.

  @Description
    None

  @Preconditions
    ADCC_Initialize() and ADCC_StartConversion() functions should have been called before calling this function.

  @Returns
    None

  @Param
    None

  @Example
    <code>
    ADCC_Initialize();
	ADCC_StartConversion(AN1_Channel);
	ADCC_StopConversion();
    </code>
*/
void ADCC_StopConversion(void);

/**
  @Summary
    Stops the ADCC from automatic retriggering in continuous sampling mode

  @Description
    None

  @Preconditions
    ADCC_Initialize() and ADCC_EnableContinuousConversion() functions should have been called before calling this function.

  @Returns
    None

  @Param
    None

  @Example
    <code>
        ADCC_Initialize();
	ADCC_EnableContinuousConversion();	
    </code>
*/
void ADCC_SetStopOnInterrupt(void);

/**
  @Summary
    Discharges the input sample capacitor by setting the channel to AVss.

  @Description
    None

  @Preconditions
    None.

  @Returns
    None

  @Param
    None

  @Example
    <code>
    </code>
*/
void ADCC_DischargeSampleCapacitor(void); 

/**
  @Summary
    Loads the ADACQ register by the 8 bit value sent as argument.

  @Description
    None

  @Preconditions
    None.

  @Returns
    None

  @Param
    8 bit value to be set in the acquisition register.

  @Example
    <code>
        uint16_t acquisitionValue = 98;
	ADCC_LoadAcquisitionRegister(acquisitionValue);
    </code>
*/
void ADCC_LoadAcquisitionRegister(uint16_t);
/**
  @Summary
    Loads the ADPRE register by the 8 bit value sent as argument.

  @Description
    None

  @Preconditions
    None.

  @Returns
    None

  @Param
    8 bit value to be set in the precharge register.

  @Example
    <code>
        uint16_t prechargeTime = 98;
	ADCC_SetPrechargeTime(prechargeTime);
    </code>
*/
void ADCC_SetPrechargeTime(uint16_t);

/**
  @Summary
    Loads the ADRPT register by the 8 bit value sent as argument.

  @Description
    None

  @Preconditions
    None.

  @Returns
    None

  @Param
    8 bit value to be set in the Repeat register.

  @Example
    <code>
	uint8_t repeat = 98;
	ADCC_SetRepeatCount(repeat);
    </code>
*/
void ADCC_SetRepeatCount(uint8_t);

/**
  @Summary
    Returns the ADCNT register contents.

  @Description
    None

  @Preconditions
    None.

  @Returns
    Contents of ADCNT register

  @Param
    None.

  @Example
    <code>
	uint8_t count = ADCC_GetCurrentCountofConversions();
	
    </code>
*/
uint8_t ADCC_GetCurrentCountofConversions(void);

/**
  @Summary
    Clears the ADACLR bit.

  @Description
    None

  @Preconditions
    None.

  @Returns
    None

  @Param
    None.

  @Example
    <code>
	ADCC_ClearAccumulator();
	
    </code>
*/
void ADCC_ClearAccumulator(void);

/**
  @Summary
   Returns the value obtained from ADACCU, ADACCH and ADACCL registers.

  @Description
    None

  @Preconditions
    None.

  @Returns
    Value obtained from ADACCU, ADACCH and ADACCL registers.

  @Param
    None.

  @Example
    <code>
        uint32_t accumulatorValue = ADCC_GetAccumulatorValue();
    </code>
*/
int24_t ADCC_GetAccumulatorValue(void);

/**
  @Summary
   Returns the contents of ADAOV setting.

  @Description
    None

  @Preconditions
    None.

  @Returns
    Value of ADAOV bit.

  @Param
    None.

  @Example
    <code>
	bool accumulatorOverflow = ADCC_HasAccumulatorOverflowed();
	
    </code>
*/
bool ADCC_HasAccumulatorOverflowed(void);

/**
  @Summary
   Returns the contents of ADFLTRH and ADFLTRL registers.

  @Description
    None

  @Preconditions
    None.

  @Returns
    16 bit value obtained from ADFLTRH and ADFLTRL registers.

  @Param
    None.

  @Example
    <code>
	uint16_t filterValue = ADCC_GetFilterValue();
	
    </code>
*/
uint16_t ADCC_GetFilterValue(void);

/**
  @Summary
   Returns the contents of ADPREVH and ADPREVL registers.

  @Description
    None

  @Preconditions
    None.

  @Returns
    16 bit value obtained from ADPREVH and ADPREVL registers.

  @Param
    None.

  @Example
    <code>
	uint16_t prevResult = ADCC_GetPreviousResult();
	
    </code>
*/
uint16_t ADCC_GetPreviousResult(void);

/**
  @Summary
   Sets the ADSTPTH and ADSTPTL registers.

  @Description
    None

  @Preconditions
    None

  @Returns
    None

  @Param
    16 bit value for set point.

  @Example
    <code>
	uint16_t setPoint = 90;
	ADCC_DefineSetPoint(setPoint);
    </code>
*/
void ADCC_DefineSetPoint(uint16_t);

/**
  @Summary
   Sets the ADUTHH and ADUTHL register.

  @Description
    None

  @Preconditions
    None

  @Returns
    None

  @Param
    16 bit value for upper threshold.

  @Example
    <code>
	uint16_t upperThreshold = 90;
	ADCC_SetUpperThreshold(upperThreshold);
	
    </code>
*/
void ADCC_SetUpperThreshold(uint16_t);

/**
  @Summary
   Sets the ADLTHH and ADLTHL register.

  @Description
    None

  @Preconditions
    None

  @Returns
    None

  @Param
    16 bit value for lower threshold..

  @Example
    <code>
	uint16_t lowerThreshold = 90;
	ADCC_SetLowerThreshold(lowerThreshold);
	
    </code>
*/
void ADCC_SetLowerThreshold(uint16_t);

/**
  @Summary
   Returns the 16 bit value obtained from ADERRH and ADERRL registers.

  @Description
    None

  @Preconditions
    None

  @Returns
    16 bit value obtained from ADERRH and ADERRL registers.

  @Param
    None.

  @Example
    <code>
	uint16_t error = ADCC_GetErrorCalculation(void);
    </code>
*/
uint16_t ADCC_GetErrorCalculation(void);

/**
  @Summary
   Sets the ADDSEN bit.

  @Description
    None

  @Preconditions
    None

  @Returns
    None

  @Param
    None.

  @Example
    <code>
    </code>
*/
void ADCC_EnableDoubleSampling(void);

/**
  @Summary
   Sets the ADCONT bit.

  @Description
    None

  @Preconditions
    None

  @Returns
    None

  @Param
    None.

  @Example
    <code>
    </code>
*/
void ADCC_EnableContinuousConversion(void);

/**
  @Summary
   Resets the ADCONT bit.

  @Description
    None

  @Preconditions
    None

  @Returns
    None

  @Param
    None.

  @Example
    <code>
    </code>
*/
void ADCC_DisableContinuousConversion(void);

/**
  @Summary
   Returns the value of ADUTHR bit.

  @Description
    None

  @Preconditions
    None

  @Returns
    Returns the value of ADUTHR bit.

  @Param
    None.

  @Example
    <code>
    </code>
*/
bool ADCC_HasErrorCrossedUpperThreshold(void);

/**
  @Summary
   Returns the value of ADLTHR bit.

  @Description
    None

  @Preconditions
    None

  @Returns
    Returns the value of ADLTHR bit.

  @Param
    None.

  @Example
    <code>
    </code>
*/
bool ADCC_HasErrorCrossedLowerThreshold(void);

/**
  @Summary
   Returns value of ADSTAT setting.

  @Description
    None

  @Preconditions
    None

  @Returns
    Returns value of ADSTAT setting.

  @Param
    None.

  @Example
    <code>
    </code>
*/
uint8_t ADCC_GetConversionStageStatus(void);




#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif

#endif	//ADCC_H
/**
 End of File
*/

