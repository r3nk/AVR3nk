/*!
*******************************************************************************
*******************************************************************************
** \brief   Analog Digital Converter
**
**          Please refer to the Atmel datasheet for detailed information about
**          the analog-to-digital converter and its technical characteristics.
**
** \author  Robin Klose
**
** Copyright (C) 2009-2013 Robin Klose
**
** This file is part of AVR3nk, available at https://github.com/r3nk/AVR3nk
**
*******************************************************************************
*******************************************************************************
*/

#ifndef ADC_H
#define ADC_H

#include <stdint.h>

//*****************************************************************************
//*************************** DEFINES AND MACROS ******************************
//*****************************************************************************


//*****************************************************************************
//******************************** DATA TYPES *********************************
//*****************************************************************************

/*!
*******************************************************************************
** \brief   Voltage Reference sources.
**
**          See ATmega644_20PU chapter 21.9.1 for detailed information.
**
*******************************************************************************
*/
typedef enum
{
    ADC_REF_AREF = 0,
    ADC_REF_AVCC = 1,
    ADC_REF_1100mV = 2,
    ADC_REF_2560mv = 3
} ADC_VoltageReferenceT;

/*!
*******************************************************************************
** \brief   Adjust the presentation of the ADC conversion in the
**          ADC Data Register.
**
** \sa      ATmega644_20PU chapter 21.9.1
**
*******************************************************************************
*/
typedef enum
{
    ADC_ADJUST_RIGHT = 0,
    ADC_ADJUST_LEFT  = 1
} ADC_ResultAdjustT;

/*!
*******************************************************************************
** \brief   Analog channel and gain selection.
**
**          This setting selects which combination of analog inputs are
**          connected to the ADC. It also select the gain for the differential
**          channels.
**
** \sa      ATmega644_20PU chapter 21.9.1
**
*******************************************************************************
*/
typedef enum
{
    ADC_MUX_SINGLE_ADC0 = 0,
    ADC_MUX_SINGLE_ADC1,
    ADC_MUX_SINGLE_ADC2,
    ADC_MUX_SINGLE_ADC3,
    ADC_MUX_SINGLE_ADC4,
    ADC_MUX_SINGLE_ADC5,
    ADC_MUX_SINGLE_ADC6,
    ADC_MUX_SINGLE_ADC7,
    ADC_MUX_DIFFERENTIAL_ADC0_ADC0_10x,
    ADC_MUX_DIFFERENTIAL_ADC1_ADC0_10x,
    ADC_MUX_DIFFERENTIAL_ADC0_ADC0_200x,
    ADC_MUX_DIFFERENTIAL_ADC1_ADC0_200x,
    ADC_MUX_DIFFERENTIAL_ADC2_ADC2_10x,
    ADC_MUX_DIFFERENTIAL_ADC3_ADC2_10x,
    ADC_MUX_DIFFERENTIAL_ADC2_ADC2_200x,
    ADC_MUX_DIFFERENTIAL_ADC3_ADC2_200x,
    ADC_MUX_DIFFERENTIAL_ADC0_ADC1_1x,
    ADC_MUX_DIFFERENTIAL_ADC1_ADC1_1x,
    ADC_MUX_DIFFERENTIAL_ADC2_ADC1_1x,
    ADC_MUX_DIFFERENTIAL_ADC3_ADC1_1x,
    ADC_MUX_DIFFERENTIAL_ADC4_ADC1_1x,
    ADC_MUX_DIFFERENTIAL_ADC5_ADC1_1x,
    ADC_MUX_DIFFERENTIAL_ADC6_ADC1_1x,
    ADC_MUX_DIFFERENTIAL_ADC7_ADC1_1x,
    ADC_MUX_DIFFERENTIAL_ADC0_ADC2_1x,
    ADC_MUX_DIFFERENTIAL_ADC1_ADC2_1x,
    ADC_MUX_DIFFERENTIAL_ADC2_ADC2_1x,
    ADC_MUX_DIFFERENTIAL_ADC3_ADC2_1x,
    ADC_MUX_DIFFERENTIAL_ADC4_ADC2_1x,
    ADC_MUX_DIFFERENTIAL_ADC5_ADC2_1x,
    ADC_MUX_SINGLE_1100mV,
    ADC_MUX_SINGLE_GND
} ADC_MuxSelectT;

/*!
*******************************************************************************
** \brief   ADC clock prescaler.
**
**          This setting determines the division factor between the crystal
**          frequency and the input clock to the ADC.
**
** \sa      ATmega644_20PU chapter 21.9.2
**
*******************************************************************************
*/
typedef enum
{
    ADC_PRESCALER_2 = 1,
    ADC_PRESCALER_4,
    ADC_PRESCALER_8,
    ADC_PRESCALER_16,
    ADC_PRESCALER_32,
    ADC_PRESCALER_64,
    ADC_PRESCALER_128
} ADC_ClockPrescalerT;

/*!
*******************************************************************************
** \brief   ADC auto trigger select.
**
** \sa      ATmega644_20PU chapter 21.9.4
**
*******************************************************************************
*/
typedef enum
{
    ADC_TRIGGER_FREE_RUNNING = 0,
    ADC_TRIGGER_ANALOG_COMPARATOR,
    ADC_TRIGGER_EXT_IRQ_0,
    ADC_TRIGGER_TIMER_0_COMPARE_MATCH,
    ADC_TRIGGER_TIMER_0_OVERFLOW,
    ADC_TRIGGER_TIMER_1_COMPARE_MATCH_B,
    ADC_TRIGGER_TIMER_1_OVERFLOW,
    ADC_TRIGGER_TIMER_1_CAPTURE_EVENT
} ADC_TriggerSourceT;

/*!
*******************************************************************************
** \brief   Binary enable/disable enumeration for the ADC.
**
*******************************************************************************
*/
typedef enum
{
    ADC_DISABLE = 0,
    ADC_ENABLE
} ADC_EnableDisableT;

//*****************************************************************************
//************************* FUNCTION DECLARATIONS *****************************
//*****************************************************************************

void ADC_SetVoltageReference(ADC_VoltageReferenceT reference);
void ADC_SetResultAdjust(ADC_ResultAdjustT resultAdjust);
void ADC_SetMuxSelect(ADC_MuxSelectT muxSelect);
void ADC_SetClockPrescaler(ADC_ClockPrescalerT clkPs);
void ADC_SetHardwareEnable(ADC_EnableDisableT mode);
void ADC_SetAutoTriggerEnable(ADC_EnableDisableT mode);
void ADC_SetAutoTriggerSource(ADC_TriggerSourceT source);
void ADC_SetInterruptEnable(ADC_EnableDisableT mode);
void ADC_SetDigitalInputEnable(ADC_EnableDisableT mode);
void ADC_StartConversion(void);


#endif // ADC_H
