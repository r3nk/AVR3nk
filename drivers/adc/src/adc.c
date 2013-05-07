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

#include <stdint.h>
#include <avr/io.h>
#include "adc.h"

//*****************************************************************************
//*************************** DEFINES AND MACROS ******************************
//*****************************************************************************

//******** ATmega16 setup ********

#ifdef atmega16

    #define ADCSRB  SFIOR

#endif // atmega16

//*****************************************************************************
//**************************** LOCAL DATA TYPES *******************************
//*****************************************************************************


//*****************************************************************************
//**************************** LOCAL VARIABLES ********************************
//*****************************************************************************

static struct
{
    uint8_t digitalInputEnable  : 1;
    uint8_t muxSelect           : 5;
} adcState = {ADC_ENABLE, ADC_MUX_SINGLE_ADC0};

//*****************************************************************************
//********************** LOCAL FUNCTION DECLARATIONS **************************
//*****************************************************************************

static void adcDigitalInputEnable(void);

//*****************************************************************************
//**************************** LOCAL FUNCTIONS ********************************
//*****************************************************************************

/*!
*******************************************************************************
** \brief   Enable or disable digital inputs for the pins which are used
**          by the ADC.
**
**          The action taken depends on the ADC driver state variables.
**
*******************************************************************************
*/
#if (defined atmega644) || (defined atmega644p)
static void adcDigitalInputEnable(void)
{
    DIDR0 = 0x00;
    if (adcState.digitalInputEnable == ADC_ENABLE)
    {
        return;
    }
    else
    {
        switch(adcState.muxSelect)
        {
            case(ADC_MUX_SINGLE_ADC0):
            case(ADC_MUX_DIFFERENTIAL_ADC0_ADC0_10x):
            case(ADC_MUX_DIFFERENTIAL_ADC0_ADC0_200x):
            case(ADC_MUX_DIFFERENTIAL_ADC0_ADC1_1x):
            case(ADC_MUX_DIFFERENTIAL_ADC0_ADC2_1x):
                DIDR0 |= (1 << ADC0D);
                break;
            case(ADC_MUX_SINGLE_ADC1):
            case(ADC_MUX_DIFFERENTIAL_ADC1_ADC0_10x):
            case(ADC_MUX_DIFFERENTIAL_ADC1_ADC0_200x):
            case(ADC_MUX_DIFFERENTIAL_ADC1_ADC1_1x):
            case(ADC_MUX_DIFFERENTIAL_ADC1_ADC2_1x):
                DIDR0 |= (1 << ADC1D);
                break;
            case(ADC_MUX_SINGLE_ADC2):
            case(ADC_MUX_DIFFERENTIAL_ADC2_ADC2_10x):
            case(ADC_MUX_DIFFERENTIAL_ADC2_ADC2_200x):
            case(ADC_MUX_DIFFERENTIAL_ADC2_ADC1_1x):
            case(ADC_MUX_DIFFERENTIAL_ADC2_ADC2_1x):
                DIDR0 |= (1 << ADC2D);
                break;
            case(ADC_MUX_SINGLE_ADC3):
            case(ADC_MUX_DIFFERENTIAL_ADC3_ADC2_10x):
            case(ADC_MUX_DIFFERENTIAL_ADC3_ADC2_200x):
            case(ADC_MUX_DIFFERENTIAL_ADC3_ADC1_1x):
            case(ADC_MUX_DIFFERENTIAL_ADC3_ADC2_1x):
                DIDR0 |= (1 << ADC3D);
                break;
            case(ADC_MUX_SINGLE_ADC4):
            case(ADC_MUX_DIFFERENTIAL_ADC4_ADC1_1x):
            case(ADC_MUX_DIFFERENTIAL_ADC4_ADC2_1x):
                DIDR0 |= (1 << ADC4D);
                break;
            case(ADC_MUX_SINGLE_ADC5):
            case(ADC_MUX_DIFFERENTIAL_ADC5_ADC1_1x):
            case(ADC_MUX_DIFFERENTIAL_ADC5_ADC2_1x):
                DIDR0 |= (1 << ADC5D);
                break;
            case(ADC_MUX_SINGLE_ADC6):
            case(ADC_MUX_DIFFERENTIAL_ADC6_ADC1_1x):
                DIDR0 |= (1 << ADC6D);
                break;
            case(ADC_MUX_SINGLE_ADC7):
            case(ADC_MUX_DIFFERENTIAL_ADC7_ADC1_1x):
                DIDR0 |= (1 << ADC7D);
                break;
            case(ADC_MUX_SINGLE_1100mV):
            case(ADC_MUX_SINGLE_GND):
            default:
                break;
        }
        switch(adcState.muxSelect)
        {
            case(ADC_MUX_DIFFERENTIAL_ADC0_ADC0_10x):
            case(ADC_MUX_DIFFERENTIAL_ADC1_ADC0_10x):
            case(ADC_MUX_DIFFERENTIAL_ADC0_ADC0_200x):
            case(ADC_MUX_DIFFERENTIAL_ADC1_ADC0_200x):
                DIDR0 |= (1 << ADC0D);
                break;
            case(ADC_MUX_DIFFERENTIAL_ADC0_ADC1_1x):
            case(ADC_MUX_DIFFERENTIAL_ADC2_ADC1_1x):
            case(ADC_MUX_DIFFERENTIAL_ADC3_ADC1_1x):
            case(ADC_MUX_DIFFERENTIAL_ADC4_ADC1_1x):
            case(ADC_MUX_DIFFERENTIAL_ADC5_ADC1_1x):
            case(ADC_MUX_DIFFERENTIAL_ADC6_ADC1_1x):
            case(ADC_MUX_DIFFERENTIAL_ADC7_ADC1_1x):
                DIDR0 |= (1 << ADC1D);
                break;
            case(ADC_MUX_DIFFERENTIAL_ADC3_ADC2_10x):
            case(ADC_MUX_DIFFERENTIAL_ADC3_ADC2_200x):
            case(ADC_MUX_DIFFERENTIAL_ADC0_ADC2_1x):
            case(ADC_MUX_DIFFERENTIAL_ADC1_ADC2_1x):
            case(ADC_MUX_DIFFERENTIAL_ADC3_ADC2_1x):
            case(ADC_MUX_DIFFERENTIAL_ADC4_ADC2_1x):
            case(ADC_MUX_DIFFERENTIAL_ADC5_ADC2_1x):
                DIDR0 |= (1 << ADC2D);
                break;
            default:
                break;
        }
    }
    return;
}
#else
static void adcDigitalInputEnable(void)
{
    return;
}
#endif // atmega644 || atmega644p

//*****************************************************************************
//*************************** PUBLIC FUNCTIONS ********************************
//*****************************************************************************

/*!
*******************************************************************************
** \brief   Set up the voltage reference source.
**
**          This function selects the voltage reference for the ADC.
**          If this setting is changed during a conversion, the change will
**          not go in effect until this conversion is complete (ADIF in ADCSRA
**          is set). The internal voltage reference options may not be used if
**          an external reference voltage is being applied to the AREF pin.
**
** \sa      ATmega644_20PU chapter 21.9.1
**
*******************************************************************************
*/
void ADC_SetVoltageReference(ADC_VoltageReferenceT reference)
{
    ADMUX = (ADMUX & ~((1 << REFS1) | (1 << REFS0))) | (reference << REFS0);
    return;
}

/*!
*******************************************************************************
** \brief   Set up whether the conversion result of the ADC will be aligned
**          to the left or to the right side of the concatenation of
**          ADCH and ADCL. This setting will affect the ADC data register
**          immediately, regardless of any ongoing conversions.
**
** \sa      ATmega644_20PU chapter 21.9.1
**
*******************************************************************************
*/
void ADC_SetResultAdjust(ADC_ResultAdjustT resultAdjust)
{
    ADMUX = (ADMUX & ~(1 << ADLAR)) | (resultAdjust << ADLAR);
    return;
}

/*!
*******************************************************************************
** \brief   Analog channel and gain selection.
**
**          This procedure sets up a combination of analog inputs that
**          are connected to the ADC. The gain for the differential channels
**          is also selected. If the setting is changed during a conversion,
**          the change will not go in effect until the conversion is complete.
**
** \sa      ATmega644_20PU chapter 21.9.1
**
*******************************************************************************
*/
void ADC_SetMuxSelect(ADC_MuxSelectT muxSelect)
{
    ADMUX = (ADMUX & ~(0x1F)) | muxSelect;
    adcState.muxSelect = muxSelect;
    adcDigitalInputEnable();
    return;
}

/*!
*******************************************************************************
** \brief   ADC prescaler selection.
**
**          This setting determines the division factor between the crystal
**          frequency and the input clock to the ADC.
**
** \sa      ATmega644_20PU chapter 21.9.2
**
*******************************************************************************
*/
void ADC_SetClockPrescaler(ADC_ClockPrescalerT clkPs)
{
    ADCSRA = (ADCSRA & ~((1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0))) | clkPs;
    return;
}

/*!
*******************************************************************************
** \brief   ADC enable.
**
**          This setting enables or disables the ADC hardware block.
**
** \sa      ATmega644_20PU chapter 21.9.2
**
*******************************************************************************
*/
void ADC_SetHardwareEnable(ADC_EnableDisableT mode)
{
    if(mode == ADC_DISABLE)
    {
        ADCSRA &= ~(1 << ADEN);
    }
    else
    {
        ADCSRA |= (1 << ADEN);
    }
    return;
}

/*!
*******************************************************************************
** \brief   ADC auto trigger enable.
**
**          This setting enables or disables the ADC auto trigger
**          functionality. When active, the auto trigger source is selected
**          by ADC_SetAutoTriggerSource().
**
** \sa      ADC_SetAutoTriggerSource()
**
** \sa      ATmega644_20PU chapter 21.9.2
**
*******************************************************************************
*/
void ADC_SetAutoTriggerEnable(ADC_EnableDisableT mode)
{
    if(mode == ADC_DISABLE)
    {
        ADCSRA &= ~(1 << ADATE);
    }
    else
    {
        ADCSRA |= (1 << ADATE);
    }
    return;
}

/*!
*******************************************************************************
** \brief   ADC auto trigger select.
**
**          This setting selects the ADC auto trigger source.
**
** \sa      ADC_SetAutoTriggerEnable()
**
** \sa      ATmega644_20PU chapter 21.9.4
**
*******************************************************************************
*/
void ADC_SetAutoTriggerSource(ADC_TriggerSourceT source)
{
    ADCSRB = (ADCSRB & ~0x07) | source;
    return;
}

/*!
*******************************************************************************
** \brief   ADC irq enable.
**
**          This setting enables or disables the ADC interrupt.
**
** \note    The interrupt service routine must be specified by the
**          application software!
**
** \sa      ATmega644_20PU chapter 21.9.2
**
*******************************************************************************
*/
void ADC_SetInterruptEnable(ADC_EnableDisableT mode)
{
    if(mode == ADC_DISABLE)
    {
        ADCSRA &= ~(1 << ADIE);
    }
    else
    {
        ADCSRA |= (1 << ADIE);
    }
    return;
}

/*!
*******************************************************************************
** \brief   Decides whether the digital input buffer of pins that are used
**          for the ADC should be disabled for power-saving reasons.
**
**          When this option is set as ADC_DISABLE, the input buffers will
**          be disabled.
**          This feature is supported by the following devices:
**          atmega644, atmega644p
**
** \sa      ATmega644_20PU chapter 21.9.5
**
*******************************************************************************
*/
void ADC_SetDigitalInputEnable(ADC_EnableDisableT mode)
{
    adcState.digitalInputEnable = mode;
    adcDigitalInputEnable();
    return;
}

/*!
*******************************************************************************
** \brief   Start a conversion.
**
**          Initiates a conversion manually.
**
** \sa      ATmega644_20PU chapter 21.9.2
**
*******************************************************************************
*/
void ADC_StartConversion(void)
{
    ADCSRA |= (1 << ADSC);
    return;
}


//*****************************************************************************
//*********************** INTERRUPT SERVICE ROUTINES **************************
//*****************************************************************************
