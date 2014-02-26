/*!
*******************************************************************************
*******************************************************************************
** \brief   Timer driver with support for PWM (Pulse Width Modulation)
**
**          This driver allows to operate the AVRs' hardware timers
**          independently from each other.
**          The initialization function TIMER_Init() returns a handle,
**          which is used in all other functions to identifiy the corresponding
**          timer. The timer can be started by TIMER_Start() and stopped by
**          TIMER_Stop(). When calling TIMER_OneShot(), the timer runs for
**          one iteration, i.e., it stops after its next overflow.
**
** \attention
**          In order to safely call TIMER functions from within other ISRs,
**          set the TIMER_INTERRUPT_SAFETY macro to 1.
**
** \author  Robin Klose
**
** Copyright (C) 2014-2014 Robin Klose
**
** This file is part of AVR3nk, available at https://github.com/r3nk/AVR3nk
**
*******************************************************************************
*******************************************************************************
*/

#include <stdint.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <setjmp.h>
#include <util/atomic.h>
#include <drivers/macros_pin.h>
#include "timer.h"

//*****************************************************************************
//*************************** DEFINES AND MACROS ******************************
//*****************************************************************************

#define TIMER_NUMBER_OF_TIMERS 3

#define PIN_OC0A B,3
#define PIN_OC0B B,4
#define PIN_OC1A D,5
#define PIN_OC1B D,4
#define PIN_OC2A D,7
#define PIN_OC2B D,6

//*****************************************************************************
//**************************** LOCAL DATA TYPES *******************************
//*****************************************************************************

// Timer/counter registers can be 8-bit and 16-bit. For details, see:
// /opt/local/avr/include/avr/{iom644p.h, iomxx4.h, sfr_defs.h}
typedef union
{
    volatile uint8_t*   uint8Ptr;
    volatile uint16_t*  uint16Ptr;
} timerRegisterT;

// Distinguish between 8-bit and 16-bit timers:
typedef enum
{
    timerBitWidth_8 = 0,
    timerBitWidth_16
} timerBitWidthT;

// Timer states:
typedef enum
{
    timerStateStopped = 0,
    timerStateRunning,
    timerStateOneShot
} timerStateT;

// Internal driver handle structure:
typedef struct
{
    uint8_t               initialized : 1;
    TIMER_TimerIdT        timerId : 2;
    timerBitWidthT        bitWidth : 1;
    TIMER_OutputModeT     outputModeA : 2;
    TIMER_OutputModeT     outputModeB : 2;
    TIMER_ClockPrescalerT clockPrescaler : 3;
    TIMER_WaveGenerationT waveGenerationMode : 3;
    timerStateT           timerState : 2;
    volatile uint16_t     overflowCallbackCounter;
    uint16_t              overflowCallbackPeriod;
    TIMER_CallbackT       overflowCallbackPtr;
    void*                 overflowCallbackArgPtr;
    volatile uint8_t*     tccraPtr;
    volatile uint8_t*     tccrbPtr;
    volatile uint8_t*     timskPtr;
    volatile uint8_t*     tifrPtr;
    timerRegisterT        tcnt;
    timerRegisterT        ocra;
    timerRegisterT        ocrb;
} timerHandleT;


//*****************************************************************************
//**************************** LOCAL VARIABLES ********************************
//*****************************************************************************

static timerHandleT timerHandleArr[TIMER_NUMBER_OF_TIMERS];


//*****************************************************************************
//********************** LOCAL FUNCTION DECLARATIONS **************************
//*****************************************************************************

static void   timerResetRegisters (timerHandleT* handlePtr);
static int8_t timerSetCompareOutputMode (timerHandleT* handlePtr);
static int8_t timerSetWaveGenerationMode (timerHandleT* handlePtr);
static int8_t timerCheckClockPrescaler (timerHandleT* handlePtr,
                                        TIMER_ClockPrescalerT clockPrescaler);
static int8_t timerStartClock (timerHandleT* handlePtr);
static int8_t timerSetPinsAsOutputs (timerHandleT* handlePtr);
static int8_t timerSetPinsAsInputs (timerHandleT* handlePtr);
static inline void timerStopClock (timerHandleT* handlePtr);
static inline void timerResetTimerRegister (timerHandleT* handlePtr);
static void   timerOverflowHandler (timerHandleT* handlePtr);

//*****************************************************************************
//**************************** LOCAL FUNCTIONS ********************************
//*****************************************************************************

/*!
*******************************************************************************
** \brief   Set all registers associated with the handle to 0.
**
** \param   handlePtr   The handle must be checked by the caller.
**
*******************************************************************************
*/
static void timerResetRegisters (timerHandleT* handlePtr)
{
    // Reset all registers:
    *handlePtr->tccraPtr = 0;
    *handlePtr->tccrbPtr = 0;
    *handlePtr->timskPtr = 0;
    *handlePtr->tifrPtr = 0xFF;
    if (handlePtr->bitWidth == timerBitWidth_8)
    {
        *handlePtr->tcnt.uint8Ptr = 0;
        *handlePtr->ocra.uint8Ptr = 0;
        *handlePtr->ocrb.uint8Ptr = 0;
    }
    else
    {
        *handlePtr->tcnt.uint16Ptr = 0;
        *handlePtr->ocra.uint16Ptr = 0;
        *handlePtr->ocrb.uint16Ptr = 0;
    }
}

/*!
*******************************************************************************
** \brief   Set up the compare output mode.
**
** \param   handlePtr   The handle must be checked by the caller.
**                      It also contains the output compare modes to set up.
**
** \return
**          - #TIMER_ERR_BAD_HANDLE if the handle is in a bad state.
**          - #TIMER_ERR_BAD_PARAMETER if an unsupported mode is to be set up.
**          - #TIMER_OK on success.
**
*******************************************************************************
*/
static int8_t timerSetCompareOutputMode (timerHandleT* handlePtr)
{
    switch (handlePtr->outputModeA)
    {
        case (TIMER_OutputMode_NormalPortOperation):
            break;
        case (TIMER_OutputMode_ToggleOnCompareMatch):
            *handlePtr->tccraPtr |= (1 << COM0A0);
            break;
        case (TIMER_OutputMode_ClearOnCompareMatch_NonInvertingPWM):
            *handlePtr->tccraPtr |= (1 << COM0A1);
            break;
        case (TIMER_OutputMode_SetOnCompareMatch_InvertingPWM):
            *handlePtr->tccraPtr |= (1 << COM0A1) | (1 << COM0A0);
            break;
        default:
            return (TIMER_ERR_BAD_HANDLE);
    }
    switch (handlePtr->outputModeB)
    {
        case (TIMER_OutputMode_NormalPortOperation):
            break;
        case (TIMER_OutputMode_ToggleOnCompareMatch):
            if ((handlePtr->waveGenerationMode == TIMER_WaveGeneration_NormalMode)
            ||  (handlePtr->waveGenerationMode == TIMER_WaveGeneration_ClearTimerOnCompareMatchA))
            {
                *handlePtr->tccraPtr |= (1 << COM0B0);
            }
            else
            {
                return (TIMER_ERR_BAD_PARAMETER);
            }
            break;
        case (TIMER_OutputMode_ClearOnCompareMatch_NonInvertingPWM):
            *handlePtr->tccraPtr |= (1 << COM0B1);
            break;
        case (TIMER_OutputMode_SetOnCompareMatch_InvertingPWM):
            *handlePtr->tccraPtr |= (1 << COM0B1) | (1 << COM0B0);
            break;
        default:
            return (TIMER_ERR_BAD_HANDLE);
    }
    return (TIMER_OK);
}

/*!
*******************************************************************************
** \brief   Set up the wave generation mode.
**
** \param   handlePtr           The handle must be checked by the caller.
**                              It also contains the wave generation mode
**                              to set up.
**
** \return
**          - #TIMER_ERR_BAD_PARAMETER if a bad parameter is provided.
**          - #TIMER_OK on success.
**
*******************************************************************************
*/
static int8_t timerSetWaveGenerationMode (timerHandleT* handlePtr)
{
    if ((handlePtr->timerId == TIMER_TimerId_0)
    ||  (handlePtr->timerId == TIMER_TimerId_2))
    {
        switch(handlePtr->waveGenerationMode)
        {
            case TIMER_WaveGeneration_NormalMode:
                // nothing to do
                break;
            case TIMER_WaveGeneration_ClearTimerOnCompareMatchA:
                *handlePtr->tccraPtr |= (1 << WGM01);
                break;
            case TIMER_WaveGeneration_FastPWM_8bit:
                *handlePtr->tccraPtr |= (1 << WGM01) | (1 << WGM00);
                break;
            case TIMER_WaveGeneration_PhaseCorrectPWM_8bit:
                *handlePtr->tccraPtr |= (1 << WGM00);
                break;
            case TIMER_WaveGeneration_FastPWM_9bit:
            case TIMER_WaveGeneration_FastPWM_10bit:
            case TIMER_WaveGeneration_PhaseCorrectPWM_9bit:
            case TIMER_WaveGeneration_PhaseCorrectPWM_10bit:
            default:
                return (TIMER_ERR_BAD_PARAMETER);
        }
    }
    else // (handlePtr->timerId == TIMER_TimerId_1)
    {
        switch(handlePtr->waveGenerationMode)
        {
            case TIMER_WaveGeneration_NormalMode:
                // nothing to do
                break;
            case TIMER_WaveGeneration_ClearTimerOnCompareMatchA:
                // ATmega644p.pdf table 13-5: Mode 4:
                *handlePtr->tccrbPtr |= (1 << WGM12);
                break;
            case TIMER_WaveGeneration_FastPWM_8bit:
                *handlePtr->tccraPtr |= (1 << WGM10);
                *handlePtr->tccrbPtr |= (1 << WGM12);
                break;
            case TIMER_WaveGeneration_FastPWM_9bit:
                *handlePtr->tccraPtr |= (1 << WGM11);
                *handlePtr->tccrbPtr |= (1 << WGM12);
                break;
            case TIMER_WaveGeneration_FastPWM_10bit:
                *handlePtr->tccraPtr |= (1 << WGM11) | (1 << WGM10);
                *handlePtr->tccrbPtr |= (1 << WGM12);
                break;
            case TIMER_WaveGeneration_PhaseCorrectPWM_8bit:
                *handlePtr->tccraPtr |= (1 << WGM10);
                break;
            case TIMER_WaveGeneration_PhaseCorrectPWM_9bit:
                *handlePtr->tccraPtr |= (1 << WGM11);
                break;
            case TIMER_WaveGeneration_PhaseCorrectPWM_10bit:
                *handlePtr->tccraPtr |= (1 << WGM11) | (1 << WGM10);
                break;
            default:
                return (TIMER_ERR_BAD_PARAMETER);
        }
    }
    return (TIMER_OK);
}

/*!
*******************************************************************************
** \brief   Check if a clock prescaler can be set up with a specific timer.
**
** \param   handlePtr       The handle must be checked by the caller.
**                          It is used to identify the timer.
** \param   clockPrescaler  The clock prescaler to check.
**
** \return
**          - #TIMER_ERR_BAD_PARAMETER if the clock prescaler is not supported
**              by the timer.
**          - #TIMER_OK if the clock prescaler is supported by the timer.
**
*******************************************************************************
*/
static int8_t timerCheckClockPrescaler (timerHandleT* handlePtr,
                                        TIMER_ClockPrescalerT clockPrescaler)
{
    if ((handlePtr->timerId == TIMER_TimerId_0)
    ||  (handlePtr->timerId == TIMER_TimerId_1))
    {
        switch(clockPrescaler)
        {
            case TIMER_ClockPrescaler_1:
            case TIMER_ClockPrescaler_8:
            case TIMER_ClockPrescaler_64:
            case TIMER_ClockPrescaler_256:
            case TIMER_ClockPrescaler_1024:
                break;
            case TIMER_ClockPrescaler_32:
            case TIMER_ClockPrescaler_128:
            default:
                return (TIMER_ERR_BAD_PARAMETER); // not supported for TC0 and TC1
        }
    }
    else // (timerId == TIMER_TimerId_2)
    {
        switch(handlePtr->clockPrescaler)
        {
            case TIMER_ClockPrescaler_1:
            case TIMER_ClockPrescaler_8:
            case TIMER_ClockPrescaler_32:
            case TIMER_ClockPrescaler_64:
            case TIMER_ClockPrescaler_128:
            case TIMER_ClockPrescaler_256:
            case TIMER_ClockPrescaler_1024:
                break;
            default:
                return (TIMER_ERR_BAD_PARAMETER);
        }
    }
    return (TIMER_OK);
}

/*!
*******************************************************************************
** \brief   Set the clock prescaler registers. This causes the timer to run.
**
**          The timer must be stopped by resetting the clock select bits to
**          zero (e.g., via timerStopClock()) before calling this function.
**
** \param   handlePtr       The handle must be checked by the caller.
**                          It also contains the clock prescaler to set up.
**
** \return
**          - #TIMER_ERR_BAD_HANDLE if the handle is in a bad state.
**          - #TIMER_OK on success.
**
*******************************************************************************
*/
static int8_t timerStartClock (timerHandleT* handlePtr)
{
    if ((handlePtr->timerId == TIMER_TimerId_0)
    ||  (handlePtr->timerId == TIMER_TimerId_1))
    {
        switch(handlePtr->clockPrescaler)
        {
            case TIMER_ClockPrescaler_1:
                *handlePtr->tccrbPtr |= (1 << CS00);
                break;
            case TIMER_ClockPrescaler_8:
                *handlePtr->tccrbPtr |= (1 << CS01);
                break;
            case TIMER_ClockPrescaler_64:
                *handlePtr->tccrbPtr |= (1 << CS01) | (1 << CS00);
                break;
            case TIMER_ClockPrescaler_256:
                *handlePtr->tccrbPtr |= (1 << CS02);
                break;
            case TIMER_ClockPrescaler_1024:
                *handlePtr->tccrbPtr |= (1 << CS02) | (1 << CS00);
                break;
            case TIMER_ClockPrescaler_32:   // not supported for TC0 and TC1
            case TIMER_ClockPrescaler_128:  // not supported for TC0 and TC1
            default:
                return (TIMER_ERR_BAD_HANDLE);
        }
    }
    else // (timerId == TIMER_TimerId_2)
    {
        switch(handlePtr->clockPrescaler)
        {
            case TIMER_ClockPrescaler_1:
                *handlePtr->tccrbPtr |= (1 << CS20);
                break;
            case TIMER_ClockPrescaler_8:
                *handlePtr->tccrbPtr |= (1 << CS21);
                break;
            case TIMER_ClockPrescaler_32:
                *handlePtr->tccrbPtr |= (1 << CS21) | (1 << CS20);
                break;
            case TIMER_ClockPrescaler_64:
                *handlePtr->tccrbPtr |= (1 << CS22);
                break;
            case TIMER_ClockPrescaler_128:
                *handlePtr->tccrbPtr |= (1 << CS22) | (1 << CS20);
                break;
            case TIMER_ClockPrescaler_256:
                *handlePtr->tccrbPtr |= (1 << CS22) | (1 << CS21);
                break;
            case TIMER_ClockPrescaler_1024:
                *handlePtr->tccrbPtr |= (1 << CS22) | (1 << CS21) | (1 << CS20);
                break;
            default:
                return (TIMER_ERR_BAD_HANDLE);
        }
    }
    return (TIMER_OK);
}

/*!
*******************************************************************************
** \brief   Set the data direction of the output compare pins to output.
**
** \param   handlePtr   The handle must be checked by the caller.
**
** \return
**          - #TIMER_ERR_BAD_HANDLE if the handle is in a bad state.
**          - #TIMER_OK on success.
**
*******************************************************************************
*/
static int8_t timerSetPinsAsOutputs (timerHandleT* handlePtr)
{
    switch(handlePtr->timerId)
    {
        case TIMER_TimerId_0:
            if (handlePtr->outputModeA != TIMER_OutputMode_NormalPortOperation)
            {
                SET_OUTPUT(PIN_OC0A);
            }
            if (handlePtr->outputModeB != TIMER_OutputMode_NormalPortOperation)
            {
                SET_OUTPUT(PIN_OC0B);
            }
            break;
        case TIMER_TimerId_1:
            if (handlePtr->outputModeA != TIMER_OutputMode_NormalPortOperation)
            {
                SET_OUTPUT(PIN_OC1A);
            }
            if (handlePtr->outputModeB != TIMER_OutputMode_NormalPortOperation)
            {
                SET_OUTPUT(PIN_OC1B);
            }
            break;
        case TIMER_TimerId_2:
            if (handlePtr->outputModeA != TIMER_OutputMode_NormalPortOperation)
            {
                SET_OUTPUT(PIN_OC2A);
            }
            if (handlePtr->outputModeB != TIMER_OutputMode_NormalPortOperation)
            {
                SET_OUTPUT(PIN_OC2B);
            }
            break;
        default:
            return (TIMER_ERR_BAD_HANDLE);
    }
    return (TIMER_OK);
}

/*!
*******************************************************************************
** \brief   Set the data direction of the output compare pins to input.
**
** \param   handlePtr   The handle must be checked by the caller.
**
** \return
**          - #TIMER_ERR_BAD_HANDLE if the handle is in a bad state.
**          - #TIMER_OK on success.
**
*******************************************************************************
*/
static int8_t timerSetPinsAsInputs (timerHandleT* handlePtr)
{
    switch(handlePtr->timerId)
    {
        case TIMER_TimerId_0:
            if (handlePtr->outputModeA != TIMER_OutputMode_NormalPortOperation)
            {
                SET_INPUT(PIN_OC0A);
            }
            if (handlePtr->outputModeB != TIMER_OutputMode_NormalPortOperation)
            {
                SET_INPUT(PIN_OC0B);
            }
            break;
        case TIMER_TimerId_1:
            if (handlePtr->outputModeA != TIMER_OutputMode_NormalPortOperation)
            {
                SET_INPUT(PIN_OC1A);
            }
            if (handlePtr->outputModeB != TIMER_OutputMode_NormalPortOperation)
            {
                SET_INPUT(PIN_OC1B);
            }
            break;
        case TIMER_TimerId_2:
            if (handlePtr->outputModeA != TIMER_OutputMode_NormalPortOperation)
            {
                SET_INPUT(PIN_OC2A);
            }
            if (handlePtr->outputModeB != TIMER_OutputMode_NormalPortOperation)
            {
                SET_INPUT(PIN_OC2B);
            }
            break;
        default:
            return (TIMER_ERR_BAD_HANDLE);
    }
    return (TIMER_OK);
}

/*!
*******************************************************************************
** \brief   Stops the timer by setting the clock select bits to zero.
**
** \param   handlePtr       The handle must be checked by the caller.
**
*******************************************************************************
*/
static inline void timerStopClock (timerHandleT* handlePtr)
{
    *handlePtr->tccrbPtr &= ~( (1 << CS02) | (1 << CS01) | (1 << CS00) );
    return;
}

/*!
*******************************************************************************
** \brief   Reset the timer register.
**
** \param   handlePtr       The handle must be checked by the caller.
**
*******************************************************************************
*/
static inline void timerResetTimerRegister (timerHandleT* handlePtr)
{
    if (handlePtr->bitWidth == timerBitWidth_8)
    {
        *handlePtr->tcnt.uint8Ptr = 0;
    }
    else
    {
        *handlePtr->tcnt.uint16Ptr = 0;
    }
    return;
}

/*!
*******************************************************************************
** \brief   Generic interrupt callback that is executed when a timer overflows.
**
** \param   handlePtr   The handle associated with the timer that triggered
**                      the interrupt.
**
*******************************************************************************
*/
static void timerOverflowHandler (timerHandleT* handlePtr)
{
    if (handlePtr->timerState == timerStateOneShot)
    {
        // Stop the timer:
        timerStopClock(handlePtr);
        // Reset the timer register in case it already incremented:
        timerResetTimerRegister(handlePtr);
        // Reset timer state:
        handlePtr->timerState = timerStateStopped;
    }

    // Overflow callback:
    if (handlePtr->overflowCallbackPtr)
    {
        handlePtr->overflowCallbackCounter++;
        if(handlePtr->overflowCallbackCounter >= handlePtr->overflowCallbackPeriod)
        {
            handlePtr->overflowCallbackCounter = 0;
            handlePtr->overflowCallbackPtr(handlePtr->overflowCallbackArgPtr);
        }
    }
    else
    {
        // Disable timer overflow interrupt:
        *handlePtr->timskPtr &= ~(1 << TOIE0);
    }
    return;
}


//*****************************************************************************
//*************************** PUBLIC FUNCTIONS ********************************
//*****************************************************************************

/*!
*******************************************************************************
** \brief   Initializes a timer.
**
** \param   timerId             Identifies the timer to be set up.
** \param   clockPrescaler      Specifies the prescaler of the clock.
**                              A higher prescaler makes the timer run slower.
** \param   waveGenerationMode  Specifies the wave generation mode for pulse
**                              width modulation operation.
** \param   outputModeA         Specifies whether and how the output compare
**                              unit A takes over the OCnA pin's port function.
** \param   outputModeB         Specifies whether and how the output compare
**                              unit B takes over the OCnB pin's port function.
**
** \return
**          - A valid TIMER_HandleT handle on success.
**          - NULL if a bad parameter has been passed.
**
*******************************************************************************
*/
TIMER_HandleT TIMER_Init (TIMER_TimerIdT          timerId,
                          TIMER_ClockPrescalerT   clockPrescaler,
                          TIMER_WaveGenerationT   waveGenerationMode,
                          TIMER_OutputModeT       outputModeA,
                          TIMER_OutputModeT       outputModeB)
{
    int8_t result = 0;
    timerHandleT* handlePtr = NULL;
    jmp_buf env;

    if (timerId < TIMER_NUMBER_OF_TIMERS)
    {
        handlePtr = &timerHandleArr[timerId];
    }
    else
    {
        return (NULL);
    }
    // Reset handle:
    memset(handlePtr, 0, sizeof(timerHandleT));

    // Populate handle:
    handlePtr->timerId = timerId;
    switch(timerId)
    {
        case TIMER_TimerId_0:
        case TIMER_TimerId_2:
            handlePtr->bitWidth = timerBitWidth_8;
            break;
        case TIMER_TimerId_1:
            handlePtr->bitWidth = timerBitWidth_16;
            break;
        default:
            return (NULL);
    }
    handlePtr->clockPrescaler = clockPrescaler;
    handlePtr->waveGenerationMode = waveGenerationMode;
    handlePtr->outputModeA = outputModeA;
    handlePtr->outputModeB = outputModeB;
    handlePtr->timerState = timerStateStopped;

    // Initialize register addresses:
    switch(timerId)
    {
        case TIMER_TimerId_0:
            handlePtr->tccraPtr       = &TCCR0A;
            handlePtr->tccrbPtr       = &TCCR0B;
            handlePtr->timskPtr       = &TIMSK0;
            handlePtr->tifrPtr        = &TIFR0;
            handlePtr->tcnt.uint8Ptr  = &TCNT0;
            handlePtr->ocra.uint8Ptr  = &OCR0A;
            handlePtr->ocrb.uint8Ptr  = &OCR0B;
            break;
        case TIMER_TimerId_1:
            handlePtr->tccraPtr       = &TCCR1A;
            handlePtr->tccrbPtr       = &TCCR1B;
            handlePtr->timskPtr       = &TIMSK1;
            handlePtr->tifrPtr        = &TIFR1;
            handlePtr->tcnt.uint16Ptr = &TCNT1;
            handlePtr->ocra.uint16Ptr = &OCR1A;
            handlePtr->ocrb.uint16Ptr = &OCR1B;
            break;
        case TIMER_TimerId_2:
            handlePtr->tccraPtr       = &TCCR2A;
            handlePtr->tccrbPtr       = &TCCR2B;
            handlePtr->timskPtr       = &TIMSK2;
            handlePtr->tifrPtr        = &TIFR2;
            handlePtr->tcnt.uint8Ptr  = &TCNT2;
            handlePtr->ocra.uint8Ptr  = &OCR2A;
            handlePtr->ocrb.uint8Ptr  = &OCR2B;
            break;
        default:
            return (NULL);
    }

    // Reset all registers:
    timerResetRegisters (handlePtr);

    // Reset registers if any errors occur:
    result = setjmp(env);
    if (result)
    {
        timerResetRegisters (handlePtr);
        memset (handlePtr, 0, sizeof(timerHandleT));
        return (NULL);
    }

    // Set up compare output mode:
    result = timerSetCompareOutputMode (handlePtr);
    if (result) longjmp (env, result);

    // Set up wave generation mode:
    result = timerSetWaveGenerationMode (handlePtr);
    if (result) longjmp (env, result);

    // Check if the clock prescaler is supported by the timer:
    result = timerCheckClockPrescaler (handlePtr, clockPrescaler);
    if (result) longjmp (env, result);

    // Set PWM pins as outputs:
    // Set up DDR registers after compare output mode, according to ATmega644p 12.5.3
    result = timerSetPinsAsOutputs (handlePtr);
    if (result) longjmp (env, result);

    // Mark handle as initialized:
    handlePtr->initialized = 1;

    return (TIMER_HandleT)handlePtr;
}

/*!
*******************************************************************************
** \brief   Checks whether a timer handle is initialized.
**
** \param   handle  The TIMER handle to be checked.
**
** \return
**          - 1 if the timer is initialized.
**          - 0 if the timer is not initialized or if the handle is invalid.
**
*******************************************************************************
*/
int8_t TIMER_IsInitialized (TIMER_HandleT handle)
{
    uint8_t initialized = 0;

#if TIMER_INTERRUPT_SAFETY
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
#endif
    {
        timerHandleT* handlePtr = (timerHandleT*)handle;
        if (handlePtr == NULL)
        {
            return 0;
        }
        initialized = handlePtr->initialized;
    }
    return initialized;
}

/*!
*******************************************************************************
** \brief   Shut down timer driver.
**
**          This function sets all PWM pins as inputs, resets all hardware
**          registers associated with the handle to zero and resets the handle.
**
** \param   handle  The timer handle of the timer to shut down.
**
** \return
**          - #TIMER_OK on success.
**          - #TIMER_ERR_BAD_HANDLE if the handle is invalid.
**
*******************************************************************************
*/
int8_t TIMER_Exit (TIMER_HandleT handle)
{
#if TIMER_INTERRUPT_SAFETY
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
#endif
    {
        timerHandleT* handlePtr = (timerHandleT*)handle;
        if ((handlePtr == NULL) || (!handlePtr->initialized))
        {
            return (TIMER_ERR_BAD_HANDLE);
        }

        ////////////////////////////////////////////////
        // Enter critical section:
        *handlePtr->timskPtr &= ~(1 << TOIE0);

        // Set PWM pins as inputs:
        (void) timerSetPinsAsInputs(handlePtr);

        // Reset timer registers:
        timerResetRegisters(handlePtr);

        // Reset handle:
        memset(handlePtr, 0, sizeof(timerHandleT));
    }
    return (TIMER_OK);
}

/*!
*******************************************************************************
** \brief   Start the timer.
**
**          This function enables continuous timer operation.
**
** \param   handle  The timer handle of the timer to start.
**
** \return
**          - #TIMER_OK on success.
**          - #TIMER_ERR_BAD_HANDLE if the handle is invalid.
**
*******************************************************************************
*/
int8_t TIMER_Start (TIMER_HandleT handle)
{
    int8_t result = 0;

#if TIMER_INTERRUPT_SAFETY
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
#endif
    {
        timerHandleT* handlePtr = (timerHandleT*)handle;
        if ((handlePtr == NULL) || (!handlePtr->initialized))
        {
            return (TIMER_ERR_BAD_HANDLE);
        }

        ////////////////////////////////////////////////
        // Enter critical section:
        *handlePtr->timskPtr &= ~(1 << TOIE0);

        // Mark the timer to run continuously:
        handlePtr->timerState = timerStateRunning;

        // Start the timer by setting the clock select bits:
        result = timerStartClock(handlePtr);

        // Enable interrupt if a callback is registerd:
        if (handlePtr->overflowCallbackPtr)
        {
            *handlePtr->timskPtr |= (1 << TOIE0);
        }
        ////////////////////////////////////////////////
    }
    return result;
}

/*!
*******************************************************************************
** \brief   Start the timer for one iteration, then stop again.
**          The timer will be reset to 0 after completion.
**
**          If executed while the timer is already running, this function
**          will have the same effect as
**          TIMER_Stop(handle, TIMER_Stop_OnOverflow).
**
** \param   handle  The timer handle of the timer to start.
**
** \return
**          - #TIMER_OK on success.
**          - #TIMER_ERR_BAD_HANDLE if the handle is invalid.
**
*******************************************************************************
*/
int8_t TIMER_OneShot (TIMER_HandleT handle)
{
    int8_t result = 0;

#if TIMER_INTERRUPT_SAFETY
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
#endif
    {
        timerHandleT* handlePtr = (timerHandleT*)handle;
        if ((handlePtr == NULL) || (!handlePtr->initialized))
        {
            return (TIMER_ERR_BAD_HANDLE);
        }

        ////////////////////////////////////////////////
        // Enter critical section:
        *handlePtr->timskPtr &= ~(1 << TOIE0);

        // Make the timer stop in the ISR after current run:
        handlePtr->timerState = timerStateOneShot;

        // If a callback is set, it has already cleared the overflow flag
        // during the last iteration in the ISR. Otherwise, clear the overflow
        // flag here so that the interrupt won't be triggered immediately but
        // AFTER the current iteration:
        if (handlePtr->overflowCallbackPtr == NULL)
        {
            *handlePtr->tifrPtr = (1 << TOV0);
        }

        // Start the timer by setting the clock select bits:
        result = timerStartClock(handlePtr);

        // Leave critical section by enabling timer overflow interrupt:
        *handlePtr->timskPtr |= (1 << TOIE0);
        ////////////////////////////////////////////////
    }
    return result;
}

/*!
*******************************************************************************
** \brief   Stop the timer.
**
**          There are different ways to stop the timer, specified by the
**          stopMode parameter. The timer can be restarted by TIMER_Start().
**
** \param   handle      The timer handle of the timer to stop.
** \param   stopMode    Specifies how to stop the timer.
**                      If it is TIMER_Stop_OnOverflow, the timer will be
**                      stopped when the next overflow occurs. The timer
**                      will also be reset to zero. This is the recommended
**                      option for wave generating modes.
**                      If it is TIMER_Stop_Immediately, the timer will be
**                      stopped immediately. The timer register will not be
**                      reset.
**                      If it is TIMER_Stop_ImmediatelyAndReset, the timer
**                      will be stopped immediately and the timer will be
**                      reset to zero.
**
** \return
**          - #TIMER_OK on success.
**          - #TIMER_ERR_BAD_HANDLE if the handle is invalid.
**
*******************************************************************************
*/
int8_t TIMER_Stop (TIMER_HandleT handle, TIMER_StopT stopMode)
{
#if TIMER_INTERRUPT_SAFETY
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
#endif
    {
        timerHandleT* handlePtr = (timerHandleT*)handle;
        if ((handlePtr == NULL) || (!handlePtr->initialized))
        {
            return (TIMER_ERR_BAD_HANDLE);
        }

        ////////////////////////////////////////////////
        // Enter critical section:
        *handlePtr->timskPtr &= ~(1 << TOIE0);

        if (stopMode == TIMER_Stop_OnOverflow)
        {
            if (handlePtr->timerState != timerStateStopped)
            {
                // Make the timer stop in the ISR after current run:
                handlePtr->timerState = timerStateOneShot;

                // If a callback is set, it has already cleared the overflow flag
                // during the last iteration in the ISR. Otherwise, clear the overflow
                // flag here so that the interrupt won't be triggered immediately but
                // AFTER the current iteration:
                if (handlePtr->overflowCallbackPtr == NULL)
                {
                    *handlePtr->tifrPtr = (1 << TOV0);
                }

                // Leave critical section by enabling timer overflow interrupt:
                *handlePtr->timskPtr |= (1 << TOIE0);
                ////////////////////////////////////////////
            }
        }
        else // TIMER_Stop_Immediately[AndReset]
        {
            // Stop the timer immediately:
            timerStopClock(handlePtr);
            if (stopMode == TIMER_Stop_ImmediatelyAndReset)
            {
                timerResetTimerRegister(handlePtr);
            }
            handlePtr->timerState = timerStateStopped;
        }
    }
    return (TIMER_OK);
}

/*!
*******************************************************************************
** \brief   Register a callback function for the timer overflow interrupt.
**
** \param   handle          A valid timer handle.
** \param   callbackPtr     The callback to register with the handle.
**                          A previously registered callback can be unregistered
**                          by passing NULL.
** \param   optArgPtr       An optional argument pointer that will be passed to
**                          the callback upon execution.
** \param   callbackPeriod  Number of timer overflows until callback is executed.
**                          If 0 or 1, the callback will be executed on each
**                          timer overflow.
**
** \return
**          - #TIMER_OK on success.
**          - #TIMER_ERR_BAD_HANDLE if the handle is invalid.
**
*******************************************************************************
*/
int8_t TIMER_SetOverflowCallback (TIMER_HandleT handle,
                                  TIMER_CallbackT callbackPtr,
                                  void* optArgPtr,
                                  uint16_t callbackPeriod)
{
#if TIMER_INTERRUPT_SAFETY
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
#endif
    {
        timerHandleT* handlePtr = (timerHandleT*)handle;
        if ((handlePtr == NULL) || (!handlePtr->initialized))
        {
            return (TIMER_ERR_BAD_HANDLE);
        }

        ////////////////////////////////////////////////
        // Enter critical section:
        *handlePtr->timskPtr &= ~(1 << TOIE0);

        // Clear the timer overflow flag if there was previously no callback,
        // which would have cleared the flag automatically in ISR:
        if ((handlePtr->overflowCallbackPtr == NULL)
        &&  (handlePtr->timerState != timerStateOneShot))
        {
            *handlePtr->tifrPtr = (1 << TOV0);
        }

        handlePtr->overflowCallbackPtr = callbackPtr;
        handlePtr->overflowCallbackArgPtr = optArgPtr;
        handlePtr->overflowCallbackPeriod = callbackPeriod;

        // Enable overflow interrupt:
        if((handlePtr->overflowCallbackPtr)
        || (handlePtr->timerState == timerStateOneShot))
        {
            *handlePtr->timskPtr |= (1 << TOIE0);
        }
        ////////////////////////////////////////////////
    }
    return (TIMER_OK);
}

/*!
*******************************************************************************
** \brief   Set the output compare registers. In PWM modes, this function
**          effectively sets the pulse width of the PWM signals.
**
**          Note that the output compare registers are double buffered in
**          PWM modes (see e.g. ATmega644p.pdf 12.5).
**
** \param   handle              A valid timer handle.
** \param   outputCompareA      Value for output compare unit A.
**                              For timer 0 and timer 2, the value must be
**                              8-bit, i.e., less or equal to 255. For timer
**                              1, the value may be 8-bit, 9-bit or 10-bit,
**                              depending on the wave generation mode.
** \param   outputCompareB      Value for output compare unit A.
**                              Otherwise it has the same constraints as
**                              outputCompareA.
**
** \return
**          - #TIMER_OK on success.
**          - #TIMER_ERR_BAD_HANDLE if the handle is invalid.
**
*******************************************************************************
*/
int8_t TIMER_SetOutputCompareRegisters (TIMER_HandleT handle,
                                        uint16_t outputCompareA,
                                        uint16_t outputCompareB)
{
#if TIMER_INTERRUPT_SAFETY
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
#endif
    {
        timerHandleT* handlePtr = (timerHandleT*)handle;
        if ((handlePtr == NULL) || (!handlePtr->initialized))
        {
            return (TIMER_ERR_BAD_HANDLE);
        }

        if (handlePtr->bitWidth == timerBitWidth_8)
        {
            *handlePtr->ocra.uint8Ptr = (uint8_t)outputCompareA;
            *handlePtr->ocrb.uint8Ptr = (uint8_t)outputCompareB;
        }
        else // if (handlePtr->bitWidth == timerBitWidth_16)
        {
            // 16-bit register access: see ATmega644p.pdf 13.3
            *handlePtr->ocra.uint16Ptr = outputCompareA;
            *handlePtr->ocrb.uint16Ptr = outputCompareB;
        }
    }
    return (TIMER_OK);
}

/*!
*******************************************************************************
** \brief   Set the clock prescaler.
**
**          If the timer is running, the clock prescaler will be updated
**          during operation. Otherwise, it is stored in the handle and will
**          take effect when starting the timer.
**
** \param   handle          A valid timer handle.
** \param   clockPrescaler  The new prescaler to set up.
**
** \return
**          - #TIMER_OK on success.
**          - #TIMER_ERR_BAD_HANDLE if the handle is invalid.
**          - #TIMER_ERR_BAD_PARAMETER if the clock prescaler is not supported
**              by the respective timer.
**
*******************************************************************************
*/
int8_t TIMER_SetClockPrescaler (TIMER_HandleT handle,
                                TIMER_ClockPrescalerT clockPrescaler)
{
    int8_t result = 0;

#if TIMER_INTERRUPT_SAFETY
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
#endif
    {
        timerHandleT* handlePtr = (timerHandleT*)handle;
        if ((handlePtr == NULL) || (!handlePtr->initialized))
        {
            return (TIMER_ERR_BAD_HANDLE);
        }

        // Check new prescaler:
        result = timerCheckClockPrescaler(handlePtr, clockPrescaler);
        if (result) return result;

        ////////////////////////////////////////////////
        // Enter critical section:
        *handlePtr->timskPtr &= ~(1 << TOIE0);

        handlePtr->clockPrescaler = clockPrescaler;

        if (handlePtr->timerState != timerStateStopped)
        {
            // Run timer with new clock prescaler:
            timerStopClock(handlePtr);
            result = timerStartClock(handlePtr);

            // Enable overflow interrupt:
            if((handlePtr->overflowCallbackPtr)
            || (handlePtr->timerState == timerStateOneShot))
            {
                *handlePtr->timskPtr |= (1 << TOIE0);
            }
            ////////////////////////////////////////////////
        }
    }
    return result;
}


//*****************************************************************************
//*********************** INTERRUPT SERVICE ROUTINES **************************
//*****************************************************************************

/*!
*******************************************************************************
** \brief   ISR for timer 0 overflow.
**
**          The ISR increments the corresponding handle's counter and
**          periodically invokes a callback function.
**
*******************************************************************************
*/
ISR (TIMER0_OVF_vect, ISR_BLOCK)
{
    timerOverflowHandler(&timerHandleArr[0]);
    return;
}

/*!
*******************************************************************************
** \brief   ISR for timer 1 overflow.
**
**          The ISR increments the corresponding handle's counter and
**          periodically invokes a callback function.
**
*******************************************************************************
*/
ISR (TIMER1_OVF_vect, ISR_BLOCK)
{
    timerOverflowHandler(&timerHandleArr[1]);
    return;
}

/*!
*******************************************************************************
** \brief   ISR for timer 2 overflow.
**
**          The ISR increments the corresponding handle's counter and
**          periodically invokes a callback function.
**
*******************************************************************************
*/
ISR (TIMER2_OVF_vect, ISR_BLOCK)
{
    timerOverflowHandler(&timerHandleArr[2]);
    return;
}

