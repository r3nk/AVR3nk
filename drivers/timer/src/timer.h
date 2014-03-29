/*!
*******************************************************************************
*******************************************************************************
** \brief   Timer driver with support for PWM (Pulse Width Modulation)
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

#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>


//*****************************************************************************
//*************************** DEFINES AND MACROS ******************************
//*****************************************************************************

//! CPU clock frequency
#ifndef F_CPU
#define F_CPU                       18432000
#endif

//! Set to 1 if TIMER functions will be called from within ISRs.
#ifndef TIMER_INTERRUPT_SAFETY
#define TIMER_INTERRUPT_SAFETY      0
#endif

//! Switch to enable the countdown feature (requires additional 60 bytes in RAM).
#ifndef TIMER_WITH_COUNTDOWN
#define TIMER_WITH_COUNTDOWN        0
#endif

//! A smaller value increases countdown precision at the cost of generated interrupts.
#ifndef TIMER_COUNTDOWN_IMPRECISION
#define TIMER_COUNTDOWN_IMPRECISION 256
#endif


//*****************************************************************************
//************************* TIMER SPECIFIC ERROR CODES ************************
//*****************************************************************************

/*! TIMER specific error base */
#ifndef TIMER_ERR_BASE
#define TIMER_ERR_BASE                      50
#endif

/*! TIMER returns with no errors. */
#define TIMER_OK                            0

/*! A bad parameter has been passed. */
#define TIMER_ERR_BAD_PARAMETER             TIMER_ERR_BASE + 0

/*! A bad handle has been passed. */
#define TIMER_ERR_BAD_HANDLE                TIMER_ERR_BASE + 1

/*! The set up wave generation mode is not suited for the operation. */
#define TIMER_ERR_INCOMPATIBLE_WGM          TIMER_ERR_BASE + 2

/*! The stopwatch is disabled. */
#define TIMER_ERR_STOPWATCH_DISABLED        TIMER_ERR_BASE + 3

/*! There would be a timer resource conflict if the function was executed. */
#define TIMER_ERR_RESOURCE_CONFLICT         TIMER_ERR_BASE + 4


//*****************************************************************************
//******************************** DATA TYPES *********************************
//*****************************************************************************

/*! TIMER handle */
typedef void* TIMER_HandleT;

/*! Timer/counter identifier. */
typedef enum
{
    TIMER_TimerId_0 = 0,
    TIMER_TimerId_1,
    TIMER_TimerId_2
} TIMER_TimerIdT;

/*! The timer's clock is the system clock divided by the prescaler. */
typedef enum
{
    TIMER_ClockPrescaler_1 = 0,
    TIMER_ClockPrescaler_8,
    TIMER_ClockPrescaler_32,  //<! only for timer 2
    TIMER_ClockPrescaler_64,
    TIMER_ClockPrescaler_128, //<! only for timer 2
    TIMER_ClockPrescaler_256,
    TIMER_ClockPrescaler_1024
} TIMER_ClockPrescalerT;

/*! Wave generation mode. */
typedef enum
{
    TIMER_WaveGeneration_NormalMode = 0,
    TIMER_WaveGeneration_ClearTimerOnCompareMatchA,
    TIMER_WaveGeneration_FastPWM_8bit,
    TIMER_WaveGeneration_FastPWM_9bit,         //<! only for timer 1
    TIMER_WaveGeneration_FastPWM_10bit,        //<! only for timer 1
    TIMER_WaveGeneration_PhaseCorrectPWM_8bit,
    TIMER_WaveGeneration_PhaseCorrectPWM_9bit, //<! only for timer 1
    TIMER_WaveGeneration_PhaseCorrectPWM_10bit //<! only for timer 1
} TIMER_WaveGenerationT;

/*! Mode of the output compare unit (important in PWM modes). */
typedef enum
{
    TIMER_OutputMode_NormalPortOperation = 0,
    TIMER_OutputMode_ToggleOnCompareMatch,
    TIMER_OutputMode_ClearOnCompareMatch_NonInvertingPWM,
    TIMER_OutputMode_SetOnCompareMatch_InvertingPWM
} TIMER_OutputModeT;

/*! These modes define how the timer stops counting. */
typedef enum
{
    TIMER_Stop_OnOverflow = 0,
    TIMER_Stop_Immediately,
    TIMER_Stop_ImmediatelyAndReset
}
TIMER_StopT;

/*! TIMER callback function. */
typedef void (*TIMER_CallbackT) (void* optArgPtr);

/*! Defines whether the timer keeps track of elapsed system clock cycles. */
typedef enum
{
    TIMER_Stopwatch_Disable = 0,
    TIMER_Stopwatch_Enable
} TIMER_StopwatchEnableDisableT;

/*! Defines whether to reset the stopwatch. */
typedef enum
{
    TIMER_Stopwatch_NoReset = 0,
    TIMER_Stopwatch_Reset
} TIMER_StopwatchResetT;


//*****************************************************************************
//************************* FUNCTION DECLARATIONS *****************************
//*****************************************************************************

TIMER_HandleT TIMER_Init (TIMER_TimerIdT          timerId,
                          TIMER_ClockPrescalerT   clockPrescaler,
                          TIMER_WaveGenerationT   waveGenerationMode,
                          TIMER_OutputModeT       outputModeA,
                          TIMER_OutputModeT       outputModeB);

uint8_t TIMER_IsInitialized (TIMER_HandleT handle);

uint8_t TIMER_Exit (TIMER_HandleT handle);

uint8_t TIMER_Start (TIMER_HandleT handle);

uint8_t TIMER_OneShot (TIMER_HandleT handle);

uint8_t TIMER_Stop (TIMER_HandleT handle, TIMER_StopT stopMode);

uint8_t TIMER_SetOverflowCallback (TIMER_HandleT handle,
                                   TIMER_CallbackT callbackPtr,
                                   void* optArgPtr,
                                   uint16_t callbackPeriod);

uint8_t TIMER_SetOutputCompareRegisters (TIMER_HandleT handle,
                                         uint16_t outputCompareA,
                                         uint16_t outputCompareB);

uint8_t TIMER_SetClockPrescaler (TIMER_HandleT handle,
                                 TIMER_ClockPrescalerT clockPrescaler);

uint16_t TIMER_GetClockPrescalerValue (TIMER_ClockPrescalerT prescaler);

#if TIMER_WITH_COUNTDOWN
uint8_t TIMER_StartCountdown (TIMER_HandleT handle,
                              TIMER_CallbackT callbackPtr,
                              void* callbackArgPtr,
                              uint16_t timeMs,
                              uint16_t numberOfExecutions);
#endif // TIMER_WITH_COUNTDOWN

uint8_t TIMER_EnableDisableStopwatch (TIMER_HandleT handle,
                                      TIMER_StopwatchEnableDisableT enableDisable);

uint8_t TIMER_GetStopwatchSystemClockCycles (TIMER_HandleT handle,
                                             uint32_t* clockCycles,
                                             TIMER_StopwatchResetT stopwatchReset);

uint8_t TIMER_GetStopwatchTimeMs (TIMER_HandleT handle,
                                  uint32_t* timeMs,
                                  TIMER_StopwatchResetT stopwatchReset);

uint8_t TIMER_SetStopwatchTimeCallback(TIMER_HandleT handle,
                                       TIMER_CallbackT callbackPtr,
                                       void* callbackArgPtr,
                                       uint32_t clockCycles);

#endif

