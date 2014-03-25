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
#ifndef TIMER_ENABLE_COUNTDOWN
#define TIMER_ENABLE_COUNTDOWN      1
#endif

//! A smaller value increases timer precision at the cost of generated interrupts.
#ifndef TIMER_COUNTDOWN_IMPRECISION
#define TIMER_COUNTDOWN_IMPRECISION 0
#endif


//*****************************************************************************
//************************* TIMER SPECIFIC ERROR CODES ************************
//*****************************************************************************

/*! TIMER specific error base */
#define TIMER_ERR_BASE                      0

/*! TIMER returns with no errors. */
#define TIMER_OK                            0

/*! A bad parameter has been passed. */
#define TIMER_ERR_BAD_PARAMETER             TIMER_ERR_BASE - 1

/*! A bad handle has been passed. */
#define TIMER_ERR_BAD_HANDLE                TIMER_ERR_BASE - 2

/*! The set up wave generation mode is not suited for the operation. */
#define TIMER_ERR_INCOMPATIBLE_WGM          TIMER_ERR_BASE - 3


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
    TIMER_Stopwatch_Off = 0,
    TIMER_Stopwatch_On
} TIMER_StopwatchEnableT;


//*****************************************************************************
//************************* FUNCTION DECLARATIONS *****************************
//*****************************************************************************

TIMER_HandleT TIMER_Init (TIMER_TimerIdT          timerId,
                          TIMER_ClockPrescalerT   clockPrescaler,
                          TIMER_WaveGenerationT   waveGenerationMode,
                          TIMER_OutputModeT       outputModeA,
                          TIMER_OutputModeT       outputModeB);

int8_t TIMER_IsInitialized (TIMER_HandleT handle);

int8_t TIMER_Exit (TIMER_HandleT handle);

int8_t TIMER_Start (TIMER_HandleT handle);

int8_t TIMER_OneShot (TIMER_HandleT handle);

int8_t TIMER_Stop (TIMER_HandleT handle, TIMER_StopT stopMode);

int8_t TIMER_SetOverflowCallback (TIMER_HandleT handle,
                                  TIMER_CallbackT callbackPtr,
                                  void* optArgPtr,
                                  uint16_t callbackPeriod);

int8_t TIMER_SetOutputCompareRegisters (TIMER_HandleT handle,
                                        uint16_t outputCompareA,
                                        uint16_t outputCompareB);

int8_t TIMER_SetClockPrescaler (TIMER_HandleT handle,
                                TIMER_ClockPrescalerT clockPrescaler);

#if TIMER_ENABLE_COUNTDOWN
int8_t TIMER_StartCountdown (TIMER_HandleT handle,
                             TIMER_CallbackT callbackPtr,
                             void* optArgPtr,
                             uint16_t timeMs,
                             uint16_t numberOfExecutions);
#endif // TIMER_ENABLE_COUNTDOWN

int8_t TIMER_ResetStopwatch (TIMER_HandleT handle,
                             TIMER_StopwatchEnableT stopwatchEnable);

int8_t TIMER_GetStopwatchSystemClockCycles (TIMER_HandleT handle,
                                            uint32_t* clockCycles);

int8_t TIMER_GetStopwatchTimeMs (TIMER_HandleT handle,
                                 uint32_t* timeMs);

#endif

