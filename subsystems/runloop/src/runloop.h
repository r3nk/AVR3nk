/*!
*******************************************************************************
*******************************************************************************
** \brief   The runloop subsystem allows to create tasks and to schedule their
**          execution in a periodic manner.
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

#ifndef RUNLOOP_H
#define RUNLOOP_H

#include <stdint.h>
#include <drivers/timer.h>
#include <drivers/uart.h>

//*****************************************************************************
//*************************** DEFINES AND MACROS ******************************
//*****************************************************************************

//! CPU clock frequency
#ifndef F_CPU
#define F_CPU                       18432000
#endif

//! Maximum number of tasks that can be scheduled at the same time. Should be < 256.
#ifndef RUNLOOP_MAX_NUMBER_OF_TASKS
#define RUNLOOP_MAX_NUMBER_OF_TASKS         10
#endif

/*! Set to 1 if RUNLOOP_AddTask() will be called from within ISRs.
**  If set, TIMER_INTERRUPT_SAFETY must also be set to 1. */
#ifndef RUNLOOP_INTERRUPT_SAFETY
#define RUNLOOP_INTERRUPT_SAFETY            0
#endif

//! Debug switch, should be set to 0 for normal applications:
#ifndef RUNLOOP_DEBUG
#define RUNLOOP_DEBUG                       0
#endif

//*****************************************************************************
//************************* RUNLOOP SPECIFIC ERROR CODES **********************
//*****************************************************************************

/*! RUNLOOP specific error base */
#ifndef RUNLOOP_ERR_BASE
#define RUNLOOP_ERR_BASE                    110
#endif

/*! RUNLOOP returns with no errors. */
#define RUNLOOP_OK                          0

/*! Special return value for tasks that return without error but that should
**  be removed from the runloop. */
#define RUNLOOP_OK_TASK_ABORT               1

/*! A bad parameter has been passed. */
#define RUNLOOP_ERR_BAD_PARAMETER           RUNLOOP_ERR_BASE + 0

/*! The runloop is already initialized. */
#define RUNLOOP_ERR_ALREADY_INITIALIZED     RUNLOOP_ERR_BASE + 1

/*! The TIMER driver could not be initialized. */
#define RUNLOOP_ERR_TIMER_INITIALIZATION    RUNLOOP_ERR_BASE + 2

/*! There is no task slot free to register a new task. */
#define RUNLOOP_ERR_NO_TASK_SLOT_FREE       RUNLOOP_ERR_BASE + 3


//*****************************************************************************
//******************************** DATA TYPES *********************************
//*****************************************************************************

/*! Signature of the task error callback. */
typedef void (*RUNLOOP_TaskErrorCallbackT) (uint8_t taskId, uint8_t errorCode);

/*! Signature of the synchronization error callback. */
typedef void (*RUNLOOP_SyncErrorCallbackT) (uint8_t taskId, uint16_t dropCount);

/*! The callback of a task takes an optional pointer as an argument
** and returns an error code. If the task returns an error other than
** RUNLOOP_OK, it will be removed from the runloop. If the task returns
** an error code other than RUNLOOP_OK or RUNLOOP_OK_TASK_ABORT, and
** if a taskErrorCallback was registered during initialization, that
** callback will be executed after removing the task from the runloop. */
typedef uint8_t (*RUNLOOP_TaskCallbackT) (void* optArgPtr);


//*****************************************************************************
//************************* FUNCTION DECLARATIONS *****************************
//*****************************************************************************

int8_t RUNLOOP_Init (TIMER_TimerIdT timerId,
                     TIMER_ClockPrescalerT timerClockPrescaler,
                     UART_HandleT uartHandle,
                     RUNLOOP_TaskErrorCallbackT taskErrorCallback,
                     RUNLOOP_SyncErrorCallbackT syncErrorCallback);

int8_t RUNLOOP_AddTask (RUNLOOP_TaskCallbackT callbackPtr,
                        void* callbackArgPtr,
                        uint16_t numberOfExecutions,
                        uint32_t periodMs,
                        uint32_t initialDelayMs,
                        uint8_t* taskIdPtr);

void RUNLOOP_Run (void);

void RUNLOOP_Stop (void* optArgPtr);

#endif
