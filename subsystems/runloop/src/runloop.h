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
#define F_CPU                               18432000
#endif

//! Maximum number of tasks that can be scheduled at the same time. Should be < 256.
#ifndef RUNLOOP_MAX_NUMBER_OF_TASKS
#define RUNLOOP_MAX_NUMBER_OF_TASKS         10
#endif

//! Set to 1 to enable the interactive commandline interface in the runloop.
#ifndef RUNLOOP_WITH_CMDL
#define RUNLOOP_WITH_CMDL                   1
#endif

//! Set to 1 to enable the uptime feature of the runloop.
#ifndef RUNLOOP_WITH_UPTIME
#define RUNLOOP_WITH_UPTIME                 1
#endif

/*! If RUNLOOP_WITH_UPTIME is enabled, this defines the minimum
**  uptime update interval in milliseconds. This setting is only
**  needed if the uptime is read from interrupt service routines
**  since the uptime is always updated before task execution.
**  If set to 0, the uptime will be updated only before task
**  execution in the runloop, which is the recommended setting in
**  case that the uptime is only read by runloop tasks. The value
**  should not be specified greater than
**  ((2^32 - 1) / (F_CPU / 1000)) - 1. */
#ifndef RUNLOOP_UPTIME_UPDATE_INTERVAL_MS
#define RUNLOOP_UPTIME_UPDATE_INTERVAL_MS   0
#endif

/*! Set to 1 if RUNLOOP_AddTask(), RUNLOOP_GetUptimeClockCycles(),
**  or RUNLOOP_GetUptimeHumanReadable() will be called from within ISRs. */
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

#if RUNLOOP_WITH_UPTIME
int8_t RUNLOOP_GetUptimeClockCycles (uint64_t* systemClockCyclesPtr);

int8_t RUNLOOP_GetUptimeHumanReadable (uint16_t* daysPtr,
                                       uint8_t*  hoursPtr,
                                       uint8_t*  minutesPtr,
                                       uint8_t*  secondsPtr,
                                       uint16_t* millisecondsPtr);
#endif // RUNLOOP_WITH_UPTIME

#endif // RUNLOOP_H

