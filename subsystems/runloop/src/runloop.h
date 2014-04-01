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

//! Maximum number of tasks that can be scheduled at the same time.
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

/*! The callback of a task takes an optional pointer as an argument
** and returns an error code. */
typedef int8_t (*RUNLOOP_CallbackT) (void* optArgPtr);


//*****************************************************************************
//************************* FUNCTION DECLARATIONS *****************************
//*****************************************************************************

int8_t RUNLOOP_Init (TIMER_TimerIdT timerId,
                     UART_HandleT uartHandle);

int8_t RUNLOOP_AddTask (RUNLOOP_CallbackT callbackPtr,
                        void* callbackArgPtr,
                        uint16_t numberOfExecutions,
                        uint32_t periodMs,
                        uint32_t initialDelayMs);

void RUNLOOP_Run (void);

void RUNLOOP_Stop (void* optArgPtr);

#endif
