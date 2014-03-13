/*!
*******************************************************************************
*******************************************************************************
** \brief   The RUNLOOP subsystem allows to create tasks and to schedule their
**          execution in a periodic manner.
**
**          The runloop subsystem provides facilities to execute tasks in a
**          periodic and time-delayed manner. To initialize the runloop,
**          a UART handle and a TIMER id must be provided.
**          After initialization, tasks can be added to the runloop by
**          RUNLOOP_AddTask() and the runloop can be started by calling
**          RUNLOOP_Run(). When adding a task, the number of executions,
**          the time period and an initial delay can be specified. If the
**          runloop is already running and the initial delay is 0, the
**          task will be executed as soon as possible. However, the currently
**          running task and any other tasks in the runloop that are already
**          ready to be scheduled will be executed first.
**          When tasks should be added from interrupt service routines
**          concurrently to runloop execution, the macro RUNLOOP_INTERRUPT_SAFETY
**          must be set to 1. Otherwise, race conditions may occur. If new
**          tasks are added only by already running tasks in the runloop,
**          this macro can safely be disabled, which may improve the
**          responsiveness of interrupts a little bit.
**          The runloop execution can be temporarily superseded by an
**          interactive command line interface by sending a 'q' to the UART.
**          The runloop is halted and a prompt is shown. See the CMDL
**          subsystem for detailed information on how to register commands
**          with the commandline. When typing 'exit', runloop execution
**          will continue.
**          The runloop execution can also be interrupted by RUNLOOP_Stop().
**          This command halts the runloop and effectively causes RUNLOOP_Run()
**          to return.
**          The maximum number of tasks that can be concurrently scheduled
**          in the runloop is specified by the macro
**          RUNLOOP_MAX_NUMBER_OF_TASKS.
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
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <util/atomic.h>
#include <drivers/timer.h>
#include <drivers/uart.h>
#include <subsystems/cmdl.h>
#include "runloop.h"


//*****************************************************************************
//*************************** DEFINES AND MACROS ******************************
//*****************************************************************************


//*****************************************************************************
//**************************** LOCAL DATA TYPES *******************************
//*****************************************************************************

// Task states:
typedef enum runloopTaskState
{
    runloopTaskStateEmpty = 0,
    runloopTaskStateNew,
    runloopTaskStateActive
} runloopTaskStateT;

// Task structure:
typedef struct runloopTask
{
    RUNLOOP_CallbackT callbackPtr;
    void* callbackArgPtr;
    uint16_t remainingExecutions;
    uint16_t timeToNextExecutionMs;
    uint16_t periodMs;
    runloopTaskStateT state : 2;
} runloopTaskT;


//*****************************************************************************
//**************************** LOCAL VARIABLES ********************************
//*****************************************************************************

// All tasks in an unordered array:
static runloopTaskT runloopTaskSlotArr [RUNLOOP_MAX_NUMBER_OF_TASKS];

// Next task to schedule:
static runloopTaskT* runloopTaskHeadPtr = NULL;

// UART handle:
static UART_HandleT runloopUartHandle = NULL;

// TIMER handle:
static TIMER_HandleT runloopTimerHandle = NULL;

// RUNLOOP handle:
static struct RUNLOOP_State
{
    uint8_t initialized : 1;
    volatile uint8_t running : 1;
    volatile uint8_t flagCmdl : 1;
    volatile uint8_t flagCountdown : 1;
    volatile uint8_t flagTaskAdded : 1;
    TIMER_TimerIdT timerId : 2;
} runloopHandle;


//*****************************************************************************
//********************** LOCAL FUNCTION DECLARATIONS **************************
//*****************************************************************************

static void runloopEnterCmdl (void* optArgPtr);
static void runloopCountdownCallback (void* optArgPtr);
static void runloopActivateNewTasks (uint32_t elapsedTimeMs);
static int8_t runloopUpdateAndExecuteTasks (uint32_t elapsedTimeMs);


//*****************************************************************************
//**************************** LOCAL FUNCTIONS ********************************
//*****************************************************************************

/*!
*******************************************************************************
** \brief   Callback that makes the RUNLOOP enter an interactive
**          commandline mode.
**
** \param   optArgPtr   Not used by this function. Satisfies the callback
**                      interface.
**
*******************************************************************************
*/
static void runloopEnterCmdl (void* optArgPtr)
{
    runloopHandle.flagCmdl = 1;
    return;
}

/*!
*******************************************************************************
** \brief   Callback that makes the RUNLOOP wake up from idle mode
**          when a countdown has finished.
**
** \param   optArgPtr   Not used by this function. Satisfies the callback
**                      interface.
**
*******************************************************************************
*/
static void runloopCountdownCallback (void* optArgPtr)
{
    runloopHandle.flagCountdown = 1;
    return;
}

/*!
*******************************************************************************
** \brief   Marks all new tasks as active, which makes them executable.
**
** \param   elapsedTimeMs   Time in milliseconds since
**                          runloopUpdateAndExecuteTasks() was called last.
**                          It is needed to add an offset to the execution
**                          time, which is subtracted again before execution.
**
*******************************************************************************
*/
static void runloopActivateNewTasks (uint32_t elapsedTimeMs)
{
    uint8_t ii = 0;

    // Reset the flag "task added":
    runloopHandle.flagTaskAdded = 0;

    for (ii = 0; ii < RUNLOOP_MAX_NUMBER_OF_TASKS; ii++)
    {
        if (runloopTaskSlotArr[ii].state == runloopTaskStateNew)
        {
            runloopTaskSlotArr[ii].state = runloopTaskStateActive;
            runloopTaskSlotArr[ii].timeToNextExecutionMs += (uint16_t)elapsedTimeMs;
        }
    }
    return;
}

/*!
*******************************************************************************
** \brief   Executes all tasks ready to run.
**
**          This function executes all tasks that are ready to run.
**          If a task has been executed for the last time, its slot will
**          be freed for new tasks.
**          This function also updates the runloopTaskHeadPtr, which points
**          to the task with the smallest time to its next execution.
**
** \param   elapsedTimeMs   Time in milliseconds since this function was
**                          called the last time.
**
** \return  - 0 if no tasks were executed
**          - 1 if at least one task was executed
**
*******************************************************************************
*/
static int8_t runloopUpdateAndExecuteTasks (uint32_t elapsedTimeMs)
{
    runloopTaskT* task_ptr = NULL;
    runloopTaskT* task_head_ptr = NULL;
    uint8_t ii = 0;
    int8_t result = 0;
    int8_t task_was_executed = 0;
    uint32_t elapsed_time_overdue = 0;

    for (ii = 0; ii < RUNLOOP_MAX_NUMBER_OF_TASKS; ii++)
    {
        if (runloopTaskSlotArr[ii].state == runloopTaskStateActive)
        {
            task_ptr = &runloopTaskSlotArr[ii];
            if (task_ptr->timeToNextExecutionMs > elapsedTimeMs)
            {
                // Update execution time:
                task_ptr->timeToNextExecutionMs -= elapsedTimeMs;
            }
            else
            {
                // Execute task:
                result = task_ptr->callbackPtr(task_ptr->callbackArgPtr);

                // Update return value:
                task_was_executed = 1;

                if (result != 0)
                {
                    // Task returned with error code, invalidate task:
#if RUNLOOP_DEBUG
                    printf("[RUNLOOP] Task error: %d\n", result);
#endif
#if RUNLOOP_INTERRUPT_SAFETY
                    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
#endif
                    {
                        memset(task_ptr, 0, sizeof(runloopTaskT));
                    }
                }
                else
                {
                    // Update task information:
                    if (task_ptr->remainingExecutions < UINT16_MAX)
                    {
                        task_ptr->remainingExecutions--;
                    }
                    if (task_ptr->remainingExecutions)
                    {
                        // Update execution time:
                        elapsed_time_overdue = elapsedTimeMs - task_ptr->timeToNextExecutionMs;
                        if (task_ptr->periodMs > elapsed_time_overdue)
                        {
                            task_ptr->timeToNextExecutionMs = \
                                task_ptr->periodMs - elapsed_time_overdue;
                        }
                        else
                        {
                            task_ptr->timeToNextExecutionMs = 0;
                        }
                    }
                    else
                    {
                        // Task was executed for the last time, invalidate task:
#if RUNLOOP_INTERRUPT_SAFETY
                        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
#endif
                        {
                            memset(task_ptr, 0, sizeof(runloopTaskT));
                        }
                    }
                }
            }
            // Update the task head pointer
            // (also consider the case that the task became invalidated):
            if ((task_ptr->state == runloopTaskStateActive)
            &&  (  (task_head_ptr == NULL)
                || (task_ptr->timeToNextExecutionMs < task_head_ptr->timeToNextExecutionMs)))
            {
                task_head_ptr = task_ptr;
            }
        }
    }
    runloopTaskHeadPtr = task_head_ptr;
    return task_was_executed;
}


//*****************************************************************************
//*************************** PUBLIC FUNCTIONS ********************************
//*****************************************************************************


/*!
*******************************************************************************
** \brief   Initialize the RUNLOOP.
**
**          This function also registers the 'q' key with a UART rx callback
**          function that causes the RUNLOOP to enter an interactive
**          commandline mode. If the CMDL subsystem has not been initialized
**          by the application, it will be initialized during RUNLOOP
**          initialization.
**
** \param   timerId     Identifies a hardware timer to be used for timing
**                      operations in the RUNLOOP. A 16-bit timer allows for
**                      a greater resolution and may hence generate less
**                      interrupts during operation. Timer 2, however,
**                      is the only timer that allows to set the MCU into
**                      extended standby while the RUNLOOP is idle.
** \param   uartHandle  A valid UART handle that will be used to control
**                      the RUNLOOP via keys and a commandline interface.
**
** \return
**          - #RUNLOOP_OK on success.
**          - #RUNLOOP_ERR_TIMER_INITIALIZATION if the timer could not be
**              initialized.
**          - A UART specific error code if there was an error during
**              rx callback registration.
**          - A CMDL specific error code if there was an error during
**              CMDL initialization.
**
*******************************************************************************
*/
int8_t RUNLOOP_Init (TIMER_TimerIdT timerId,
                     UART_HandleT uartHandle)
{
    int8_t result = 0;

    TIMER_ClockPrescalerT clock_prescaler;
    TIMER_WaveGenerationT wave_generation_mode;
    TIMER_OutputModeT output_mode_A;
    TIMER_OutputModeT output_mode_B;
    UART_RxCallbackOptionsT rx_options;
    CMDL_OptionsT cmdl_options;

    if (runloopHandle.initialized)
    {
        return (RUNLOOP_ERR_ALREADY_INITIALIZED);
    }

    // Initialize local data structures:
    memset (runloopTaskSlotArr, 0, sizeof(runloopTaskSlotArr));
    memset (&runloopHandle, 0, sizeof(runloopHandle));

    // Initialize timer:
    clock_prescaler = TIMER_ClockPrescaler_1;
    wave_generation_mode = TIMER_WaveGeneration_NormalMode;
    output_mode_A = TIMER_OutputMode_NormalPortOperation;
    output_mode_B = TIMER_OutputMode_NormalPortOperation;
    runloopTimerHandle = TIMER_Init (timerId,
                                     clock_prescaler,
                                     wave_generation_mode,
                                     output_mode_A,
                                     output_mode_B);
    if (runloopTimerHandle == NULL)
    {
        return (RUNLOOP_ERR_TIMER_INITIALIZATION);
    }

    // Register 'q' key callback for interactive commandline mode:
    memset(&rx_options, 0, sizeof(rx_options));
    rx_options.execOnRxWait = 0;
    rx_options.writeRxToBuffer = 0;
    result = UART_RegisterRxCallback(uartHandle,
                                     (uint8_t)'q',
                                     runloopEnterCmdl,
                                     NULL,
                                     rx_options);
    if (result)
    {
        // Exit TIMER:
        (void)TIMER_Exit(runloopTimerHandle);
        // Return UART specific error code:
        return result;
    }

    // Initialize CMDL if not done so by the application:
    if ( ! CMDL_IsInitialized() )
    {
        memset (&cmdl_options, 0, sizeof(cmdl_options));
        cmdl_options.flushRxAfterExec = 1;
        cmdl_options.flushTxOnExit = 1;
        result = CMDL_Init(uartHandle, cmdl_options);
        if (result)
        {
            // Try to unregister previously registered RX callback:
            (void) UART_UnregisterRxCallback(uartHandle, (uint8_t)'q');
            // Exit TIMER:
            (void) TIMER_Exit(runloopTimerHandle);
            // Return CMDL specific error code:
            return result;
        }
    }

    runloopUartHandle = uartHandle;
    runloopHandle.timerId = timerId;
    runloopHandle.initialized = 1;
    return (RUNLOOP_OK);
}

/*!
*******************************************************************************
** \brief   Add a new task to the RUNLOOP.
**
** \param   callbackPtr         Defines the task to be executed.
** \param   callbackArgPtr      The argument that will be passed to the callback.
**                              May be NULL if not required by the callback.
** \param   numberOfExecutions  Defines the number of executions of the task.
**                              If it is set to 0 or UINT16_MAX, the task will
**                              be executed infinitely.
** \param   periodMs            Defines the delay between executions in ms.
**                              Must be greater than 0.
** \param   initialDelayMs      Defines an initial delay before the task will
**                              be executed for the first time.
**
** \return
**          - #RUNLOOP_OK on success.
**          - #RUNLOOP_ERR_BAD_PARAMETER if callbackPtr is NULL or if periodMs
**              is 0.
**          - #RUNLOOP_ERR_NO_TASK_SLOT_FREE if all task slots are taken.
**
*******************************************************************************
*/
int8_t RUNLOOP_AddTask (RUNLOOP_CallbackT callbackPtr,
                        void* callbackArgPtr,
                        uint16_t numberOfExecutions,
                        uint16_t periodMs,
                        uint16_t initialDelayMs)
{
    uint8_t ii = 0;
    runloopTaskT* task_ptr = NULL;

    if ((callbackPtr == NULL)
    ||  (periodMs == 0))
    {
        return (RUNLOOP_ERR_BAD_PARAMETER);
    }

#if RUNLOOP_INTERRUPT_SAFETY
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
#endif
    {
        // Search empty task slot:
        for (ii = 0; ii < RUNLOOP_MAX_NUMBER_OF_TASKS; ii++)
        {
            if (runloopTaskSlotArr[ii].state == runloopTaskStateEmpty) break;
        }
        if (ii >= RUNLOOP_MAX_NUMBER_OF_TASKS)
        {
            return (RUNLOOP_ERR_NO_TASK_SLOT_FREE);
        }
        task_ptr = &runloopTaskSlotArr[ii];
        task_ptr->state = runloopTaskStateNew;
    }

    // Populate task:
    task_ptr->callbackPtr = callbackPtr;
    task_ptr->callbackArgPtr = callbackArgPtr;
    if (numberOfExecutions == 0)
    {
        // Infinite execution:
        task_ptr->remainingExecutions = UINT16_MAX;
    }
    else
    {
        // Finite execution:
        task_ptr->remainingExecutions = numberOfExecutions;
    }
    task_ptr->timeToNextExecutionMs = initialDelayMs;
    task_ptr->periodMs = periodMs;

    runloopHandle.flagTaskAdded = 1;

    return (RUNLOOP_OK);
}

/*!
*******************************************************************************
** \brief   Start the RUNLOOP.
**
**          The RUNLOOP runs infinitely and does not return until
**          RUNLOOP_Stop() is called from an external event.
**          A task is removed from the task list if it was executed for
**          the last time or if it returned an error code other than 0.
**
*******************************************************************************
*/
void RUNLOOP_Run (void)
{
    int8_t task_was_executed = 0;
    uint32_t stopwatch_time = 0;

    // Reset/enable the timer's stopwatch:
    TIMER_ResetStopwatch(runloopTimerHandle,
                         TIMER_Stopwatch_On);

    // Reset flags:
    runloopHandle.running = 1;
    runloopHandle.flagCmdl = 0;
    runloopHandle.flagCountdown = 0;

    // RUNLOOP execution can be halted by RUNLOOP_Stop():
    while ( runloopHandle.running )
    {
        // Enable watchdog timer:
        wdt_enable(WDTO_1S);

        // RUNLOOP execution can be temporarily superseded by CMDL execution:
        while (runloopHandle.running
        &&    (runloopHandle.flagCmdl == 0))
        {
            // The timer is stopped at this point.

            // Set the clock prescaler to maximum (generates least interrupts):
            TIMER_SetClockPrescaler(runloopTimerHandle,
                                             TIMER_ClockPrescaler_1024);

            do
            {
                // Get the elapsed time measured by the stopwatch:
                TIMER_GetStopwatchTimeMs(runloopTimerHandle, &stopwatch_time);

#if RUNLOOP_INTERRUPT_SAFETY
                ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
#endif
                {
                    // Activate new tasks:
                    if (runloopHandle.flagTaskAdded)
                    {
                        runloopActivateNewTasks(stopwatch_time);
                    }

                    // Reset the stopwatch:
                    TIMER_ResetStopwatch(runloopTimerHandle,
                                         TIMER_Stopwatch_On);
                }

                // Start the stopwatch (runs concurrently to executing tasks):
                TIMER_Start(runloopTimerHandle);

                // Update the execution times of all tasks and
                // execute all tasks whose execution times have expired:
                task_was_executed = runloopUpdateAndExecuteTasks(stopwatch_time);

                // Reset watchdog:
                wdt_reset();

                // Stop the stopwatch:
                TIMER_Stop(runloopTimerHandle, TIMER_Stop_ImmediatelyAndReset);

            } while (runloopHandle.running
              &&     (task_was_executed || runloopHandle.flagTaskAdded));
            // as long as at least one task was executed or added

            // Reset the countdown flag:
            runloopHandle.flagCountdown = 0;

            // Set the countdown to the execution time of the next task.
            // The stopwatch does also run:
            TIMER_StartCountdown(runloopTimerHandle,
                                 runloopCountdownCallback,
                                 NULL,
                                 runloopTaskHeadPtr->timeToNextExecutionMs,
                                 1);

            // Sleep while the runloop is idle:
            while ((runloopHandle.running)
            &&     (runloopHandle.flagCmdl == 0)
            &&     (runloopHandle.flagCountdown == 0)
            &&     (runloopHandle.flagTaskAdded == 0))
            {
                // Reset watchdog:
                wdt_reset();

                // Enter sleep mode while idle:
                cli();
                #if RUNLOOP_DEBUG
                set_sleep_mode(SLEEP_MODE_IDLE);
                #else
                // Timer 2 allows wake-up from power save mode / extended stand-by
                if (runloopHandle.timerId == TIMER_TimerId_2)
                {
                    set_sleep_mode(SLEEP_MODE_EXT_STANDBY);
                }
                else
                {
                    set_sleep_mode(SLEEP_MODE_IDLE);
                }
                #endif
                //sleep_mode(); // or alternatively:
                sleep_enable();
                sleep_bod_disable();
                sei();
                sleep_cpu();
                sleep_disable();
            }
            if (runloopHandle.flagCountdown == 0)
            {
                // If the countdown flag is set, the timer is already stopped
                TIMER_Stop(runloopTimerHandle, TIMER_Stop_ImmediatelyAndReset);
            }
        }
        if (runloopHandle.flagCmdl)
        {
            // Enter interactive command line interface:
            runloopHandle.flagCmdl = 0;
            wdt_disable();
            CMDL_Run();
            UART_TxFlush(runloopUartHandle);
        }
    }
    // Disable watchdog timer:
    wdt_disable();
    return;
}

/*!
*******************************************************************************
** \brief   Stop the RUNLOOP.
**
**          This function makes the RUNLOOP execution halt.
**
** \param   optArgPtr   An optional argument pointer which is not used here
**                      and may be NULL. Its purpose is to implement the
**                      interface used by many callback functions in AVR3nk.
**
*******************************************************************************
*/
void RUNLOOP_Stop (void* optArgPtr)
{
    runloopHandle.running = 0;
    return;
}

//*****************************************************************************
//*********************** INTERRUPT SERVICE ROUTINES **************************
//*****************************************************************************

