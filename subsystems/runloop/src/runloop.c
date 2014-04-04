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
**          The runloop is not subject to any systematic timing drifts as
**          it makes use of a continuously running timer.
**          However, the execution of a task may be subject to a timing jitter,
**          which depends mainly on two factors:
**          - If the system clock frequency is not a multiple of 1024000 Hz
**              (e.g., 18432000 Hz = 18 * 1024000 Hz), the underlying timer does
**              not pass the 1ms boundaries at clock cycles multiple of the
**              prescaler because the milliseconds are not aligned with the
**              timer's prescaled clock. Hence, a task may be delayed
**              by up to "prescaler" system clock cycles even if no other tasks
**              are running. If this is an issue, a crystal/oscillator whose
**              frequency is a multiple of 1024000 Hz should be used.
**              Alternatively, the jitter can be reduced by using a smaller
**              timer prescaler at the cost of additional interrupts, which
**              may slow down the MCU a bit.
**          - If multiple tasks are running and the scheduling times of two
**              tasks overlap, the second task will be delayed until the first
**              task has finished execution.
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

#if RUNLOOP_DEBUG
#include <stdio.h>
#endif


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
    runloopTaskStateReady,
    runloopTaskStateActive
} runloopTaskStateT;

// Task structure:
typedef struct runloopTask
{
    RUNLOOP_TaskCallbackT callbackPtr;
    void* callbackArgPtr;
    uint16_t remainingExecutions;
    uint32_t cyclesToNextExecution;
    uint32_t cyclesPerPeriod;
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
    volatile uint8_t flagStopwatch : 1;
    volatile uint8_t flagTaskAdded : 1;
    TIMER_TimerIdT timerId : 2;
} runloopHandle;

// Callback that will be executed on task errors:
static RUNLOOP_TaskErrorCallbackT runloopTaskErrorCallback = NULL;

// Callback that will be executed when task executions are dropped:
static RUNLOOP_SyncErrorCallbackT runloopSyncErrorCallback = NULL;


//*****************************************************************************
//********************** LOCAL FUNCTION DECLARATIONS **************************
//*****************************************************************************

static void runloopEnterCmdl (void* optArgPtr);
static void runloopStopwatchCallback (void* optArgPtr);
static void runloopActivateNewTasks (uint32_t elapsedCycles);
static uint8_t runloopUpdateAndExecuteTasks (uint32_t elapsedCycles);
#if RUNLOOP_DEBUG
static void runloopPrintTask(uint8_t ii, uint32_t elapsedCycles, runloopTaskT* taskPtr);
#endif


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
#if RUNLOOP_DEBUG
    printf("Entering CMDL.\n");
#endif
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
static void runloopStopwatchCallback (void* optArgPtr)
{
    runloopHandle.flagStopwatch = 1;
    return;
}

/*!
*******************************************************************************
** \brief   Marks all new tasks as active, which makes them executable.
**
** \param   elapsedCycles   Approximate number of system clock cycles since
**                          runloopUpdateAndExecuteTasks() was called last.
**                          It is needed here to add an offset to the execution
**                          time, which is subtracted again before execution.
**
*******************************************************************************
*/
static void runloopActivateNewTasks (uint32_t elapsedCycles)
{
#if RUNLOOP_INTERRUPT_SAFETY
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
#endif
    {
        uint8_t ii = 0;

        // Reset the flag "task added":
        runloopHandle.flagTaskAdded = 0;

        for (ii = 0; ii < RUNLOOP_MAX_NUMBER_OF_TASKS; ii++)
        {
            if (runloopTaskSlotArr[ii].state == runloopTaskStateNew)
            {
                runloopTaskSlotArr[ii].state = runloopTaskStateReady;
            }
        }
    }
    return;
}

/*!
*******************************************************************************
** \brief   Updates all ready and active tasks and executes them if their
**          time to execution has elapsed.
**
**          This function executes all tasks that are ready to run.
**          If a task is executed for the last time, its slot will be freed.
**          This function also updates the runloopTaskHeadPtr, which points
**          to the task with the smallest time to its next execution.
**
** \param   elapsedCycles   Approxmiate number of system clock cycles since this
**                          function was called the last time.
**
** \return  - The number of tasks that were executed. If no tasks are
**            executed, it will return 0.
**
*******************************************************************************
*/
static uint8_t runloopUpdateAndExecuteTasks (uint32_t elapsedCycles)
{
    runloopTaskT* task_ptr = NULL;
    runloopTaskT* task_head_ptr = NULL;
    uint8_t ii = 0;
    uint8_t result = 0;
    uint8_t tasks_executed = 0;
    uint32_t elapsed_cycles_overdue = 0;

    for (ii = 0; ii < RUNLOOP_MAX_NUMBER_OF_TASKS; ii++)
    {
        task_ptr = NULL;
        if (runloopTaskSlotArr[ii].state == runloopTaskStateActive)
        {
            task_ptr = &runloopTaskSlotArr[ii];
#if RUNLOOP_DEBUG
            runloopPrintTask(ii, elapsedCycles, task_ptr);
#endif
            if (task_ptr->cyclesToNextExecution > elapsedCycles)
            {
                // Update execution time:
                task_ptr->cyclesToNextExecution -= elapsedCycles;
            }
            else
            {
                // Execute task:
                result = task_ptr->callbackPtr(task_ptr->callbackArgPtr);
                tasks_executed++;
                if (result != RUNLOOP_OK)
                {
#if RUNLOOP_DEBUG
                    printf("[RUNLOOP] Task %u: %u\n", ii, result);
#endif
                    // Task returned with RUNLOOP_OK_TASK_ABORT or with error code,
                    // invalidate task:
#if RUNLOOP_INTERRUPT_SAFETY
                    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
#endif
                    {
                        memset(task_ptr, 0, sizeof(runloopTaskT));
                    }
                    if ((result != RUNLOOP_OK_TASK_ABORT)
                    &&  (runloopTaskErrorCallback))
                    {
                        runloopTaskErrorCallback(ii, result);
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
                        elapsed_cycles_overdue = elapsedCycles - task_ptr->cyclesToNextExecution;
                        if (task_ptr->cyclesPerPeriod > elapsed_cycles_overdue)
                        {
                            task_ptr->cyclesToNextExecution = \
                                task_ptr->cyclesPerPeriod - elapsed_cycles_overdue;
                        }
                        else
                        {
                            // Skip one or more executions of the task, but re-align
                            // the timing of the task to its previous pattern:
                            task_ptr->cyclesToNextExecution = task_ptr->cyclesPerPeriod - \
                                (elapsed_cycles_overdue % task_ptr->cyclesPerPeriod);
                            if (runloopSyncErrorCallback)
                            {
                                // Execute error callback and give number of drops:
                                runloopSyncErrorCallback(ii, (uint16_t) \
                                    (elapsed_cycles_overdue / task_ptr->cyclesPerPeriod));
                            }
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
        }
        else if (runloopTaskSlotArr[ii].state == runloopTaskStateReady)
        {
            task_ptr = &runloopTaskSlotArr[ii];
#if RUNLOOP_DEBUG
            runloopPrintTask(ii, elapsedCycles, task_ptr);
#endif
            task_ptr->state = runloopTaskStateActive;

            if (task_ptr->cyclesToNextExecution == 0)
            {
                // Execute task:
                result = task_ptr->callbackPtr(task_ptr->callbackArgPtr);
                tasks_executed++;
                task_ptr->cyclesToNextExecution = task_ptr->cyclesPerPeriod;
                if (result != RUNLOOP_OK)
                {
#if RUNLOOP_DEBUG
                    printf("[RUNLOOP] Task %u: %u\n", ii, result);
#endif
                    // Task returned with RUNLOOP_OK_TASK_ABORT or with error code,
                    // invalidate task:
#if RUNLOOP_INTERRUPT_SAFETY
                    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
#endif
                    {
                        memset(task_ptr, 0, sizeof(runloopTaskT));
                    }
                    if ((result != RUNLOOP_OK_TASK_ABORT)
                    &&  (runloopTaskErrorCallback))
                    {
                        runloopTaskErrorCallback(ii, result);
                    }
                }
                else
                {
                    // Update task information:
                    if (task_ptr->remainingExecutions < UINT16_MAX)
                    {
                        task_ptr->remainingExecutions--;
                    }
                    if (task_ptr->remainingExecutions == 0)
                    {
                        // Task was executed only once, invalidate task:
#if RUNLOOP_INTERRUPT_SAFETY
                        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
#endif
                        {
                            memset(task_ptr, 0, sizeof(runloopTaskT));
                        }
                    }
                }
            }
        }
        // Update the task head pointer:
        if ((task_ptr)
        &&  (  (task_head_ptr == NULL)
            || (task_ptr->cyclesToNextExecution < task_head_ptr->cyclesToNextExecution)))
        {
            task_head_ptr = task_ptr;
        }
    }
    runloopTaskHeadPtr = task_head_ptr;
    return tasks_executed;
}

#if RUNLOOP_DEBUG
static void runloopPrintTask(uint8_t ii, uint32_t elapsedCycles, runloopTaskT* taskPtr)
{
    //printf("el.: %lu\n", elapsedCycles);
    ///*
    printf("\n");
    printf("================\n");
    printf("Task %02u - Cycles elapsed: %lu\n", ii, elapsedCycles);
    printf("================\n");
    printf("callbackPtr:    0x%04x\n", (uint16_t)taskPtr->callbackPtr);
    printf("callbackArgPtr: 0x%04x\n", (uint16_t)taskPtr->callbackArgPtr);
    printf("remainingExec.: %u\n",     taskPtr->remainingExecutions);
    printf("cyclesToNextE.: %lu\n",    taskPtr->cyclesToNextExecution);
    printf("cyclesPerPd.:   %lu\n",    taskPtr->cyclesPerPeriod);
    printf("state:          %u\n",     taskPtr->state);
    return;
    // */
}
#endif

//*****************************************************************************
//*************************** PUBLIC FUNCTIONS ********************************
//*****************************************************************************


/*!
*******************************************************************************
** \brief   Initialize the RUNLOOP.
**
**          TODO: CMDL behavior
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
** \param   timerClockPrescaler
**                      Specifies the clock prescaler for the timer.
**                      It is highly recommended to set up the runloop with
**                      TIMER_ClockPrescaler_1024 for efficiency. However,
**                      if the system clock frequency F_CPU is not a multiple
**                      of 1024 kHz, a lower prescaler may reduce the
**                      timing jitter (on the precondition that the execution
**                      times of tasks do not overlap). This may be helpful
**                      in applications which require precise timing
**                      intervals. Note that if F_CPU is a multiple of
**                      1024 kHz, the duration of a millisecond is a multiple
**                      of the timer's prescaled clock, so there is no need to
**                      use a lower prescaler.
** \param   uartHandle  A valid UART handle that will be used to control
**                      the RUNLOOP via keys and a commandline interface.
** \param   taskErrorCallback
**                      This argument allows to register a callback with the
**                      runloop that is executed whenever a task returns
**                      with an error code greater than 1. Note that the
**                      error codes 0 and 1 have special meanings.
**                      The error code 0 (RUNLOOP_OK) means that a task has
**                      executed successfully. The error code 1
**                      (RUNLOOP_OK_TASK_ABORT) means that a task
**                      has executed successfully but that it should be
**                      removed from the runloop. The callback receives
**                      two arguments: the task id and the error code returned
**                      by the task.
** \param   syncErrorCallback
**                      This argument allows to register a callback with the
**                      runloop that is executed whenever a task is delayed
**                      by more than its execution period. In this case,
**                      the runloop will skip one or more executions of the
**                      task. However, the runloop re-aligns future executions
**                      of the task to its previous timing pattern, if possible.
**                      The callback receives two arguments: the task id, and
**                      the count of dropped executions.
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
// TODO: CMDL while running?
int8_t RUNLOOP_Init (TIMER_TimerIdT timerId,
                     TIMER_ClockPrescalerT timerClockPrescaler,
                     UART_HandleT uartHandle,
                     RUNLOOP_TaskErrorCallbackT taskErrorCallback,
                     RUNLOOP_SyncErrorCallbackT syncErrorCallback)
{
    int8_t result = 0;

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
    wave_generation_mode = TIMER_WaveGeneration_NormalMode;
    output_mode_A = TIMER_OutputMode_NormalPortOperation;
    output_mode_B = TIMER_OutputMode_NormalPortOperation;
    runloopTimerHandle = TIMER_Init (timerId,
                                     timerClockPrescaler,
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

    runloopTaskErrorCallback = taskErrorCallback;
    runloopSyncErrorCallback = syncErrorCallback;
    runloopUartHandle = uartHandle;
    runloopHandle.timerId = timerId;
    runloopHandle.initialized = 1;
    return (RUNLOOP_OK);
}

/*!
*******************************************************************************
** \brief   Add a new task to the RUNLOOP.
**
** \note    The maximum value that can be passed through periodMs and
**          initialDelayMs is (2^32 - 1) / (F_CPU / 1000) milliseconds.
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
** \param   taskIdPtr           Receives a unique identifier of the task, which
**                              may be used to identify tasks in the error
**                              callbacks. The argument may be NULL if not needed.
**
** \return
**          - #RUNLOOP_OK on success.
**          - #RUNLOOP_ERR_BAD_PARAMETER if callbackPtr is NULL or if periodMs
**              is 0.
**          - #RUNLOOP_ERR_NO_TASK_SLOT_FREE if all task slots are taken.
**
*******************************************************************************
*/
int8_t RUNLOOP_AddTask (RUNLOOP_TaskCallbackT callbackPtr,
                        void* callbackArgPtr,
                        uint16_t numberOfExecutions,
                        uint32_t periodMs,
                        uint32_t initialDelayMs,
                        uint8_t* taskIdPtr)
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
    task_ptr->cyclesToNextExecution = initialDelayMs * ((F_CPU) / 1000UL);
    task_ptr->cyclesPerPeriod = periodMs * ((F_CPU) / 1000UL);

    runloopHandle.flagTaskAdded = 1;

    // Disclose task id:
    *taskIdPtr = ii;

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
    uint8_t tasks_executed = 0;
    uint32_t stopwatch_cycles = 0;

    // Reset flags:
    runloopHandle.running = 1;
    runloopHandle.flagCmdl = 0;
    runloopHandle.flagStopwatch = 0;

    // Stop and reset the timer:
    TIMER_Stop(runloopTimerHandle, TIMER_Stop_ImmediatelyAndReset);

    // Reset and enable the timer's stopwatch:
    TIMER_EnableDisableStopwatch(runloopTimerHandle, TIMER_Stopwatch_Enable);

    // Start the timer:
    TIMER_Start(runloopTimerHandle);

    // RUNLOOP execution can be halted by RUNLOOP_Stop():
    while (runloopHandle.running)
    {
        // Enable watchdog timer:
        wdt_enable(WDTO_1S);

        // RUNLOOP execution can be temporarily superseded by CMDL execution:
        while (runloopHandle.running
        &&    (runloopHandle.flagCmdl == 0))
        {
            do
            {
                // Get the elapsed time measured by the stopwatch:
                TIMER_GetStopwatchSystemClockCycles(runloopTimerHandle,
                                                    &stopwatch_cycles,
                                                    TIMER_Stopwatch_Reset);

                // Activate new tasks:
                if (runloopHandle.flagTaskAdded)
                {
                    runloopActivateNewTasks(stopwatch_cycles);
                }

                // Update the execution times of all tasks and
                // execute all tasks whose execution times have expired:
                tasks_executed = runloopUpdateAndExecuteTasks(stopwatch_cycles);

                // Reset watchdog:
                wdt_reset();

            } while (runloopHandle.running
              &&     (tasks_executed || runloopHandle.flagTaskAdded));
            // as long as at least one task was executed or added

            // Reset the countdown flag:
            runloopHandle.flagStopwatch = 0;

            // Make the timer's stopwatch notify the runloop when the next
            // execution time is reached:
            if (runloopTaskHeadPtr)
            {
                TIMER_SetStopwatchTimeCallback \
                                    (runloopTimerHandle,
                                     runloopStopwatchCallback,
                                     NULL,
                                     runloopTaskHeadPtr->cyclesToNextExecution);
            }

            // Sleep while the runloop is idle:
            while ((runloopHandle.running)
            &&     (runloopHandle.flagCmdl == 0)
            &&     (runloopHandle.flagStopwatch == 0)
            &&     (runloopHandle.flagTaskAdded == 0))
            {
                // Reset watchdog:
                wdt_reset();

                // /*
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
                // */
            }
        }
        if (runloopHandle.flagCmdl)
        {
            // Enter interactive command line interface:
            TIMER_Stop(runloopTimerHandle, TIMER_Stop_Immediately);
            wdt_disable();
            CMDL_Run();
            UART_TxFlush(runloopUartHandle);
            runloopHandle.flagCmdl = 0;
            TIMER_Start(runloopTimerHandle);
        }
    }
    // Disable watchdog timer:
    wdt_disable();

    // Disable the timer's stopwatch:
    TIMER_EnableDisableStopwatch(runloopTimerHandle, TIMER_Stopwatch_Disable);

    // Stop and reset the timer:
    TIMER_Stop(runloopTimerHandle, TIMER_Stop_ImmediatelyAndReset);
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

