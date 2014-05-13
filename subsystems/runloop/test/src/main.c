/*!
*******************************************************************************
*******************************************************************************
** \brief   Test set for the RUNLOOP subsystem.
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <drivers/macros_pin.h>
#include <drivers/uart.h>
#include <drivers/timer.h>
#include <subsystems/cmdl.h>
#include <subsystems/runloop.h>


//*****************************************************************************
//*************************** DEFINES AND MACROS ******************************
//*****************************************************************************

#define APP_LED D,7

//*****************************************************************************
//**************************** LOCAL DATA TYPES *******************************
//*****************************************************************************


//*****************************************************************************
//********************** LOCAL FUNCTION DECLARATIONS **************************
//*****************************************************************************

static int8_t  appInit (void);
static int     appStdioPut (char chr, FILE* streamPtr);
static int     appStdioGet (FILE* streamPtr);
static void    appTaskErrorCallback (uint8_t taskId, uint8_t errorCode);
static void    appSyncErrorCallback (uint8_t taskId, uint16_t dropCount);
static uint8_t appToggleLedTask (void* optArgPtr);
static uint8_t appPrintUptimeTask (void* optArgPtr);
static uint8_t appActiveWaitingTask (void* optArgPtr);
#if RUNLOOP_WITH_CMDL
static void    appAddToggleLedTaskViaCmdl (uint8_t argc, char* argv[]);
static void    appAddPrintUptimeTaskViaCmdl (uint8_t argc, char* argv[]);
static void    appAddActiveWaitingTaskViaCmdl (uint8_t argc, char* argv[]);
#else
static void    appAddToggleLedTaskViaKey (void* optArgPtr);
static void    appAddPrintUptimeTaskViaKey (void* optArgPtr);
static void    appAddActiveWaitingTaskViaKey (void* optArgPtr);
#endif


//*****************************************************************************
//**************************** LOCAL VARIABLES ********************************
//*****************************************************************************

static uint8_t appMCUSR __attribute__((section(".noinit")));
static UART_HandleT appUartHandle = NULL;
static FILE appStdio = FDEV_SETUP_STREAM(appStdioPut, appStdioGet, _FDEV_SETUP_RW);
static uint8_t appPinIsHigh = 0;
static uint32_t appActiveWaitingTaskDelayMs = 0;

//*****************************************************************************
//**************************** LOCAL FUNCTIONS ********************************
//*****************************************************************************

/*!
*******************************************************************************
** \brief   Initialize the application.
**
**          The UART driver and the commandline subsystem are initialized
**          and commands are registered with the commandline.
**
** \return
**          - 0 on success.
**          - -1 on error.
**
*******************************************************************************
*/
static int8_t appInit (void)
{
    int8_t result;
    UART_LedParamsT led_params;
#if (RUNLOOP_WITH_CMDL == 0)
    UART_RxCallbackOptionsT uart_cb_opts;
#endif

    // Initialize UART and STDIO:
    memset(&led_params, 0, sizeof(led_params));
    led_params.txLedPortPtr = &PORTA;
    led_params.txLedDdrPtr  = &DDRA;
    led_params.txLedIdx     = 6;
    led_params.rxLedPortPtr = &PORTA;
    led_params.rxLedDdrPtr  = &DDRA;
    led_params.rxLedIdx     = 7;

    appUartHandle = UART_Init(UART_InterfaceId0,
                              UART_Baud_230400,
                              UART_Parity_off,
                              UART_StopBit_1,
                              UART_CharSize_8,
                              UART_Transceive_RxTx,
                              &led_params);
    if(appUartHandle == NULL)
    {
        return(-1);
    }

    // Assign stdio:
    stdout = &appStdio;
    stdin  = &appStdio;

    // Enable interrupts:
    sei();

    // Initialize RUNLOOP:
    result = RUNLOOP_Init(TIMER_TimerId_1,
                          TIMER_ClockPrescaler_1024,
                          appUartHandle,
                          appTaskErrorCallback,
                          appSyncErrorCallback);
    if (result != RUNLOOP_OK)
    {
        printf("RUNLOOP_Init: %d\n", result);
        return result;
    }

#if RUNLOOP_WITH_CMDL
    // Register CMDL commands:
    result = CMDL_RegisterCommand(appAddToggleLedTaskViaCmdl,
                                  "toggle");
    if (result != CMDL_OK)
    {
        printf ("CMDL_RegisterCommand: %d\n", result);
        return result;
    }
    result = CMDL_RegisterCommand(appAddPrintUptimeTaskViaCmdl,
                                  "uptime");
    if (result != CMDL_OK)
    {
        printf ("CMDL_RegisterCommand: %d\n", result);
        return result;
    }
    result = CMDL_RegisterCommand(appAddActiveWaitingTaskViaCmdl,
                                  "wait");
    if (result != CMDL_OK)
    {
        printf ("CMDL_RegisterCommand: %d\n", result);
        return result;
    }
#else
    // Register UART callback that adds a task to the runloop:
    memset(&uart_cb_opts, 0, sizeof(uart_cb_opts));
    uart_cb_opts.execOnRxWait = 0;
    uart_cb_opts.writeRxToBuffer = 1;
    result = UART_RegisterRxCallback(appUartHandle,
                                     (uint8_t)'f',
                                     appAddToggleLedTaskViaKey,
                                     NULL,
                                     uart_cb_opts);
    if (result != UART_OK)
    {
        printf("UART_RegisterRxCallback: %d\n", result);
        return(result);
    }
    result = UART_RegisterRxCallback(appUartHandle,
                                     (uint8_t)'u',
                                     appAddPrintUptimeTaskViaKey,
                                     NULL,
                                     uart_cb_opts);
    if (result != UART_OK)
    {
        printf("UART_RegisterRxCallback: %d\n", result);
        return(result);
    }
    result = UART_RegisterRxCallback(appUartHandle,
                                     (uint8_t)'w',
                                     appAddActiveWaitingTaskViaKey,
                                     NULL,
                                     uart_cb_opts);
    if (result != UART_OK)
    {
        printf("UART_RegisterRxCallback: %d\n", result);
        return(result);
    }
#endif

    // Set the application's LED as output:
    appPinIsHigh = 0;
    SET_LOW(APP_LED);
    SET_OUTPUT(APP_LED);

    return(0);
}

/*!
*******************************************************************************
** \brief   Print a character on the UART output.
**
**          This procedure will be used in combination with stdio when switching
**          the standard output to the UART interface.
**
** \param   chr         The character that will be printed.
** \param   streamPtr   Not used.
**
** \return  0
**
*******************************************************************************
*/
static int appStdioPut (char chr, FILE* streamPtr)
{
    UART_TxByte(appUartHandle, chr);
    return(0);
}

/*!
*******************************************************************************
** \brief   Read a character from the UART output.
**
**          This procedure will be used in combination with stdio when switching
**          the standard input to the UART interface.
**
** \param   streamPtr   Not used.
**
** \return  The read character.
**
*******************************************************************************
*/
static int appStdioGet (FILE* streamPtr)
{
    return((int)UART_RxByte(appUartHandle));
}

/*!
*******************************************************************************
** \brief   Callback function that will be executed by the runloop when
**          a task returns with an error code other than RUNLOOP_OK
**          or RUNLOOP_OK_TASK_ABORT.
**
** \param   taskId      Contains the id of the task.
** \param   errorCode   Contains the error code returned by the task.
**
*******************************************************************************
*/
static void appTaskErrorCallback (uint8_t taskId, uint8_t errorCode)
{
    printf ("Task error. Task ID: %u Return code: %u\n", taskId, errorCode);
    return;
}

/*!
*******************************************************************************
** \brief   Callback function that will be executed by the runloop when
**          it loses synchronization and needs to skip task executions.
**
** \param   taskId      Contains the id of the task that was skipped.
** \param   dropCount   Number of executions that were skipped.
**
*******************************************************************************
*/
static void appSyncErrorCallback (uint8_t taskId, uint16_t dropCount)
{
    printf(" Sync error. Task ID: %u Drop count: %u\n", taskId, dropCount);
    return;
}

/*!
*******************************************************************************
** \brief   Toggle the application's LED.
**
*******************************************************************************
*/
static uint8_t appToggleLedTask (void* optArgPtr)
{
    if (appPinIsHigh)
    {
        SET_LOW(APP_LED);
        appPinIsHigh = 0;
    }
    else
    {
        SET_HIGH(APP_LED);
        appPinIsHigh = 1;
    }
    return (RUNLOOP_OK);
}

/*!
*******************************************************************************
** \brief   Print the uptime.
**
*******************************************************************************
*/
static uint8_t appPrintUptimeTask (void* optArgPtr)
{
    uint16_t days = 0;
    uint8_t  hours = 0;
    uint8_t  minutes = 0;
    uint8_t  seconds = 0;
    uint16_t milliseconds = 0;

    RUNLOOP_GetUptimeHumanReadable (&days,
                                    &hours,
                                    &minutes,
                                    &seconds,
                                    &milliseconds);
    if (days)
    {
        printf ("Uptime: %d days %02u:%02u:%02u.%03u\n", \
                days, hours, minutes, seconds, milliseconds);
    }
    else
    {
        printf ("Uptime: %02u:%02u:%02u.%03u\n", \
                hours, minutes, seconds, milliseconds);
    }
    return (RUNLOOP_OK);
}

/*!
*******************************************************************************
** \brief   Wait actively for a specified amount of time.
**
** \param   optArgPtr   Should point to a uint32_t containing the number of
**                      milliseconds to wait.
**
*******************************************************************************
*/
static uint8_t appActiveWaitingTask (void* optArgPtr)
{
    uint32_t ii = 0;
    for (ii = 0; ii < *(uint32_t*)optArgPtr; ii++)
    {
        _delay_ms(1UL);
    }
    return (RUNLOOP_OK);
}

#if RUNLOOP_WITH_CMDL
/*!
*******************************************************************************
** \brief   Callback for the CMDL which adds the LED toggle task to the RUNLOOP.
**
** \param   argc    The argument count is should be 4.
** \param   argv    The argument vector is expected to contain the command name,
**                  the number of executions of the task, the task's period
**                  in ms and the initial delay of the task in ms.
**
*******************************************************************************
*/
static void appAddToggleLedTaskViaCmdl (uint8_t argc, char* argv[])
{
    int8_t result = 0;
    uint8_t task_id = 0;

    if (argc != 4)
    {
        printf ("Usage: %s <numOfExec> <periodMs> <initialDelayMs>\n", argv[0]);
        return;
    }
    result = RUNLOOP_AddTask(appToggleLedTask,
                             NULL,
                             (uint16_t)strtoul(argv[1], NULL, 0),
                             strtoul(argv[2], NULL, 0),
                             strtoul(argv[3], NULL, 0),
                             &task_id);
    if (result != (RUNLOOP_OK))
    {
        printf("Error in RUNLOOP_AddTask(): %d\n", result);
    }
    else
    {
        printf("Task added. Task ID: %u\n", task_id);
    }
    return;
}

/*!
*******************************************************************************
** \brief   Callback for the CMDL which adds the uptime printing task to
**          the RUNLOOP.
**
** \param   argc    The argument count is should be 1 or 4.
** \param   argv    The argument vector is expected to contain either
**                  - only the command name, or
**                  - the command name, the number of executions of the task,
**                    the task's period in ms and the initial delay of the task
**                    in ms.
**
*******************************************************************************
*/
static void appAddPrintUptimeTaskViaCmdl (uint8_t argc, char* argv[])
{
    int8_t result = 0;
    uint8_t task_id = 0;

    if (argc == 1)
    {
        appPrintUptimeTask(NULL);
    }
    else if (argc == 4)
    {
        result = RUNLOOP_AddTask(appPrintUptimeTask,
                                 NULL,
                                 (uint16_t)strtoul(argv[1], NULL, 0),
                                 strtoul(argv[2], NULL, 0),
                                 strtoul(argv[3], NULL, 0),
                                 &task_id);
        if (result != (RUNLOOP_OK))
        {
            printf("Error in RUNLOOP_AddTask(): %d\n", result);
        }
        else
        {
            printf("Task added. Task ID: %u\n", task_id);
        }
    }
    else
    {
        printf ("Usage: %s <numOfExec> <periodMs> <initialDelayMs>\n", argv[0]);
        return;
    }
    return;
}

/*!
*******************************************************************************
** \brief   Callback for the CMDL which adds the active waiting task to the
**          RUNLOOP.
**
** \param   argc    The argument count is should be 5.
** \param   argv    The argument vector is expected to contain the command name,
**                  the number of milliseconds for active waiting, the number
**                  of executions of the task, the task's period in ms and
**                  the initial delay of the task in ms.
**
*******************************************************************************
*/
static void appAddActiveWaitingTaskViaCmdl (uint8_t argc, char* argv[])
{
    int8_t result = 0;
    uint8_t task_id = 0;

    if (argc != 5)
    {
        printf ("Usage: %s <activeWaitingDelayMs> <numOfExec> <periodMs> <initialDelayMs>\n", argv[0]);
        return;
    }
    appActiveWaitingTaskDelayMs = strtoul(argv[1], NULL, 0);
    result = RUNLOOP_AddTask(appActiveWaitingTask,
                             (void*)&appActiveWaitingTaskDelayMs,
                             (uint16_t)strtoul(argv[2], NULL, 0),
                             strtoul(argv[3], NULL, 0),
                             strtoul(argv[4], NULL, 0),
                             &task_id);
    if (result != (RUNLOOP_OK))
    {
        printf("Error in RUNLOOP_AddTask(): %d\n", result);
    }
    else
    {
        printf("Task added. Task ID: %u\n", task_id);
    }
    return;
}

#else // RUNLOOP_WITH_CMDL

/*!
*******************************************************************************
** \brief   Callback for the UART which adds the toggle task to the RUNLOOP.
**
** \param   optArgPtr   Not used.
**
*******************************************************************************
*/
static void appAddToggleLedTaskViaKey (void* optArgPtr)
{
    int8_t result = 0;
    uint8_t task_id = 0;
    result = RUNLOOP_AddTask(appToggleLedTask,
                             NULL,   // optional argument
                             16,     // number of executions
                             500,    // period in ms
                             0,      // initial delay in ms
                             &task_id);  // task id pointer
    if (result != (RUNLOOP_OK))
    {
        printf("Error in RUNLOOP_AddTask(): %d\n", result);
    }
    else
    {
        printf("Task added. Task ID: %u\n", task_id);
    }
    return;
}

/*!
*******************************************************************************
** \brief   Callback for the UART which adds the uptime printing task to
**          the RUNLOOP.
**
** \param   optArgPtr   Not used.
**
*******************************************************************************
*/
static void appAddPrintUptimeTaskViaKey (void* optArgPtr)
{
    int8_t result = 0;
    uint8_t task_id = 0;
    result = RUNLOOP_AddTask(appPrintUptimeTask,
                             NULL,  // optional argument
                             1,     // one execution
                             0,     // period in ms (may be 0 for non-periodic tasks)
                             0,     // initial delay in ms
                             &task_id);
    if (result != (RUNLOOP_OK))
    {
        printf("Error in RUNLOOP_AddTask(): %d\n", result);
    }
    else
    {
        printf("Task added. Task ID: %u\n", task_id);
    }
    return;
}

/*!
*******************************************************************************
** \brief   Callback for the UART which adds the active waiting task to the
**          RUNLOOP.
**
** \param   optArgPtr   Not used.
**
*******************************************************************************
*/
static void appAddActiveWaitingTaskViaKey (void* optArgPtr)
{
    int8_t result = 0;
    uint8_t task_id = 0;
    appActiveWaitingTaskDelayMs = 1000; // active waiting time in ms
    result = RUNLOOP_AddTask(appActiveWaitingTask,
                             &appActiveWaitingTaskDelayMs,
                             1,          // number of executions
                             0,          // period in ms
                             0,          // initial delay in ms
                             &task_id);  // task id pointer
    if (result != (RUNLOOP_OK))
    {
        printf("Error in RUNLOOP_AddTask(): %d\n", result);
    }
    else
    {
        printf("Task added. Task ID: %u\n", task_id);
    }
    return;
}
#endif // RUNLOOP_WITH_CMDL


//*****************************************************************************
//*************************** PUBLIC FUNCTIONS ********************************
//*****************************************************************************

// HARDWARE RESET:
void reset(void) __attribute__((naked)) __attribute__((section(".init3")));

/*! Clear SREG_I on hardware reset, get reset source, disable the watchdog. */
void reset(void)
{
    cli();
    appMCUSR = MCUSR;
    MCUSR = 0x00;
    wdt_disable();
}

/*!
*******************************************************************************
** \brief   Main procedure.
**
**          Initializes drivers and subsystems and runs a commandline
**          interface.
**
** \param   argc    Not used.
** \param   argv    Not used.
**
** \return
**          - 0 on success.
**          - -1 on error.
**
*******************************************************************************
*/
int main(int argc, char* argv[])
{
    if(appInit()) return(-1);
    printf("\n\nInitialized. Reset source: JTRF:%d WDRF:%d BORF:%d EXTRF:%d PORF:%d\n",
                                    appMCUSR & (1 << JTRF ) ? 1 : 0,
                                    appMCUSR & (1 << WDRF ) ? 1 : 0,
                                    appMCUSR & (1 << BORF ) ? 1 : 0,
                                    appMCUSR & (1 << EXTRF) ? 1 : 0,
                                    appMCUSR & (1 << PORF ) ? 1 : 0);
    RUNLOOP_Run();
    UART_TxFlush(appUartHandle);
    return(0);
}


//*****************************************************************************
//*********************** INTERRUPT SERVICE ROUTINES **************************
//*****************************************************************************

