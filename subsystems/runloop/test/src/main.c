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
#include <drivers/mcp2515.h>
#include <drivers/mcp2515_config.h>
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
static uint8_t appSendCanMessageTask (void* optArgPtr);
#if RUNLOOP_WITH_CMDL
static void    appAddToggleLedTaskViaCmdl (uint8_t argc, char* argv[]);
static void    appAddPrintUptimeTaskViaCmdl (uint8_t argc, char* argv[]);
static void    appAddActiveWaitingTaskViaCmdl (uint8_t argc, char* argv[]);
static void    appAddSendCanMessageTaskViaCmdl (uint8_t argc, char* argv[]);
#else
static void    appAddToggleLedTaskViaKey (void* optArgPtr);
static void    appAddPrintUptimeTaskViaKey (void* optArgPtr);
static void    appAddActiveWaitingTaskViaKey (void* optArgPtr);
static void    appAddSendCanMessageTaskViaKey (void* optArgPtr);
#endif


//*****************************************************************************
//**************************** LOCAL VARIABLES ********************************
//*****************************************************************************

static uint8_t appMCUSR __attribute__((section(".noinit")));
static UART_HandleT appUartHandle = NULL;
static FILE appStdio = FDEV_SETUP_STREAM(appStdioPut, appStdioGet, _FDEV_SETUP_RW);
static struct
{
    uint8_t pinIsHigh : 1;
    volatile uint8_t canTx : 1;
} appFlags;
static uint32_t appActiveWaitingTaskDelayMs = 0;
static MCP2515_InitParamsT appCanParams;
MCP2515_CanAMessageT appCanMessage;


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
    uint8_t result;
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
    result = CMDL_RegisterCommand(appAddSendCanMessageTaskViaCmdl,
                                  "can");
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
    result = UART_RegisterRxCallback(appUartHandle,
                                     (uint8_t)'c',
                                     appAddSendCanMessageTaskViaKey,
                                     NULL,
                                     uart_cb_opts);
    if (result != UART_OK)
    {
        printf("UART_RegisterRxCallback: %d\n", result);
        return(result);
    }
#endif

    // Set up CAN driver:
    memset(&appCanParams, 0, sizeof(appCanParams));
    appCanParams.initSPI = 1;
    appCanParams.wakeupLowPassFilter = 0;
    appCanParams.baudRatePrescaler = MCP2515_AUTO_BRP;
    appCanParams.synchronisationJumpWidth = MCP2515_AUTO_SJW;
    appCanParams.propagationSegmentLength = MCP2515_AUTO_PRSEG;
    appCanParams.phaseSegment1Length = MCP2515_AUTO_PHSEG1;
    appCanParams.phaseSegment2Length = MCP2515_AUTO_PHSEG2;
    appCanParams.samplePointCount = MCP2515_SAM_3;
    appCanParams.rolloverMode = MCP2515_ROLLOVER_ENABLE;
    appCanParams.oneShotMode = MCP2515_ONESHOT_DISABLE;
    appCanParams.rxBuffer0Mask = 0x000; // accept all
    appCanParams.rxBuffer1Mask = 0x000; // accept all
    result = MCP2515_Init(&appCanParams);
    if(result != MCP2515_OK)
    {
        printf("MCP2515_Init: %d\n", result);
    }

    // Set the application's LED as output:
    appFlags.pinIsHigh = 0;
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
    if (appFlags.pinIsHigh)
    {
        SET_LOW(APP_LED);
        appFlags.pinIsHigh = 0;
    }
    else
    {
        SET_HIGH(APP_LED);
        appFlags.pinIsHigh = 1;
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

/*!
*******************************************************************************
** \brief   Transmit a can message.
**
** \param   optArgPtr   Should point to the message to transmit.
**
*******************************************************************************
*/
static uint8_t appSendCanMessageTask (void* optArgPtr)
{
    MCP2515_CanAMessageT* can_msg_ptr = (MCP2515_CanAMessageT*) optArgPtr;
    MCP2515_TxParamsT can_params;
    MCP2515_TxBufferIdT buffer_id;

    if (appFlags.canTx == 0)
    {
        return (RUNLOOP_OK_TASK_ABORT);
    }

    memset(&can_params, 0, sizeof(can_params));
    can_params.bufferId = MCP2515_TX_BUFFER_0;
    can_params.priority = MCP2515_TX_PRIORITY_0;
    buffer_id = MCP2515_Transmit(can_msg_ptr, can_params);
    if(buffer_id == 0) printf("MCP2515_Transmit: No transmit buffer free.\n");
    return (RUNLOOP_OK);
}

#if RUNLOOP_WITH_CMDL
/*!
*******************************************************************************
** \brief   Callback for the CMDL which adds the LED toggle task to the RUNLOOP.
**
** \param   argc    The argument count should be 4.
** \param   argv    The argument vector is expected to contain the command name,
**                  the number of executions of the task, the task's period
**                  in ms and the initial delay of the task in ms.
**
*******************************************************************************
*/
static void appAddToggleLedTaskViaCmdl (uint8_t argc, char* argv[])
{
    uint8_t result = 0;
    uint8_t task_id = 0;

    if (argc != 4)
    {
        printf ("Usage: %s <numOfExec> <periodMs> <initialDelayMs>\n", argv[0]);
        return;
    }
    result = RUNLOOP_AddTask(&appToggleLedTask,
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
** \param   argc    The argument count should be 1 or 4.
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
    uint8_t result = 0;
    uint8_t task_id = 0;

    if (argc == 1)
    {
        appPrintUptimeTask(NULL);
    }
    else if (argc == 4)
    {
        result = RUNLOOP_AddTask(&appPrintUptimeTask,
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
** \param   argc    The argument count should be 5.
** \param   argv    The argument vector is expected to contain the command name,
**                  the number of milliseconds for active waiting, the number
**                  of executions of the task, the task's period in ms and
**                  the initial delay of the task in ms.
**
*******************************************************************************
*/
static void appAddActiveWaitingTaskViaCmdl (uint8_t argc, char* argv[])
{
    uint8_t result = 0;
    uint8_t task_id = 0;

    if (argc != 5)
    {
        printf ("Usage: %s <activeWaitingDelayMs> <numOfExec> <periodMs> <initialDelayMs>\n", argv[0]);
        return;
    }
    appActiveWaitingTaskDelayMs = strtoul(argv[1], NULL, 0);
    result = RUNLOOP_AddTask(&appActiveWaitingTask,
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

/*!
*******************************************************************************
** \brief   Callback for the CMDL which adds the CAN transmission task to the
**          RUNLOOP.
**
** \param   argc    The argument count should be 7 or greater.
** \param   argv    The argument vector is expected to contain the command name,
**                  the number of executions of the task, the task's period in
**                  ms, the initial delay of the task in ms, the CAN message
**                  ID (standard frame), the RTR bit, the data length and
**                  the data bytes.
**
*******************************************************************************
*/
static void appAddSendCanMessageTaskViaCmdl (uint8_t argc, char* argv[])
{
    uint8_t ii = 0;
    uint8_t result = 0;
    uint8_t task_id = 0;

    if (argc < 7)
    {
        printf ("Usage: %s <numOfExec> <periodMs> <initialDelayMs> <sid> <rtr> <dlc> <data>*\n", argv[0]);
        return;
    }

    appCanMessage.sid = strtoul(argv[4], NULL, 0) & 0x7FF;
    appCanMessage.rtr = strtoul(argv[5], NULL, 0) ? 1 : 0;
    appCanMessage.dlc = strtoul(argv[6], NULL, 0);
    memset(&appCanMessage.dataArray, 0, sizeof(appCanMessage.dataArray));
    for(ii = 0; ((ii < appCanMessage.dlc) && (ii < (argc - 7))); ii++)
    {
        appCanMessage.dataArray[ii] = strtoul(argv[ii+7], NULL, 0) & 0xFF;
    }
    result = RUNLOOP_AddTask(&appSendCanMessageTask,
                             (void*)&appCanMessage,
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
    appFlags.canTx = 1;
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
    uint8_t result = 0;
    uint8_t task_id = 0;
    result = RUNLOOP_AddTask(&appToggleLedTask,
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
    uint8_t result = 0;
    uint8_t task_id = 0;
    result = RUNLOOP_AddTask(&appPrintUptimeTask,
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
    uint8_t result = 0;
    uint8_t task_id = 0;
    appActiveWaitingTaskDelayMs = 1000; // active waiting time in ms
    result = RUNLOOP_AddTask(&appActiveWaitingTask,
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

/*!
*******************************************************************************
** \brief   Callback for the CMDL which adds the CAN transmission task to the
**          RUNLOOP.
**
** \param   optArgPtr   Not used.
**
*******************************************************************************
*/
static void appAddSendCanMessageTaskViaKey (void* optArgPtr)
{
    uint8_t result = 0;
    uint8_t task_id = 0;

    appCanMessage.sid = 0x84;
    appCanMessage.rtr = 0;
    appCanMessage.dlc = 2;
    appCanMessage.dataArray[0] = 0x12;
    appCanMessage.dataArray[1] = 0x23;

    if (appFlags.canTx)
    {
        // Let the CAN task terminate if already running:
        appFlags.canTx = 0;
        return;
    }
    result = RUNLOOP_AddTask(&appSendCanMessageTask,
                             (void*)&appCanMessage,
                             0,   // infinite execution
                             100, // 100 ms interval
                             0,   // no initial delay
                             &task_id);
    if (result != (RUNLOOP_OK))
    {
        printf("Error in RUNLOOP_AddTask(): %d\n", result);
    }
    else
    {
        printf("Task added. Task ID: %u\n", task_id);
    }
    appFlags.canTx = 1;
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

