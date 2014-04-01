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

static int8_t appInit(void);
static int    appStdioPut(char chr, FILE* streamPtr);
static int    appStdioGet(FILE* streamPtr);
static int8_t appToggleLED (void* optArgPtr);
static void   appAddRunloopTaskViaKey(void* optArgPtr);
static void   appAddRunloopTaskViaCmdl(uint8_t argc, char* argv[]);


//*****************************************************************************
//**************************** LOCAL VARIABLES ********************************
//*****************************************************************************

static UART_HandleT appUartHandle = NULL;
static FILE appStdio = FDEV_SETUP_STREAM(appStdioPut, appStdioGet, _FDEV_SETUP_RW);
static uint8_t appPinIsHigh = 0;


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
static int8_t appInit(void)
{
    UART_LedParamsT led_params;
    UART_RxCallbackOptionsT cb_opts;
    CMDL_OptionsT cmdl_options;
    int8_t result;

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

    // Register UART callback that adds a task to the runloop:
    memset(&cb_opts, 0, sizeof(cb_opts));
    cb_opts.execOnRxWait = 0;
    cb_opts.writeRxToBuffer = 0;
    result = UART_RegisterRxCallback(appUartHandle,
                                     (uint8_t)'f',
                                     appAddRunloopTaskViaKey,
                                     NULL,
                                     cb_opts);
    if (result != UART_OK)
    {
        printf("UART_RegisterRxCallback: %d\n", result);
        return(result);
    }

    // Initialize CMDL:
    memset(&cmdl_options, 0, sizeof(cmdl_options));
    cmdl_options.flushRxAfterExec = 1;
    cmdl_options.flushTxOnExit = 1;
    result = CMDL_Init(appUartHandle, cmdl_options);
    if(result != CMDL_OK)
    {
        printf("CMDL_Init: %d\n", result);
        return result;
    }

    // Register commands:
    result = CMDL_RegisterCommand(appAddRunloopTaskViaCmdl,
                                  "addtask");
    if (result != CMDL_OK)
    {
        printf ("CMDL_RegisterCommand: %d\n", result);
        return result;
    }

    // Initialize RUNLOOP:
    result = RUNLOOP_Init(TIMER_TimerId_2,
                          appUartHandle);
    if (result != RUNLOOP_OK)
    {
        printf("RUNLOOP_Init: %d\n", result);
        return result;
    }

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
static int appStdioPut(char chr, FILE* streamPtr)
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
static int appStdioGet(FILE* streamPtr)
{
    return((int)UART_RxByte(appUartHandle));
}

/*!
*******************************************************************************
** \brief   Toggle the application's LEDj
**
*******************************************************************************
*/
static int8_t appToggleLED (void* optArgPtr)
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
    return 0;
}

/*!
*******************************************************************************
** \brief   Callback for the UART which adds the toggle task to the RUNLOOP.
**
** \param   optArgPtr   Not used.
**
*******************************************************************************
*/
static void appAddRunloopTaskViaKey(void* optArgPtr)
{
    int8_t result = 0;
    result = RUNLOOP_AddTask(appToggleLED,
                             NULL,
                             10,     // number of executions
                             500,    // period in ms
                             0);     // initial delay in ms
    if (result != RUNLOOP_OK)
    {
        printf("RUNLOOP_AddTask: %d\n", result);
    }
    return;
}

/*!
*******************************************************************************
** \brief   Callback for the CMDL which adds the toggle task to the RUNLOOP.
**
** \param   argc    Argument count. Should be 4.
** \param   argv    Argument vector. Should contain the command name, the
**                  number of executions of the task, the task's period in ms,
**                  and the initial delay of the task in ms.
**
*******************************************************************************
*/
static void appAddRunloopTaskViaCmdl(uint8_t argc, char* argv[])
{
    int8_t result = 0;

    if (argc != 4)
    {
        printf ("Usage: %s <numOfExec> <periodMs> <initialDelayMs>\n", argv[0]);
        return;
    }
    result = RUNLOOP_AddTask(appToggleLED,
                             NULL,
                             (uint16_t)strtoul(argv[1], NULL, 0),
                             strtoul(argv[2], NULL, 0),
                             strtoul(argv[3], NULL, 0));
    if (result != RUNLOOP_OK)
    {
        printf("RUNLOOP_AddTask: %d\n", result);
    }
    return;
}


//*****************************************************************************
//*************************** PUBLIC FUNCTIONS ********************************
//*****************************************************************************

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
    printf("Entering runloop...");
    CMDL_Run();
    UART_TxFlush(appUartHandle);
    RUNLOOP_Run();
    return(0);
}

//*****************************************************************************
//*********************** INTERRUPT SERVICE ROUTINES **************************
//*****************************************************************************

