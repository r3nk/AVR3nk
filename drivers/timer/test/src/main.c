/*!
*******************************************************************************
*******************************************************************************
** \brief   Test set for the TIMER driver.
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
#include <drivers/uart.h>
#include <drivers/timer.h>
#include <subsystems/cmdl.h>


//*****************************************************************************
//*************************** DEFINES AND MACROS ******************************
//*****************************************************************************


//*****************************************************************************
//**************************** LOCAL DATA TYPES *******************************
//*****************************************************************************


//*****************************************************************************
//********************** LOCAL FUNCTION DECLARATIONS **************************
//*****************************************************************************

static int8_t appInit(void);
static int    appStdioPut(char chr, FILE* streamPtr);
static int    appStdioGet(FILE* streamPtr);
static void   appCmdlExec(void* optArgPtr);
static void   appCmdlStop(uint8_t argc, char* argv[]);
static void   appTimerFinishedCallback(void* optArg);
static void   appTestOneShot(uint8_t argc, char* argv[]);
static void   appTestCountdown(uint8_t argc, char* argv[]);


//*****************************************************************************
//**************************** LOCAL VARIABLES ********************************
//*****************************************************************************

static UART_HandleT appUartHandle = NULL;
static FILE appStdio = FDEV_SETUP_STREAM(appStdioPut, appStdioGet, _FDEV_SETUP_RW);
static TIMER_HandleT appTimerHandle = NULL;
static struct
{
    volatile uint8_t cmdlRunning : 1;
    volatile uint8_t cmdlExec : 1;
} appFlags;


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
**          - 1 on error.
**
*******************************************************************************
*/
static int8_t appInit(void)
{
    UART_LedParamsT led_params;
    CMDL_OptionsT cmdl_options;
    uint8_t result;

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
        return(1);
    }

    // Assign stdio:
    stdout = &appStdio;
    stdin  = &appStdio;

    // Enable interrupts:
    sei();

    // Initialize CMDL:
    memset(&cmdl_options, 0, sizeof(cmdl_options));
    cmdl_options.flushRxAfterExec = 1;
    result = CMDL_Init(appUartHandle, &appCmdlExec, cmdl_options);
    if(result != CMDL_OK)
    {
        printf("CMDL_Init: %d\n", result);
        return(1);
    }

    // Register commands:
    CMDL_RegisterCommand(appCmdlStop, "exit");
    CMDL_RegisterCommand(appTestOneShot, "oneshot");
    CMDL_RegisterCommand(appTestCountdown, "countdown");

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
** \brief   Trigger commandline execution.
**
** \param   optArgPtr   Not used. Satisfies the interface as a UART callback.
**
*******************************************************************************
*/
static void appCmdlExec(void* optArgPtr)
{
    appFlags.cmdlExec = 1;
    return;
}

/*!
*******************************************************************************
** \brief   Stop the commandline.
**
** \param   argc    Not used.
** \param   argv    Not used.
**
*******************************************************************************
*/
static void appCmdlStop(uint8_t argc, char* argv[])
{
    appFlags.cmdlRunning = 0;
    return;
}

/*!
*******************************************************************************
** \brief   A callback that is triggered by the timer overflow.
**
** \param   optArg  Will be interpreted as a uint8_t* flag that will be written
**                  to indicate that the overflow has occured.
**
*******************************************************************************
*/
static void appTimerFinishedCallback(void* optArg)
{
    uint8_t* finished_ptr = (uint8_t*)optArg;
    *finished_ptr = 1;
    return;
}

/*!
*******************************************************************************
** \brief   Test the TIMERs one-shot feature.
**
**          This function runs TIMER_OneShot() for several prescalers and
**          prints the number of total (measured) elapsed system clock cycles
**          from start to stop.
**
** \param   argc    Argument count.
** \param   argv    Argument vector. Pass a number to identify a timer.
**
*******************************************************************************
*/
static void appTestOneShot(uint8_t argc, char* argv[])
{
    uint8_t result = 0;
    uint8_t timer_num;
    volatile uint8_t finished = 0;
    uint32_t clock_cycles = 0;
    TIMER_TimerIdT timer_id;
    TIMER_ClockPrescalerT timer_clock_prescaler;
    TIMER_WaveGenerationT timer_wave_generation_mode;
    TIMER_OutputModeT timer_output_mode;

    if (argc != 2)
    {
        printf("Usage: %s <timerId>\n", argv[0]);
        return;
    }

    timer_num = strtoul(argv[1], NULL, 0);
    switch(timer_num)
    {
        case 2:
            timer_id = TIMER_TimerId_2;
            break;
        case 1:
            timer_id = TIMER_TimerId_1;
            break;
        case 0:
        default:
            timer_id = TIMER_TimerId_0;
            break;
    }

    // Initialize TIMER:
    timer_clock_prescaler = TIMER_ClockPrescaler_1;
    timer_wave_generation_mode = TIMER_WaveGeneration_NormalMode;
    timer_output_mode = TIMER_OutputMode_NormalPortOperation;
    appTimerHandle = TIMER_Init (timer_id,
                                 timer_clock_prescaler,
                                 timer_wave_generation_mode,
                                 timer_output_mode,
                                 timer_output_mode);

    if (appTimerHandle == NULL)
    {
        printf ("Error during TIMER_Init().\n");
    }

    result = TIMER_SetOverflowCallback(appTimerHandle,
                                       appTimerFinishedCallback,
                                       (void*)&finished,
                                       0);
    if (result)
    {
        printf ("Error during TIMER_SetOverflowCallback().\n");
    }

    TIMER_EnableDisableStopwatch(appTimerHandle, TIMER_Stopwatch_Enable);
    TIMER_SetClockPrescaler(appTimerHandle, TIMER_ClockPrescaler_1024);
    TIMER_OneShot(appTimerHandle);
    finished = 0;
    while(!finished);
    TIMER_GetStopwatchSystemClockCycles(appTimerHandle, &clock_cycles, TIMER_Stopwatch_Reset);
    printf("OneShot [1024]: %lu\n", clock_cycles);

    TIMER_SetClockPrescaler(appTimerHandle, TIMER_ClockPrescaler_256);
    TIMER_OneShot(appTimerHandle);
    finished = 0;
    while(!finished);
    TIMER_GetStopwatchSystemClockCycles(appTimerHandle, &clock_cycles, TIMER_Stopwatch_Reset);
    printf("OneShot  [256]: %lu\n", clock_cycles);

    TIMER_SetClockPrescaler(appTimerHandle, TIMER_ClockPrescaler_64);
    TIMER_OneShot(appTimerHandle);
    finished = 0;
    while(!finished);
    TIMER_GetStopwatchSystemClockCycles(appTimerHandle, &clock_cycles, TIMER_Stopwatch_Reset);
    printf("OneShot   [64]: %lu\n", clock_cycles);

    TIMER_SetClockPrescaler(appTimerHandle, TIMER_ClockPrescaler_8);
    TIMER_OneShot(appTimerHandle);
    finished = 0;
    while(!finished);
    TIMER_GetStopwatchSystemClockCycles(appTimerHandle, &clock_cycles, TIMER_Stopwatch_Reset);
    printf("OneShot    [8]: %lu\n", clock_cycles);

    TIMER_SetClockPrescaler(appTimerHandle, TIMER_ClockPrescaler_1);
    TIMER_OneShot(appTimerHandle);
    finished = 0;
    while(!finished);
    TIMER_GetStopwatchSystemClockCycles(appTimerHandle, &clock_cycles, TIMER_Stopwatch_Reset);
    printf("OneShot    [1]: %lu\n", clock_cycles);

    TIMER_Exit(appTimerHandle);
    return;
}

/*!
*******************************************************************************
** \brief   Test the TIMERs countdown feature.
**
** \param   argc    Argument count.
** \param   argv    Argument vector. Pass a number to identify a timer and
**                  another number to specify the number of milliseconds
**                  for the countdown.
**
*******************************************************************************
*/
static void appTestCountdown(uint8_t argc, char* argv[])
{
    uint8_t timer_num;
    uint32_t milliseconds;
    volatile uint8_t finished = 0;
    TIMER_TimerIdT timer_id;
    TIMER_ClockPrescalerT timer_clock_prescaler;
    TIMER_WaveGenerationT timer_wave_generation_mode;
    TIMER_OutputModeT timer_output_mode;

    if (argc != 3)
    {
        printf("Usage: %s <timerId> <milliseconds>\n", argv[0]);
        return;
    }

    timer_num = strtoul(argv[1], NULL, 0);
    switch(timer_num)
    {
        case 2:
            timer_id = TIMER_TimerId_2;
            break;
        case 1:
            timer_id = TIMER_TimerId_1;
            break;
        case 0:
        default:
            timer_id = TIMER_TimerId_0;
            break;
    }

    milliseconds = strtoul(argv[2], NULL, 0);

    // Initialize TIMER:
    timer_clock_prescaler = TIMER_ClockPrescaler_1;
    timer_wave_generation_mode = TIMER_WaveGeneration_NormalMode;
    timer_output_mode = TIMER_OutputMode_NormalPortOperation;
    appTimerHandle = TIMER_Init (timer_id,
                                 timer_clock_prescaler,
                                 timer_wave_generation_mode,
                                 timer_output_mode,
                                 timer_output_mode);

    if (appTimerHandle == NULL)
    {
        printf ("Error during TIMER_Init().\n");
    }

    TIMER_StartCountdown (appTimerHandle,
                          appTimerFinishedCallback,
                          (void*)&finished,
                          milliseconds,
                          1);
    printf("Counting down, please wait... ");
    while(!finished);
    printf("finished.\n");

    TIMER_Exit(appTimerHandle);
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
**          - 1 on error.
**
*******************************************************************************
*/
int main(int argc, char* argv[])
{
    if(appInit()) return(1);

    // Init CMDL state:
    appFlags.cmdlRunning = 1;
    appFlags.cmdlExec = 0;

    // Print the prompt:
    CMDL_PrintPrompt(NULL);

    // Wait for commands:
    while(appFlags.cmdlRunning)
    {
        if(appFlags.cmdlExec) // start execution
        {
            CMDL_Execute();
            CMDL_PrintPrompt(NULL);
            appFlags.cmdlExec = 0;
        }
    }
    UART_TxFlush(appUartHandle);
    return(0);
}

//*****************************************************************************
//*********************** INTERRUPT SERVICE ROUTINES **************************
//*****************************************************************************

