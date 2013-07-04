/*!
*******************************************************************************
*******************************************************************************
** \brief     Demo application: Buffered UART command line interface.
**
** \author    Robin Klose
**
** Copyright (C) 2009-2013 Robin Klose
**
** This file is part of AVR3nk, available at https://github.com/r3nk/AVR3nk
**
*******************************************************************************
*******************************************************************************
*/

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h> // needed for strtol / strtoul
#include <avr/interrupt.h>
#include <drivers/uart.h>
#include <subsystems/cmdl.h>

//*****************************************************************************
//*************************** DEFINES AND MACROS ******************************
//*****************************************************************************


//*****************************************************************************
//********************** LOCAL FUNCTION DECLARATIONS **************************
//*****************************************************************************

static int8_t appInit(void);
static int    appStdioPut(char chr, FILE* streamPtr);
static int    appStdioGet(FILE* streamPtr);
static void   appList(uint8_t argc, char* argv[]);
static void   appMultiply(uint8_t argc, char* argv[]);
static void   appPrintFloat(uint8_t argc, char* argv[]);

//*****************************************************************************
//****************************** LOCAL DATA ***********************************
//*****************************************************************************

static UART_HandleT appUartHandle = NULL;
static FILE appStdio = FDEV_SETUP_STREAM(appStdioPut, appStdioGet, _FDEV_SETUP_RW);

//*****************************************************************************
//*************************** PUBLIC FUNCTIONS ********************************
//*****************************************************************************

// HARDWARE RESET:
void reset(void) __attribute__((naked)) __attribute__((section(".init3")));

/*! Clear SREG_I on hardware reset. */
void reset(void)
{
    cli();
}

int main(int argc, char* argv[])
{
    if(appInit()) return(-1);
    printf("\n\n");
    printf("**********************************************\n");
    printf(" Demo Application: Command Line Interface\n");
    printf(" Author: Robin Klose\n");
    printf("**********************************************\n");
    CMDL_Run();
    printf("\n\nExiting...\n");
    UART_TxFlush(appUartHandle);
    return(0);
}

//*****************************************************************************
//*************************** LOCAL FUNCTIONS *********************************
//*****************************************************************************

static int8_t appInit(void)
{
    UART_LedParamsT led_params;
    CMDL_OptionsT cmdl_options;
    int8_t result;

    // initialize UART and STDIO:
    memset(&led_params, 0, sizeof(led_params));
    led_params.txLedPortPtr = &PORTA;
    led_params.txLedDdrPtr  = &DDRA;
    led_params.txLedIdx     = 6;
    led_params.rxLedPortPtr = &PORTA;
    led_params.rxLedDdrPtr  = &DDRA;
    led_params.rxLedIdx     = 7;

    appUartHandle = UART_Init(0,
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

    // assign stdio:
    stdout = &appStdio;
    stdin  = &appStdio;
    sei();

    // initialze CMDL:
    cmdl_options.flushRxAfterExec = 1;
    cmdl_options.flushTxOnExit = 1;
    result = CMDL_Init(appUartHandle, cmdl_options);
    if(result != CMDL_OK)
    {
        printf("CMDL could not be initialized: %d\n", result);
        return(-1);
    }

    // register commands:
    CMDL_RegisterCommand(&appList,
                         "list");
    CMDL_RegisterCommand(&appMultiply,
                         "multiply");
    CMDL_RegisterCommand(&appPrintFloat,
                         "float");
    return(0);
}

static int appStdioPut(char chr, FILE* streamPtr)
{
    UART_TxByte(appUartHandle, chr);
    return(0);
}

static int appStdioGet(FILE* streamPtr)
{
    return((int)UART_RxByte(appUartHandle));
}

static void appList(uint8_t argc, char* argv[])
{
    uint8_t ii;

    printf("[test] argc = %d\n", argc);
    for (ii = 0; ii < argc; ii++)
    {
        printf("[test] argv[%d] = %s\n", ii, argv[ii]);
    }
    return;
}

static void appMultiply(uint8_t argc, char* argv[])
{
    int32_t n1;
    int32_t n2;
    int32_t n3;

    if(argc < 3)
    {
        printf("Too few arguments: %d\n", argc-1);
        return;
    }
    n1 = strtol (argv[1], NULL, 10);
    n2 = strtol (argv[2], NULL, 10);
    n3 = n1 * n2;
    printf("%li * %li = %li\n", n1, n2, n3);
    return;
}

static void appPrintFloat(uint8_t argc, char* argv[])
{
    double ff;

    if(argc != 2)
    {
        printf("argc = %d != 2\n", argc);
        return;
    }
    ff = strtod(argv[1], NULL);
    printf("float = %lf\n", ff);
    return;
}
