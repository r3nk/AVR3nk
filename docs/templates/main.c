/*!
*******************************************************************************
*******************************************************************************
** \brief   <short description>
**
**          <more detailed description>
**
** \author  <author>
**
** Copyright (C) <year> <author>
**
** This file is part of AVR3nk, available at https://github.com/r3nk/AVR3nk
**
*******************************************************************************
*******************************************************************************
*/

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <drivers/uart.h>
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

//*****************************************************************************
//**************************** LOCAL VARIABLES ********************************
//*****************************************************************************

static FILE appStdio = FDEV_SETUP_STREAM(appStdioPut, appStdioGet, _FDEV_SETUP_RW);


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
    CMDL_OptionsT cmdl_options;
    int8_t result;

    // initialize UART and STDIO:
    result = UART_Init (UART_Baud_38400,
                        UART_Parity_off,
                        UART_StopBit_1,
                        UART_CharSize_8,
                        UART_Transceive_RxTx);
    if(result != UART_OK)
    {
        return(-1);
    }

    // assign stdio:
    stdout = &appStdio;
    stdin  = &appStdio;

    // enable interrupts:
    sei();

    // initialize CMDL:
    cmdl_options.flushRxAfterExec = 1;
    cmdl_options.flushTxOnExit = 1;
    result = CMDL_Init(cmdl_options);
    if(result != CMDL_OK)
    {
        printf("CMDL could not be initialized: %d\n", result);
        return(-1);
    }

    // register commands:
    //CMDL_RegisterCommand([TODO: function name here]);

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
    UART_TxByte(chr);
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
    return((int)UART_RxByte());
}


//*****************************************************************************
//*************************** PUBLIC FUNCTIONS ********************************
//*****************************************************************************

/*!
*******************************************************************************
** \brief   Main procedure.
**
**          [TODO: give a short description of the program if necessary.]
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
    CMDL_Run();
    UART_TxFlush();
    return(0);
}

//*****************************************************************************
//*********************** INTERRUPT SERVICE ROUTINES **************************
//*****************************************************************************
