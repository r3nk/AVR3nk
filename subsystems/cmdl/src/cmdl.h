/*!
*******************************************************************************
*******************************************************************************
** \brief   CMDL commandline interface declarations
**
** \author  Robin Klose
**
** Copyright (C) 2009-2013 Robin Klose
**
** This file is part of AVR3nk, available at https://github.com/r3nk/AVR3nk
**
*******************************************************************************
*******************************************************************************
*/

#ifndef CMDL_H
#define CMDL_H

#include <stdint.h>
#include <drivers/uart.h>

//*****************************************************************************
//*************************** DEFINES AND MACROS ******************************
//*****************************************************************************

// Defines the count of bytes that can be recognized for a command invocation.
#ifndef CMDL_MAX_COMMAND_LENGTH
#define CMDL_MAX_COMMAND_LENGTH         64
#endif

// Count of commands that can be registered.
#ifndef CMDL_MAX_COMMAND_COUNT
#define CMDL_MAX_COMMAND_COUNT          12
#endif

// Maximal count of arguments that can be recognized for a command.
#ifndef CMDL_MAX_ARGUMENT_COUNT
#define CMDL_MAX_ARGUMENT_COUNT         12
#endif

// Enable/disable debug messages.
#ifndef CMDL_DEBUG
#define CMDL_DEBUG                      0
#endif

// Decides whether a usage string can be registered with each command.
#ifndef CMDL_USAGE_STRING_SUPPORT
#define CMDL_USAGE_STRING_SUPPORT       0
#endif

#define CMDL_PROMPT                     "AVR > "

#define CMDL_LABEL                      "[CMDL] "

#define CMDL_LABEL_DEBUG                "[CMDL/dbg] "

//*****************************************************************************
//************************* CMDL SPECIFIC ERROR CODES *************************
//*****************************************************************************

/*! CMDL specific error base */
#define CMDL_ERR_BASE                   0

/*! CMDL returns with no errors. */
#define CMDL_OK                         0

/*! A bad parameter has been passed. */
#define CMDL_ERR_BAD_PARAMETER          CMDL_ERR_BASE - 1

/*! The UART driver is not initialized. */
#define CMDL_ERR_UART_NOT_INITIALIZED   CMDL_ERR_BASE - 2

/*! Some UART specific function did not return with UART_OK. */
#define CMDL_ERR_UART_NOT_OK            CMDL_ERR_BASE - 3

/*! There is no command slot left for registering new commands. */
#define CMDL_ERR_NO_COMMAND_SLOT        CMDL_ERR_BASE - 4

//*****************************************************************************
//******************************** DATA TYPES *********************************
//*****************************************************************************

/*! User options for the command line. */
typedef struct
{
    /*! Decides whether to flush the UART rx buffer after program execution.
    **  This is useful if you want characters that you typed on the keyboard
    **  while a program was still in execution to be discarded. Might be
    **  helpful for hasty hackers who can't wait typing the next command.
    */
    uint8_t flushRxAfterExec : 1;

    /*! Set this option to 1 if you want to wait until all characters left in
    **  the tx buffer are transmitted before exiting (recommended). If you
    **  select 0, transmission may be aborted.
    */
    uint8_t flushTxOnExit : 1;
} CMDL_OptionsT;

//*****************************************************************************
//************************* FUNCTION DECLARATIONS *****************************
//*****************************************************************************

int8_t CMDL_Init (UART_HandleT uartHandle, CMDL_OptionsT options);
int8_t CMDL_IsInitialized (void);
#if CMDL_USAGE_STRING_SUPPORT
int8_t CMDL_RegisterCommand (void (*funcPtr) (uint8_t argc, char* argv[]),
                             char* namePtr,
                             char* usagePtr);
#else // CMDL_USAGE_STRING_SUPPORT
int8_t CMDL_RegisterCommand (void (*funcPtr) (uint8_t argc, char* argv[]),
                             char* namePtr);
#endif // CMDL_USAGE_STRING_SUPPORT
void   CMDL_Run (void);


#endif // CMDL_H
