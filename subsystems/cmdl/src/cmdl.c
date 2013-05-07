/*!
*******************************************************************************
*******************************************************************************
** \brief   Buffered command line for AVR microcontrollers
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

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <avr/version.h>
#include <drivers/uart.h>
#include "cmdl.h"

//*****************************************************************************
//*************************** DEFINES AND MACROS ******************************
//*****************************************************************************


//*****************************************************************************
//**************************** LOCAL DATATYPES ********************************
//*****************************************************************************

/*! Structure of a command that can be registered with CMDL. */
typedef struct
{
    /*! Points to the function to execute. */
    void (*funcPtr) (uint8_t argc, char* argv[]);
    /*! Command name, actually causing the command to execute. */
    char* namePtr;
#if CMDL_USAGE_STRING_SUPPORT
    /*! Usage string. */
    char* usagePtr;
#endif // CMDL_USAGE_STRING_SUPPORT
} cmdlCommandT;

//*****************************************************************************
//********************** LOCAL FUNCTION DECLARATIONS **************************
//*****************************************************************************

static void cmdlCallbackExec(void* optArgPtr);
static void cmdlStop(uint8_t argc, char* argv[]);
static void cmdlHelp(uint8_t argc, char* argv[]);
static cmdlCommandT* cmdlFindCommand(char* cmdNamePtr);

//#if ( ( __AVR_LIBC_MAJOR__ < 1 ) || ( __AVR_LIBC_MINOR__ < 6 ) || ( __AVR_LIBC_REVISION__ < 3 ) )
#if ( __AVR_LIBC_VERSION__ < 10603UL )
char* strtok (char* s, const char* delim);
#endif // ( __AVR_LIBC_VERSION__ < 10603UL )

//*****************************************************************************
//**************************** LOCAL VARIABLES ********************************
//*****************************************************************************

/*! State variable keeps track of CMDL state parameters. */
volatile static struct
{
    uint8_t initialized : 1; //<! Indicates whether CMDL has been initialized
    uint8_t running : 1; //<! Indicates whether the command line is running.
    uint8_t exec    : 1; //<! Indicates whether a program is in execution.
} cmdlState;

/*! Options defined by the user. */
static CMDL_OptionsT cmdlOptions;

/*! The prompt string with a leading newline. */
static char* cmdlPromptStr = "\n" CMDL_PROMPT;

/*! String that contains the command string. */
static char cmdlCmdString[CMDL_MAX_COMMAND_LENGTH + 1];

/*! Array of pointers to all registered cmdlCommandT structures. */
static cmdlCommandT cmdlCmdArray[CMDL_MAX_COMMAND_COUNT];

//*****************************************************************************
//**************************** LOCAL FUNCTIONS ********************************
//*****************************************************************************

/*!
*******************************************************************************
** \brief   Callback that makes the CMDL execute the command in the UART
**          rx buffer.
**
*******************************************************************************
*/
static void cmdlCallbackExec(void* optArgPtr)
{
    cmdlState.exec = 1;
    return;
}

/*!
*******************************************************************************
** \brief   Makes CMDL stop commandline execution.
**
** \param   argc    not used here
** \param   argv    not used here
**
*******************************************************************************
*/
static void cmdlStop(uint8_t argc, char* argv[])
{
    cmdlState.running = 0;
    return;
}

/*!
*******************************************************************************
** \brief   List all registered commands or display help for a specific command.
**
** \param   argc    If <= 1, all registered commands are listed, otherwise
**                  the usage string for the command specified by argv[1] will
**                  be displayed.
** \param   argv    may contain a command name
**
*******************************************************************************
*/
static void cmdlHelp(uint8_t argc, char* argv[])
{
    uint8_t ii;

#if CMDL_USAGE_STRING_SUPPORT

    cmdlCommandT* command_ptr;

    if(argc <= 1) // list all commands
    {
        printf(CMDL_LABEL "Registered commands:\n");
        for(ii = 0; ii < CMDL_MAX_COMMAND_COUNT; ii++)
        {
            if(cmdlCmdArray[ii].namePtr)
            {
                printf(CMDL_LABEL "%s\n", cmdlCmdArray[ii].namePtr);
            }
        }
        printf(CMDL_LABEL "for detailed information type \"help <command>\"\n");
    }
    else // specify a single command
    {
        command_ptr = cmdlFindCommand(argv[1]);
        if(command_ptr == NULL)
        {
            printf(CMDL_LABEL "unknown command: %s\n", argv[1]);
        }
        else
        {
            printf(CMDL_LABEL "usage: %s\n", command_ptr->usagePtr);
        }
    }
#else // CMDL_USAGE_STRING_SUPPORT
    printf(CMDL_LABEL "Registered commands:\n");
    for(ii = 0; ii < CMDL_MAX_COMMAND_COUNT; ii++)
    {
        if(cmdlCmdArray[ii].namePtr)
        {
            printf(CMDL_LABEL "%s\n", cmdlCmdArray[ii].namePtr);
        }
    }
#endif // CMDL_USAGE_STRING_SUPPORT
    return;
}

/*!
*******************************************************************************
** \brief   Lookup a command in the list of registered command by its command
**          name.
**
** \param   cmdNamePtr  The name of the command to look up.
**
** \return
**          - Pointer to a cmdlCommandT that is registered with the name
**            indicated by cmdNamePtr.
**          - NULL if the cmdNamePtr argument is NULL or if there is no command
**            registered that matches cmdNamePtr.
**
*******************************************************************************
*/
static cmdlCommandT* cmdlFindCommand(char* cmdNamePtr)
{
    uint8_t ii;

    if(cmdNamePtr == NULL)
    {
        return(NULL);
    }
    for(ii = 0; ii < CMDL_MAX_COMMAND_COUNT; ii++)
    {
        if(cmdlCmdArray[ii].namePtr
        && (strcmp(cmdNamePtr, cmdlCmdArray[ii].namePtr) == 0))
        {
            return(&cmdlCmdArray[ii]); // command found
        }
    }
    return (NULL); // command not in list
}

#if CMDL_DEBUG
void cmdlDumpCmdString(void)
{
    uint8_t ii;
    printf(CMDL_LABEL_DEBUG "Dumping cmdlCmdString... \n");
    for(ii = 0; ii < CMDL_MAX_COMMAND_LENGTH + 1; ii++)
    {
        printf("0x%02X ", cmdlCmdString[ii]);
    }
    printf("\n");
    return;
}
#endif // CMDL_DEBUG

//*****************************************************************************
//*************************** PUBLIC FUNCTIONS ********************************
//*****************************************************************************

/*!
*******************************************************************************
** \brief   Initialize the CMDL commandline.
**
**          Initializes local state and registers two needed rx callbacks with
**          the UART driver (backspace, newline).
**          The UART must be initialized before the CMDL. Choose
**          UART_CharSize_8 and UART_Transceive_RxTx for UART initialization!
**
** \param   options     Options affecting the CMDL behaviour.
**
** \return
**          - #CMDL_OK on success.
**          - #CMDL_ERR_UART_NOT_INITIALIZED if the UART is not initialized.
**          - #CMDL_ERR_UART_NOT_OK if the callback functions could not be
**            registered.
**          - #CMDL_ERR_NO_COMMAND_SLOT if there are no command slots
**            available to register the fundamental \exit command.
**
*******************************************************************************
*/
int8_t CMDL_Init(CMDL_OptionsT options)
{
    int8_t ret_code;
    UART_CallbackRxOptionsT callbackOptions;

    if(UART_IsInitialized() == 0)
    {
        return(CMDL_ERR_UART_NOT_INITIALIZED);
    }
    // erase commands:
    memset(cmdlCmdArray, 0, sizeof(cmdlCmdArray));

    // register backspace callback:
    callbackOptions.execOnRxWait = 0;
    callbackOptions.writeRxToBuffer = 0;
    ret_code = UART_CallbackRxRegister(0x08, // backspace
                                       UART_CallbackRxBackspace,
                                       NULL, callbackOptions);
    if (ret_code != UART_OK)
    {
        return (CMDL_ERR_UART_NOT_OK);
    }
    // register execute callback:
    callbackOptions.execOnRxWait = 0;
    callbackOptions.writeRxToBuffer = 0;
    ret_code = UART_CallbackRxRegister(0x0D, // carriage return
                                       cmdlCallbackExec,
                                       NULL, callbackOptions);
    if (ret_code != UART_OK)
    {
        return (CMDL_ERR_UART_NOT_OK);
    }

    // register internal \c exit command:
#if CMDL_USAGE_STRING_SUPPORT
    ret_code = CMDL_RegisterCommand(&cmdlStop,
                                    "exit",
                                    "exit");
#else // #if CMDL_USAGE_STRING_SUPPORT
    ret_code = CMDL_RegisterCommand(&cmdlStop,
                                    "exit");
#endif // #if CMDL_USAGE_STRING_SUPPORT
    if (ret_code != CMDL_OK)
    {
        return (ret_code);
    }

    // register internal \c help command:
#if CMDL_USAGE_STRING_SUPPORT
    ret_code = CMDL_RegisterCommand(&cmdlHelp,
                                    "help",
                                    "help [commandName]");
#else // #if CMDL_USAGE_STRING_SUPPORT
    ret_code = CMDL_RegisterCommand(&cmdlHelp,
                                    "help");
#endif // #if CMDL_USAGE_STRING_SUPPORT
    if (ret_code != CMDL_OK)
    {
        return (ret_code);
    }

    // apply options and state:
    cmdlOptions = options;
    cmdlState.running     = 0;
    cmdlState.exec        = 0;
    cmdlState.initialized = 1;
    return(CMDL_OK);
}

/*!
*******************************************************************************
** \brief   Test whether CMDL is initialized.
**
*******************************************************************************
*/
int8_t CMDL_IsInitialized(void)
{
    return(cmdlState.initialized);
}

/*!
*******************************************************************************
** \brief   Register a new command with the CMDL.
**
** \param   funcPtr Points to the function to execute when the command is called.
** \param   namePtr The string on which the command is registered. If you type
**                  this string in the command line, the command will execute.
** \param   usagePtr
**                  This parameter is active if the CMDL_USAGE_STRING_SUPPORT
**                  switch in cmdl.h is activated.
**                  Provide some additional information how to use the command,
**                  such as "foo <bar> [optArg]" for the command foo with
**                  mandatory argument bar and optional argument optArg.
**
** \return
**          - #CMDL_OK on success.
**          - #CMDL_ERR_BAD_PARAMETER if one of the passed pointer arguments
**            is NULL.
**          - #CMDL_ERR_NO_COMMAND_SLOT if there is no free slot available
**            to register a new command. Increase #CMDL_MAX_COMMAND_COUNT
**            in cmdl.h and recompile!
**
*******************************************************************************
*/
#if CMDL_USAGE_STRING_SUPPORT
int8_t CMDL_RegisterCommand(void (*funcPtr) (uint8_t argc, char* argv[]),
                            char* namePtr,
                            char* usagePtr)
#else //CMDL_USAGE_STRING_SUPPORT
int8_t CMDL_RegisterCommand(void (*funcPtr) (uint8_t argc, char* argv[]),
                            char* namePtr)
#endif // CMDL_USAGE_STRING_SUPPORT
{
    uint8_t ii;
#if CMDL_USAGE_STRING_SUPPORT
    if((funcPtr == NULL) || (namePtr == NULL) || (usagePtr == NULL))
#else // CMDL_USAGE_STRING_SUPPORT
    if((funcPtr == NULL) || (namePtr == NULL))
#endif // CMDL_USAGE_STRING_SUPPORT
    {
        return(CMDL_ERR_BAD_PARAMETER);
    }
    for(ii = 0; ii < CMDL_MAX_COMMAND_COUNT; ii++)
    {
        if(cmdlCmdArray[ii].namePtr == NULL)
        {
#if CMDL_DEBUG
            printf(CMDL_LABEL "registering command: %s\n", name);
#endif // CMDL_DEBUG
            cmdlCmdArray[ii].funcPtr = funcPtr;
            cmdlCmdArray[ii].namePtr = namePtr;
#if CMDL_USAGE_STRING_SUPPORT
            cmdlCmdArray[ii].usagePtr = usagePtr;
#endif // CMDL_USAGE_STRING_SUPPORT
            return (CMDL_OK);
        }
    }
    return (CMDL_ERR_NO_COMMAND_SLOT);
}

/*!
*******************************************************************************
** \brief   Run the commandline.
**
**          The command line prompt appears and is waiting for commands.
**          In order to exit the command line, type \c exit.
*******************************************************************************
*/
void CMDL_Run(void)
{
    // points to arguments passed, 1 argument for argv[0] as command name string:
    char*    argv[CMDL_MAX_ARGUMENT_COUNT + 1];
    uint8_t  argc; // count of parameters, including command name string
    uint8_t  cmd_len; // length of the received command string
    char*    tok_ptr; // points to received tokens
    cmdlCommandT* command_ptr; // points to the command to execute

    // init local state:
    cmdlState.running = 1;
    cmdlState.exec = 0;

    // print the prompt:
    //UART_TxField((uint8_t*)cmdlPromptStr, strlen(cmdlPromptStr));
    printf(cmdlPromptStr);

    // discard the rx buffer:
    UART_RxDiscard();

    while(cmdlState.running)
    {
        if(cmdlState.exec) // start execution
        {
            // read the command from rx buffer into CMDL's command buffer:
            memset(cmdlCmdString, 0, sizeof(cmdlCmdString));
            cmd_len = UART_RxField((uint8_t*)cmdlCmdString, CMDL_MAX_COMMAND_LENGTH);
            cmdlCmdString[cmd_len] = '\0'; // terminate string

            // parse the string into tokens:
            argc = 0;
            memset(argv, 0, sizeof(argv));
            tok_ptr = strtok (cmdlCmdString, " ");
            while((tok_ptr != NULL) && (argc <= CMDL_MAX_ARGUMENT_COUNT))
            {
                argv[argc++] = tok_ptr;
                tok_ptr = strtok (NULL, " ");
            }
#if CMDL_DEBUG
            cmdlDumpCmdString();
#endif // CMDL_DEBUG
            command_ptr = cmdlFindCommand(argv[0]);
            if(command_ptr != NULL)
            {
                // execute the command:
                command_ptr->funcPtr(argc, argv);
            }
            else if (argc) // print only if anything typed:
            {
                printf(CMDL_LABEL "unknown command: %s\n", argv[0]);
            }

            // print the prompt:
            //UART_TxField((uint8_t*)cmdlPromptStr, strlen(cmdlPromptStr));
            if(cmdlState.running)
            {
                printf(cmdlPromptStr);
            }
            if(cmdlOptions.flushRxAfterExec)
            {
                // Discard the receive buffer before next execution starts:
                UART_RxDiscard();
            }
            cmdlState.exec = 0;
        }
    }
    // flush the tx buffer:
    if(cmdlOptions.flushTxOnExit)
    {
        UART_TxFlush();
    }
    return;
}

#if ( __AVR_LIBC_VERSION__ < 10603UL )
/*!
*******************************************************************************
** \brief   This is a implementation of the strtok function which is available
**          in avr-libc from version 1.06.03 on. If the installed library's 
**          version is older, this implementation will be used.
**          This procedure is NOT reentrant.
*******************************************************************************
*/
char* strtok (char* s, const char* delim)
{
    static char* currPosPtr = NULL;
    static const char* currDelimPtr = NULL;
    auto char* retPtr = NULL;

    if(s) currPosPtr = s;
    if(delim) currDelimPtr = delim;
    if((currPosPtr == NULL) || (currDelimPtr == NULL)) return(NULL);

    // remove trailing delimiters:
    while(*currPosPtr == *currDelimPtr)
    {
        *currPosPtr++ = '\0';
    }
    if(*currPosPtr == '\0')
    {
        currPosPtr = NULL;
        return(NULL);
    }
    retPtr = currPosPtr;

    // get the next token:
    while(*currPosPtr != *currDelimPtr)
    {
        if(*currPosPtr == '\0')
        {
            currPosPtr = NULL;
            return(retPtr);
        }
        currPosPtr++;
    }

    // remove trailing delimiters:
    while(*currPosPtr == *currDelimPtr)
    {
        if(*currPosPtr == '\0')
        {
            currPosPtr = NULL;
            return(retPtr);
        }
        *currPosPtr++ = '\0';
    }

    return(retPtr);
}
#endif // ( __AVR_LIBC_VERSION__ < 10603UL )
