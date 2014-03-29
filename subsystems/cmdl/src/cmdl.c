/*!
*******************************************************************************
*******************************************************************************
** \brief   Buffered command line for AVR microcontrollers
**
** \author  Robin Klose
**
** Copyright (C) 2009-2014 Robin Klose
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

static void cmdlHelp(uint8_t argc, char* argv[]);
static cmdlCommandT* cmdlFindCommand(char* cmdNamePtr);

#if CMDL_DEBUG
static void cmdlDumpCmdString(void);
#endif

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
    uint8_t initialized      : 1; //<! Indicates whether CMDL has been initialized
    uint8_t flushRxAfterExec : 1; //<! see description of CMDL_OptionsT
} cmdlState;

/*! The associated UART handle. */
static UART_HandleT cmdlUartHandle;

/*! The prompt string with a leading newline. */
static char* cmdlPromptStr = CMDL_PROMPT;

/*! String that contains the command string. */
static char cmdlCmdString[CMDL_MAX_COMMAND_LENGTH + 1];

/*! Array of pointers to all registered cmdlCommandT structures. */
static cmdlCommandT cmdlCmdArray[CMDL_MAX_COMMAND_COUNT];


//*****************************************************************************
//**************************** LOCAL FUNCTIONS ********************************
//*****************************************************************************

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
        printf(CMDL_LABEL "For detailed information type: \"help <command>\"\n");
    }
    else // specify a single command
    {
        command_ptr = cmdlFindCommand(argv[1]);
        if(command_ptr == NULL)
        {
            printf(CMDL_LABEL "Unknown command: %s\n", argv[1]);
        }
        else
        {
            printf(CMDL_LABEL "Usage: %s\n", command_ptr->usagePtr);
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
static void cmdlDumpCmdString(void)
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
** \param   uartHandle  Identifies a UART port.
** \param   cmdlExecTriggerPtr
**                      A function pointer that triggers the execution of
**                      CMDL_Execute(), e.g., by setting a flag.
**                      Note that the function pointer should not invoke
**                      CMDL_Execute() directly as it runs in interrupt context.
** \param   options     Options affecting the CMDL behaviour.
**
** \return
**          - #CMDL_OK on success.
**          - #CMDL_ERR_UART_NOT_INITIALIZED if the UART is not initialized.
**          - #CMDL_ERR_BAD_PARAMETER if cmdlExecTriggerPtr is NULL.
**          - #CMDL_ERR_UART_NOT_OK if the callback functions could not be
**            registered.
**          - #CMDL_ERR_NO_COMMAND_SLOT if there are no command slots
**            available to register the fundamental \exit command.
**
*******************************************************************************
*/
uint8_t CMDL_Init(UART_HandleT uartHandle,
                  UART_RxCallbackT cmdlExecTriggerPtr,
                  CMDL_OptionsT options)
{
    uint8_t ret_code;
    UART_RxCallbackOptionsT callbackOptions;

    if(UART_IsInitialized(uartHandle) == 0)
    {
        return(CMDL_ERR_UART_NOT_INITIALIZED);
    }
    if (cmdlExecTriggerPtr == NULL)
    {
        return (CMDL_ERR_BAD_PARAMETER);
    }
    // Erase commands:
    memset(cmdlCmdArray, 0, sizeof(cmdlCmdArray));

    // Register backspace callback:
    callbackOptions.execOnRxWait = 0;
    callbackOptions.writeRxToBuffer = 0;
    ret_code = UART_RegisterRxCallback(uartHandle,
                                       0x08, // backspace
                                       UART_RxCallbackOnBackspace,
                                       uartHandle, callbackOptions);
    if (ret_code != UART_OK)
    {
        // TODO: just return the UART error code?
        // -> Yes, when each driver has its own error base.
        return (CMDL_ERR_UART_NOT_OK);
    }
    // Register execute callback:
    callbackOptions.execOnRxWait = 0;
    callbackOptions.writeRxToBuffer = 0;
    ret_code = UART_RegisterRxCallback(uartHandle,
                                       0x0A, // line feed (LF)
                                       cmdlExecTriggerPtr,
                                       NULL, callbackOptions);
    if (ret_code != UART_OK)
    {
        // TODO: just return the UART error code?
        // -> Yes, when each driver has its own error base.
        return (CMDL_ERR_UART_NOT_OK);
    }

    // Register internal \c help command:
#if CMDL_USAGE_STRING_SUPPORT
    ret_code = CMDL_RegisterCommand(&cmdlHelp,
                                    "help",
                                    "help <[command]>");
#else // #if CMDL_USAGE_STRING_SUPPORT
    ret_code = CMDL_RegisterCommand(&cmdlHelp,
                                    "help");
#endif // #if CMDL_USAGE_STRING_SUPPORT
    if (ret_code != CMDL_OK)
    {
        return (ret_code);
    }

    // Apply options and state:
    cmdlUartHandle = uartHandle;
    cmdlState.flushRxAfterExec = options.flushRxAfterExec;
    cmdlState.initialized = 1;

    return(CMDL_OK);
}

/*!
*******************************************************************************
** \brief   Test whether CMDL is initialized.
**
** \return  - 1 if CMDL is initialized.
**          - 0 if CMDL is not initialized.
**
*******************************************************************************
*/
uint8_t CMDL_IsInitialized(void)
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
**                  such as "foo <bar> <[optArg]>" for the command foo with
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
uint8_t CMDL_RegisterCommand(void (*funcPtr) (uint8_t argc, char* argv[]),
                             char* namePtr,
                             char* usagePtr)
#else //CMDL_USAGE_STRING_SUPPORT
uint8_t CMDL_RegisterCommand(void (*funcPtr) (uint8_t argc, char* argv[]),
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
            printf(CMDL_LABEL_DEBUG "Registering command: %s\n", namePtr);
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
** \brief   Print the prompt of the command line interface. This function
**          should be called once after application startup and after every
**          execution of CMDL_Execute().
**
** \param   prefixStr   Optional prefix string.
**
*******************************************************************************
*/
void CMDL_PrintPrompt(char* prefixStr)
{
    // Print the prompt:
    //UART_TxField((uint8_t*)cmdlPromptStr, strlen(cmdlPromptStr));
    if (prefixStr)
    {
        printf("\n[%s] %s", prefixStr, cmdlPromptStr);
    }
    else
    {
        printf("\n%s", cmdlPromptStr);
    }
    return;
}

/*!
*******************************************************************************
** \brief   Execute a command. The execution of this function should be
**          triggered by cmdlExecTriggerPtr, which is passed during
**          initialization. However, it shouldn't be called directly from
**          cmdlExecTriggerPtr, as the latter runs in interrupt context.
**
*******************************************************************************
*/
void CMDL_Execute(void)
{
    // Points to arguments passed, 1 argument for argv[0] as command name string:
    char*    argv[CMDL_MAX_ARGUMENT_COUNT + 1];
    uint8_t  argc; // count of parameters, including command name string
    uint8_t  cmd_len; // length of the received command string
    char*    tok_ptr; // points to received tokens
    cmdlCommandT* command_ptr; // points to the command to execute

    // Read the command from rx buffer into CMDL's command buffer:
    memset(cmdlCmdString, 0, sizeof(cmdlCmdString));
    cmd_len = UART_RxField(cmdlUartHandle,
                           (uint8_t*)cmdlCmdString,
                           CMDL_MAX_COMMAND_LENGTH);
    cmdlCmdString[cmd_len] = '\0'; // terminate string

    // Remove carriage return in case of CR+LF
    if(cmdlCmdString[cmd_len - 1] == 0x0D) // carriage return
    {
        cmdlCmdString[cmd_len - 1] = '\0';
    }

    // Parse the string into tokens:
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
        // Execute the command:
        command_ptr->funcPtr(argc, argv);
    }
    else if (argc) // Print only if anything typed:
    {
        printf(CMDL_LABEL "Unknown command: %s\n", argv[0]);
    }
    if(cmdlState.flushRxAfterExec)
    {
        // Discard the receive buffer before next execution starts:
        UART_RxDiscard(cmdlUartHandle);
    }
    return;
}

#if ( __AVR_LIBC_VERSION__ < 10603UL )
/*!
*******************************************************************************
** \brief   This is an implementation of the strtok function which is available
**          in avr-libc from version 1.06.03 on. If the version of avr-libc
**          is older, this implementation will be used.
**          The procedure is NOT reentrant.
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
