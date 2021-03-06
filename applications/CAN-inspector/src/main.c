/*!
*******************************************************************************
*******************************************************************************
** \brief   CAN inspector application based on MCP2515.
**
**          The CAN inspector application allows to configure the MCP2515
**          CAN controller via a command line interface. It can be used to
**          sniff a CAN bus system or for message injection.
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/interrupt.h>
#include <drivers/uart.h>
#include <drivers/mcp2515.h>
#include <drivers/mcp2515_config.h>
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
static void   appPrintCanMsg(MCP2515_CanMessageT* msgPtr);
static void   appListenAbortFunc(void* optArgPtr);
static void   appCmdSetSamplePointCount(uint8_t argc, char* argv[]);
static void   appCmdSetRolloverMode(uint8_t argc, char* argv[]);
static void   appCmdSetOneshotMode(uint8_t argc, char* argv[]);
static void   appCmdSetMask(uint8_t argc, char* argv[]);
static void   appCmdSetFilter(uint8_t argc, char* argv[]);
static void   appCmdInit(uint8_t argc, char* argv[]);
static void   appCmdExit(uint8_t argc, char* argv[]);
static void   appCmdSendMessage(uint8_t argc, char* argv[]);
static void   appCmdListenCAN(uint8_t argc, char* argv[]);

//*****************************************************************************
//**************************** LOCAL VARIABLES ********************************
//*****************************************************************************

static UART_HandleT appUartHandle = NULL;
static FILE appStdio = FDEV_SETUP_STREAM(appStdioPut, appStdioGet, _FDEV_SETUP_RW);
static MCP2515_InitParamsT appCanParams;
static struct
{
    uint8_t canInitialized : 1;
    volatile uint8_t listenAbort : 1;
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
    UART_RxCallbackOptionsT cb_opts;
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

    // Register listen abort function:
    memset(&cb_opts, 0, sizeof(cb_opts));
    cb_opts.execOnRxWait = 1;
    cb_opts.writeRxToBuffer = 1;
    result = UART_RegisterRxCallback(appUartHandle, 'q',
                                     appListenAbortFunc, NULL, cb_opts);
    if(result != UART_OK)
    {
        printf("UART_RegisterRxCallback: %d\n", result);
        return(1);
    }

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
    CMDL_RegisterCommand(appCmdSetSamplePointCount, "setsamplepoints");
    CMDL_RegisterCommand(appCmdSetRolloverMode, "setrollover");
    CMDL_RegisterCommand(appCmdSetOneshotMode, "setoneshot");
    CMDL_RegisterCommand(appCmdSetMask, "setmask");
    CMDL_RegisterCommand(appCmdSetFilter, "setfilter");
    CMDL_RegisterCommand(appCmdInit, "caninit");
    CMDL_RegisterCommand(appCmdExit, "canexit");
    CMDL_RegisterCommand(appCmdSendMessage, "send");
    CMDL_RegisterCommand(appCmdListenCAN, "listen");

    // Set up CAN driver:
    memset(&appCanParams, 0, sizeof(appCanParams));
    appCanParams.initSPI = 1;
    appCanParams.wakeupLowPassFilter = 0;
    appCanParams.baudRatePrescaler = MCP2515_AUTO_BRP;
    appCanParams.synchronisationJumpWidth = MCP2515_AUTO_SJW;
    appCanParams.propagationSegmentLength = MCP2515_AUTO_PRSEG;
    appCanParams.phaseSegment1Length = MCP2515_AUTO_PHSEG1;
    appCanParams.phaseSegment2Length = MCP2515_AUTO_PHSEG2;

    // Presettings of configurable arguments:
    appCanParams.samplePointCount = MCP2515_SAM_3;
    appCanParams.rolloverMode = MCP2515_ROLLOVER_ENABLE;
    appCanParams.oneShotMode = MCP2515_ONESHOT_DISABLE;
    appCanParams.rxBuffer0Mask = 0x000; // accept all
    appCanParams.rxBuffer1Mask = 0x000; // accept all

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
** \brief   Print a CAN message in standard format (2.0A) via UART.
**
** \param   msgPtr  Pointer to a CAN message.
**
*******************************************************************************
*/
static void appPrintCanMsg(MCP2515_CanMessageT* msgPtr)
{
    uint8_t ii;

    printf("%03x %x %x - ", msgPtr->sid, msgPtr->rtr, msgPtr->dlc);
    for(ii = 0; ii < msgPtr->dlc; ii++)
    {
        printf("%02X ", msgPtr->dataArray[ii]);
    }
    printf("\n");
    return;
}

/*!
*******************************************************************************
** \brief   Set the listenAbort flag.
**
** \param   optArgPtr   Not used.
**
*******************************************************************************
*/
static void appListenAbortFunc(void* optArgPtr)
{
    optArgPtr = optArgPtr;
    appFlags.listenAbort = 1;
    return;
}

/*!
*******************************************************************************
** \brief   Set the count of sample points for the MCP2515.
**
** \param   argc    argument count
** \param   argv    argument vector
**
*******************************************************************************
*/
static void appCmdSetSamplePointCount(uint8_t argc, char* argv[])
{
    uint32_t cnt;

    if(argc == 2)
    {
        cnt = strtoul(argv[1], NULL, 0);
        if(cnt == 1)
        {
            appCanParams.samplePointCount = MCP2515_SAM_1;
            printf("samplePointCount = 1\n");
            return;
        }
        if(cnt == 3)
        {
            appCanParams.samplePointCount = MCP2515_SAM_3;
            printf("samplePointCount = 3\n");
            return;
        }
    }
    printf("Usage: setsamplepoints <cnt>\n" \
           "where cnt is either 1 or 3. (default: 3)\n");
    return;
}

/*!
*******************************************************************************
** \brief   Set up the rollover mode for the MCP2515.
**
** \param   argc    argument count
** \param   argv    argument vector
**
*******************************************************************************
*/
static void appCmdSetRolloverMode(uint8_t argc, char* argv[])
{
    uint32_t cnt;

    cnt = strtoul(argv[1], NULL, 0);
    if(cnt)
    {
        appCanParams.rolloverMode = MCP2515_ROLLOVER_ENABLE;
        printf("Rollover enabled.\n");
    }
    else
    {
        appCanParams.rolloverMode = MCP2515_ROLLOVER_DISABLE;
        printf("Rollover disabled.\n");
    }
    return;
}

/*!
*******************************************************************************
** \brief   Set up the oneshot mode for the MCP2515.
**
** \param   argc    argument count
** \param   argv    argument vector
**
*******************************************************************************
*/
static void appCmdSetOneshotMode(uint8_t argc, char* argv[])
{
    uint32_t cnt;

    cnt = strtoul(argv[1], NULL, 0);
    if(cnt)
    {
        appCanParams.oneShotMode = MCP2515_ONESHOT_ENABLE;
        printf("Oneshot enabled.\n");
    }
    else
    {
        appCanParams.oneShotMode = MCP2515_ONESHOT_DISABLE;
        printf("Oneshot disabled.\n");
    }
    return;
}

/*!
*******************************************************************************
** \brief   Set up the receiver masks for the MCP2515.
**
** \param   argc    argument count
** \param   argv    argument vector
**
*******************************************************************************
*/
static void appCmdSetMask(uint8_t argc, char* argv[])
{
    uint32_t idx;
    uint32_t value;

    if(argc == 3)
    {
        idx   = strtoul(argv[1], NULL, 0);
        value = strtoul(argv[2], NULL, 0);
        value &= 0x7FF; // limit to 11 bits

        if(idx == 0)
        {
            appCanParams.rxBuffer0Mask = value;
            printf("RXB0 mask = 0x%lX\n", value);
            return;
        }
        if(idx == 1)
        {
            appCanParams.rxBuffer1Mask = value;
            printf("RXB1 mask = 0x%lX\n", value);
            return;
        }
    }
    printf("Usage: setmask <idx> <value>\n" \
           "where idx is either 0 or 1.\n");
    return;
}

/*!
*******************************************************************************
** \brief   Set up the receiver filters for the MCP2515.
**
** \param   argc    argument count
** \param   argv    argument vector
**
*******************************************************************************
*/
static void appCmdSetFilter(uint8_t argc, char* argv[])
{
    uint32_t idx;
    uint32_t value;

    if(argc == 3)
    {
        idx   = strtoul(argv[1], NULL, 0);
        value = strtoul(argv[2], NULL, 0);
        value &= 0x7FF; // limit to 11 bits

        if((idx == 0) || (idx == 1))
        {
            if(idx == 0)
            {
                appCanParams.rxBuffer0Filter0 = value;
            }
            else
            {
                appCanParams.rxBuffer0Filter1 = value;
            }
            printf("(RXB0) filter %lu = 0x%lX\n", idx, value);
            return;
        }
        if((1 < idx) && (idx < 6))
        {
            switch(idx)
            {
                case(2):
                    appCanParams.rxBuffer1Filter2 = value;
                    break;
                case(3):
                    appCanParams.rxBuffer1Filter3 = value;
                    break;
                case(4):
                    appCanParams.rxBuffer1Filter4 = value;
                    break;
                case(5):
                    appCanParams.rxBuffer1Filter5 = value;
                    break;
                default:
                    break;
            }
            printf("(RXB1) filter %lu = 0x%lX\n", idx, value);
            return;
        }
    }
    printf("Usage: setfilter <idx> <value>\n" \
           "where idx is in the range of 0 - 5 inclusively.\n");
    return;
}

/*!
*******************************************************************************
** \brief   Initialize the MCP2515.
**
** \param   argc    argument count
** \param   argv    argument vector
**
*******************************************************************************
*/
static void appCmdInit(uint8_t argc, char* argv[])
{
    uint8_t result;

    printf("Initializing MCP2515...");
    result = MCP2515_Init(&appCanParams);
    if(result)
    {
        printf("error: %d\n", result);
    }
    else
    {
        printf("ok.\n");
        appFlags.canInitialized = 1;
    }
    return;
}

/*!
*******************************************************************************
** \brief   Exit the MCP2515 driver.
**
** \param   argc    argument count
** \param   argv    argument vector
**
*******************************************************************************
*/
static void appCmdExit(uint8_t argc, char* argv[])
{
    printf("Exiting MCP2515...");
    MCP2515_Exit();
    printf("ok.\n");
    appFlags.canInitialized = 0;
    return;
}

/*!
*******************************************************************************
** \brief   Send a message via CAN.
**
** \param   argc    argument count
** \param   argv    argument vector
**
*******************************************************************************
*/
static void appCmdSendMessage(uint8_t argc, char* argv[])
{
    MCP2515_CanAMessageT can_msg;
    MCP2515_TxParamsT can_params;
    MCP2515_TxBufferIdT buffer_id;
    int8_t ii;

    if(appFlags.canInitialized == 0)
    {
        printf("MCP2515 not initialized.\n");
        return;
    }
    if(argc > 3)
    {
        can_msg.sid = strtoul(argv[1], NULL, 0) & 0x7FF;
        can_msg.rtr = strtoul(argv[2], NULL, 0) ? 1 : 0;
        can_msg.dlc = strtoul(argv[3], NULL, 0);
        memset(&can_msg.dataArray, 0, sizeof(can_msg.dataArray));
        for(ii = 0; ((ii < can_msg.dlc) && (ii < (argc - 4))); ii++)
        {
            can_msg.dataArray[ii] = strtoul(argv[ii+4], NULL, 0) & 0xFF;
        }
        can_params.bufferId = MCP2515_TX_BUFFER_0;
        can_params.priority = MCP2515_TX_PRIORITY_0;
        printf("Message to send: \n");
        printf("SID: 0x%03x\n", can_msg.sid);
        printf("RTR: %d\n", can_msg.rtr);
        printf("DLC: %d\n", can_msg.dlc);
        printf("data: ");
        for(ii = 0; ii < can_msg.dlc; ii++)
        {
            printf("0x%02X ", can_msg.dataArray[ii]);
        }
        printf("\nSending message...");
        buffer_id = MCP2515_Transmit(&can_msg, can_params);
        if(buffer_id == 0) printf("error: No transmit buffer free.\n");
        else
        {
            switch (buffer_id)
            {
                case (MCP2515_TX_BUFFER_0):
                    ii = 0;
                    break;
                case (MCP2515_TX_BUFFER_1):
                    ii = 1;
                    break;
                case (MCP2515_TX_BUFFER_2):
                    ii = 2;
                    break;
            }
            printf("ok. Transmit buffer: %i\n", ii);
        }
        return;
    }
    printf("Usage: send <SID> <RTR> <DLC> <DATA>*\n");
    return;
}

/*!
*******************************************************************************
** \brief   Enter CAN listen mode and print verbose messages of received
**          messages.
**
** \param   argc    Not used.
** \param   argv    Not used.
**
*******************************************************************************
*/
static void appCmdListenCAN (uint8_t argc, char* argv[])
{
    if(appFlags.canInitialized == 0)
    {
        printf("MCP2515 not initialized.\n");
        return;
    }
    //printf("###_#_#_-_##_##_##_##_##_##_##_##\n");
    printf("\n");
    printf("    R D\n");
    printf(" I  T L\n");
    printf(" D  R C   data\n");
    printf("#################################\n");
    appFlags.listenAbort = 0;
    MCP2515_SetRxCallback(appPrintCanMsg);
    while(!appFlags.listenAbort);
    appFlags.listenAbort = 0;
    MCP2515_SetRxCallback(NULL);
    return;
}


//*****************************************************************************
//*************************** PUBLIC FUNCTIONS ********************************
//*****************************************************************************

/*!
*******************************************************************************
** \brief   Main procedure.
**
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
