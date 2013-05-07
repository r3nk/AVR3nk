/*!
*******************************************************************************
*******************************************************************************
** \brief     Template for sensor modules. It provides a debug macro switch for
**            verbose message printing and a non-debug mode for an energy saving
**            deep sleep mode.
**            Watchdog is used.
**            CAN frame generation is time triggered by the timer/counter 2.
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
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <drivers/uart.h>
#include <drivers/mcp2515.h>
#include <drivers/mcp2515_config.h>
#include <subsystems/cmdl.h>

//*****************************************************************************
//*************************** DEFINES AND MACROS ******************************
//*****************************************************************************

// Switch debug operation. In debug mode, the application may be more reactive
// on pressing the 'q' key in order to start up the command line utility.
// Also, more messages are printed in debug mode.
#define APP_DEBUG 1

// The number of overflows that need to occur to trigger sensor data generation.
// For F_CPU = 18432000, an overflow interrupt is triggered every 16/1125 seconds.
#define APP_OVERFLOW_THRESHOLD 16


//*****************************************************************************
//********************** LOCAL FUNCTION DECLARATIONS **************************
//*****************************************************************************

static int8_t appInit(void);
static int    appStdioPut(char chr, FILE* streamPtr);
static int    appStdioGet(FILE* streamPtr);
static void   appEnterCmdlFunc (void* optArgPtr);
static void   appList(uint8_t argc, char* argv[]);

//*****************************************************************************
//****************************** LOCAL DATA ***********************************
//*****************************************************************************

static FILE appStdio = FDEV_SETUP_STREAM(appStdioPut, appStdioGet, _FDEV_SETUP_RW);

static volatile struct flagStruct
{
    uint8_t enterCmdl    : 1;
    uint8_t generateData : 1;
} appFlags =
{
    .enterCmdl = 0,
    .generateData = 0
};

static uint8_t appCounter = 0;

//*****************************************************************************
//*************************** PUBLIC FUNCTIONS ********************************
//*****************************************************************************

// This function is called upon a HARDWARE RESET:
void reset(void) __attribute__((naked)) __attribute__((section(".init3")));

/*! Clear SREG_I on hardware reset. */
void reset(void)
{
    cli();
    // Note that for newer devices (any AVR that has the option to also
    // generate WDT interrupts), the watchdog timer remains active even
    // after a system reset (except a power-on condition), using the fastest
    // prescaler value (approximately 15 ms). It is therefore required
    // to turn off the watchdog early during program startup.
    MCUSR = 0; // clear reset flags
    wdt_disable();
}

int main(int argc, char* argv[])
{
    MCP2515_CanMessageT can_msg;
    MCP2515_TxParamsT   can_tx_params;

    int8_t result = 0;
    int8_t ii = 0;

    // enable watchdog timer:
    wdt_enable(WDTO_1S);

    // initialize the hardware:
    if(appInit()) return(-1);

    // print a header:
    printf("\n\n");
    for(ii = 0; ii < 48; ii++) printf("*");
    printf("\nDemo Sensor\n");
    for(ii = 0; ii < 48; ii++) printf("*");
    printf("\n");

    // Add some delay to wait for other nodes to initialize.
    // Delay resolution is automatically decreased if > 262.14 ms / F_CPU
    _delay_ms(100);

    can_tx_params.bufferId =   MCP2515_TX_BUFFER_0
                             | MCP2515_TX_BUFFER_1
                             | MCP2515_TX_BUFFER_2;
    can_tx_params.priority = MCP2515_TX_PRIORITY_1;

    while(1)
    {
        // enable watchdog timer:
        wdt_enable(WDTO_1S);
        appFlags.enterCmdl = 0;
        while(!appFlags.enterCmdl)
        {
            // send sensor data periodically:
            appFlags.generateData = 0;
            while(!appFlags.generateData)
            {
                // reset watchdog:
                wdt_reset();

                // enter sleep mode while idle:
                #if APP_DEBUG
                set_sleep_mode(SLEEP_MODE_IDLE);
                #else
                set_sleep_mode(SLEEP_MODE_EXT_STANDBY);
                #endif
                //sleep_mode(); // or alternatively:
                cli();
                sleep_enable();
                sei();
                sleep_cpu();
                sleep_disable();
            }

            // **********************************************************
            // **************** Send sensor data via CAN ****************
            // **********************************************************
            can_msg.sid = 0x67;
            can_msg.rtr = 0;
            can_msg.dlc = 2;
            can_msg.dataArray[0] = 0x13;
            can_msg.dataArray[1] = 0x37;

            result = MCP2515_Transmit(&can_msg,
                                      can_tx_params);

            // **********************************************************
            // **********************************************************

            #if APP_DEBUG
            printf("MCP2515_Transmit: %d\n", result);
            UART_TxFlush();
            #endif
            if(result < 0)
            {
                #if !APP_DEBUG
                printf("MCP2515_Transmit: %d\n", result);
                UART_TxFlush();
                #endif
                // wait for watchdog to trigger reset:
                while(1);
            }
        }
        wdt_disable();
        CMDL_Run();
        UART_TxFlush();
    }
    return(0);
}

//*****************************************************************************
//*************************** LOCAL FUNCTIONS *********************************
//*****************************************************************************

static int8_t appInit(void)
{
    UART_CallbackRxOptionsT rx_options;
    CMDL_OptionsT cmdl_options;
    MCP2515_InitParamsT appCanParams;
    int8_t result;

    // initialize UART and STDIO:
    result = UART_Init (UART_Baud_115200,
                        UART_Parity_off,
                        UART_StopBit_1,
                        UART_CharSize_8,
                        UART_Transceive_RxTx);
    if(result != UART_OK)
    {
        return(-1);
    }
    stdout = &appStdio;
    stdin  = &appStdio;
    sei();

    // register key callback that aborts the CMDL execution:
    memset(&rx_options, 0, sizeof(rx_options));
    rx_options.execOnRxWait = 0;
    rx_options.writeRxToBuffer = 0;
    result = UART_CallbackRxRegister((uint8_t)'q',
                                     appEnterCmdlFunc,
                                     NULL,
                                     rx_options);

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
    CMDL_RegisterCommand(&appList,
                         "list");

    // initialize MCP2515 CAN bus controller:
    memset(&appCanParams, 0, sizeof(appCanParams));
    appCanParams.initSPI = 1;
    appCanParams.wakeupLowPassFilter = 0;
    appCanParams.baudRatePrescaler = MCP2515_AUTO_BRP;
    appCanParams.synchronisationJumpWidth = MCP2515_AUTO_SJW;
    appCanParams.propagationSegmentLength = MCP2515_AUTO_PRSEG;
    appCanParams.phaseSegment1Length = MCP2515_AUTO_PHSEG1;
    appCanParams.phaseSegment2Length = MCP2515_AUTO_PHSEG2;

    // presettings of configurable arguments:
    appCanParams.samplePointCount = MCP2515_SAM_3;
    appCanParams.rolloverMode = MCP2515_ROLLOVER_ENABLE;
    appCanParams.oneShotMode = MCP2515_ONESHOT_DISABLE;
    appCanParams.rxBuffer0Mask    = 0x000; // 0x000 = accept all
    appCanParams.rxBuffer0Filter0 = 0x000;
    appCanParams.rxBuffer0Filter1 = 0x000;
    appCanParams.rxBuffer1Mask    = 0x000; // 0x000 = accept all
    appCanParams.rxBuffer1Filter2 = 0x000;
    appCanParams.rxBuffer1Filter3 = 0x000;
    appCanParams.rxBuffer1Filter4 = 0x000;
    appCanParams.rxBuffer1Filter5 = 0x000;
    result = MCP2515_Init(&appCanParams);

    // set up Timer/Counter 2 for periodic sensor data generation:
    TCNT2  = 0x00;
    ASSR   = 0x00; // disable asynchronous/external clock
    TCCR2A = 0x00; // normal operation
    TCCR2B = (1<<CS22) | (1<<CS21) | (1<<CS20); // prescaler setup
    TIMSK2 = (0<<OCIE2B) | (0<<OCIE2A) | (1<<TOIE2);
    return(result);
}

static int appStdioPut(char chr, FILE* streamPtr)
{
    UART_TxByte(chr);
    return(0);
}

static int appStdioGet(FILE* streamPtr)
{
    return((int)UART_RxByte());
}

static void appEnterCmdlFunc (void* optArgPtr)
{
    appFlags.enterCmdl = 1;
    return;
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


//*****************************************************************************
//*********************** INTERRUPT SERVICE ROUTINES **************************
//*****************************************************************************

ISR(TIMER2_OVF_vect, ISR_BLOCK)
{
    if(++appCounter >= APP_OVERFLOW_THRESHOLD)
    {
        appCounter = 0;
        appFlags.generateData = 1;
    }
}
