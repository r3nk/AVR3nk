/*!
*******************************************************************************
*******************************************************************************
** \brief   Buffered UART driver for AVR microcontrollers
**
**          This driver allows to run the UART interface with various
**          baud rates, character sizes in the range of 5-8
**          (9 is not supported), 1 or 2 stop bits for each transmission,
**          and in rx-only, tx-only and rx+tx modes.
**          Both rx and tx are buffered and interrupt-driven. If the tx buffer
**          impends to overflow, active waiting is applied in order to keep
**          the transmit data consistent. This may slow down the system.
**          Choose an appropriate buffer size in the uart.h header file.
**          You may also register callback functions on specific rx events.
**          This is useful for program flow control such as in command line
**          applications.
**
** \attention
**          DO NOT call UART_TxByte() or UART_TxField() when the global 
**          interrupt is disabled (SREG_I bit in SREG)! 
**          Otherwise the MCU may hang up.
**
** \attention
**          In order to safely call UART_TxByte() or UART_TxField() from
**          within other ISRs, set the UART_INTERRUPT_SAFETY macro to 1.
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
#include <avr/io.h>
#include <avr/interrupt.h>
#include <drivers/macros_pin.h>
#include <drivers/buffer.h>
#include "uart.h"

//*****************************************************************************
//*************************** DEFINES AND MACROS ******************************
//*****************************************************************************

#define UART_SINGLEPORT     0 // 1 non-indexed port (older MCUs)
#define UART_MULTIPORT_1    1 // 1 indexed port
#define UART_MULTIPORT_2    2 // 2 indexed ports

#if (atmega8 || atmega16)
#define UART_PORT_FACTOR        UART_SINGLEPORT
#endif

#if atmega644
#define UART_PORT_FACTOR        UART_MULTIPORT_1
#endif

#if (atmega644p)
#define UART_PORT_FACTOR        UART_MULTIPORT_2
#endif

#if (UART_PORT_FACTOR == UART_SINGLEPORT)
#define USART0_RX_vect   USART_RXC_vect
#define USART0_TX_vect   USART_TXC_vect
#define USART0_UDRE_vect USART_UDRE_vect
#endif

#if (UART_PORT_FACTOR >= UART_MULTIPORT_1)
// UCSR0A bit positions:
#define RXC     RXC0
#define TXC     TXC0
#define UDRE    UDRE0
#define FE      FE0
#define DOR     DOR0
#define PE      UPE0
#define U2X     U2X0
#define MPCM    MPCM0

// UCSR0B bit positions:
#define RXCIE   RXCIE0
#define TXCIE   TXCIE0
#define UDRIE   UDRIE0
#define RXEN    RXEN0
#define TXEN    TXEN0
#define UCSZ2   UCSZ02
#define RXB8    RXB80
#define TXB8    TXB80

// UCSR0C bit positions:
#define UMSEL1  UMSEL01
#define UMSEL0  UMSEL00
#define UPM1    UPM01
#define UPM0    UPM00
#define USBS    USBS0
#define UCSZ1   UCSZ01
#define UCSZ0   UCSZ00
#define UCPOL   UCPOL0
#endif

//! Light the tx LED
#define UART_TX_LED_ON              if(handlePtr->txLedActive)  \
                                        *handlePtr->txLedPortPtr |= \
                                        (1 << handlePtr->txLedIdx)

//! Turn off the tx LED
#define UART_TX_LED_OFF             if(handlePtr->txLedActive)  \
                                        *handlePtr->txLedPortPtr &= \
                                        ~(1 << handlePtr->txLedIdx)

//! Light the rx LED
#define UART_RX_LED_ON              if(handlePtr->rxLedActive)  \
                                        *handlePtr->rxLedPortPtr |= \
                                        (1 << handlePtr->rxLedIdx)

//! Turn off the rx LED
#define UART_RX_LED_OFF             if(handlePtr->rxLedActive)  \
                                        *handlePtr->rxLedPortPtr &= \
                                        ~(1 << handlePtr->rxLedIdx)

//! Enable UDR empty interrupt
#define UART_UDR_EMPTY_INT_ON       *handlePtr->ucsrbPtr |=  (1 << UDRIE)

//! Disable UDR empty interrupt
#define UART_UDR_EMPTY_INT_OFF      *handlePtr->ucsrbPtr &= ~(1 << UDRIE)

//! Enable TX complete interrupt
#define UART_TX_COMPLETE_INT_ON     *handlePtr->ucsrbPtr |=  (1 << TXCIE)

//! Disable TX complete interrupt
#define UART_TX_COMPLETE_INT_OFF    *handlePtr->ucsrbPtr &= ~(1 << TXCIE)

//! Enable RX complete interrupt
#define UART_RX_COMPLETE_INT_ON     *handlePtr->ucsrbPtr |=  (1 << RXCIE)

//! Disable RX complete interrupt
#define UART_RX_COMPLETE_INT_OFF    *handlePtr->ucsrbPtr &= ~(1 << RXCIE)

//*****************************************************************************
//**************************** LOCAL DATA TYPES *******************************
//*****************************************************************************

/*! UART specific callback state. */
typedef struct
{
    //! Indicates whether to execute on active rx waiting in UART_RxByte.
    uint8_t execOnRxWait : 1;

    /*! Indicates whether the read value will still be inserted into the
    **  rx buffer or if it will be discarded.
    */
    uint8_t writeRxToBuffer : 1;

    /*! Indicates whether the callback is active at all. */
    uint8_t active : 1;

} uartRxCallbackStateT;

/*! UART rx trigger callback structure. */
typedef struct
{
    void (*funcPtr) (void* optArgPtr, uint8_t rxByte); //<! the callback function
    void* optArgPtr; //<! optional arguments passed to the callback on execution
    uartRxCallbackStateT state; //<! additional options and state
} uartRxTriggerCallbackT;

/*! UART rx character callback structure. */
typedef struct
{
    uint8_t character; //<! the byte on which the rx will run the callback
    void (*funcPtr) (void* optArgPtr); //<! the callback function
    void* optArgPtr; //<! optional arguments passed to the callback on execution
    uartRxCallbackStateT state; //<! additional options and state
} uartRxCallbackT;

/*! Internal UART handle structure. */
typedef struct
{
    volatile uint8_t* udrPtr;
    volatile uint8_t* ucsraPtr;
    volatile uint8_t* ucsrbPtr;

    volatile uint8_t initialized : 1; //<! Indicates whether UART has been initialized.
    volatile uint8_t rxWaiting   : 1; //<! Indicates active waiting state in UART_RxByte.
    volatile uint8_t txIntEn     : 1; //<! Stores tx interrupt enable bit temporarily.
    volatile uint8_t txActive    : 1; //<! Indicates an ongoing transmission.

    BUFFER_BufT rxBuffer;
    BUFFER_BufT txBuffer;

    uint8_t rxBufferArray[UART_BUFFER_LENGTH_RX];
    uint8_t txBufferArray[UART_BUFFER_LENGTH_TX];

    uartRxTriggerCallbackT rxTriggerCallback;
    uartRxCallbackT rxCallbackArray[UART_RX_CALLBACK_COUNT];

#ifdef UART_ERROR_HANDLING
    void (*frameErrorHandlerPtr) (void);
    void (*dataOverRunHandlerPtr) (void);
    void (*parityErrorHandlerPtr) (void);
    void (*rxBufferOverflowHandlerPtr) (void);
#endif

    // LED parameters:
    volatile uint8_t* txLedPortPtr;
    volatile uint8_t* rxLedPortPtr;
    uint8_t           txLedIdx : 3;
    uint8_t           rxLedIdx : 3;
    uint8_t           txLedActive : 1;
    uint8_t           rxLedActive : 1;

#if UART_INTERRUPT_SAFETY
    uint8_t           sregSave;
#endif
} uartHandleT;

//*****************************************************************************
//**************************** LOCAL VARIABLES ********************************
//*****************************************************************************

#if (UART_PORT_FACTOR <= UART_MULTIPORT_1)
static uartHandleT uartHandleArr[1];
#endif

#if (UART_PORT_FACTOR == UART_MULTIPORT_2)
static uartHandleT uartHandleArr[2];
#endif

//*****************************************************************************
//********************** LOCAL FUNCTION DECLARATIONS **************************
//*****************************************************************************

static void uartSetBaudRate (volatile uint8_t* ubrrhPtr,
                             volatile uint8_t* ubrrlPtr,
                             UART_BaudT baudRate);
static void uartEnterTxCS (uartHandleT* handlePtr);
static void uartLeaveTxCS (uartHandleT* handlePtr);
static void uartEnterRxCS (uartHandleT* handlePtr);
static void uartLeaveRxCS (uartHandleT* handlePtr);
static void uartIsrRx     (uartHandleT* handlePtr);
static void uartIsrUdre   (uartHandleT* handlePtr);
static void uartIsrTx     (uartHandleT* handlePtr);

//*****************************************************************************
//**************************** LOCAL FUNCTIONS ********************************
//*****************************************************************************

/*!
*******************************************************************************
** \brief   Sets the Baud Rate Register in the Hardware according to the
**          given baudRate argument.
**
** \param   ubrrhPtr    Points to the upper bits of the UBRR register.
** \param   ubrrlPtr    Points to the least significant bits of UBRR.
** \param   baudRate    New baud rate setting, indicates the speed in bps.
**
** \return
**          - #UART_OK on success.
**          - #UART_ERR_BAD_PARAMETER if a bad parameter has been passed.
**
*******************************************************************************
*/
static void uartSetBaudRate(volatile uint8_t* ubrrhPtr,
                            volatile uint8_t* ubrrlPtr,
                            UART_BaudT baudRate)
{
    uint32_t baud;
    uint16_t brr;

    switch (baudRate)
    {
        case UART_Baud_2400:
            baud = 2400;
            break;
        case UART_Baud_4800:
            baud = 4800;
            break;
        case UART_Baud_9600:
            baud = 9600;
            break;
        case UART_Baud_14400:
            baud = 14400;
            break;
        case UART_Baud_19200:
            baud = 19200;
            break;
        case UART_Baud_28800:
            baud = 28800;
            break;
        case UART_Baud_38400:
            baud = 38400;
            break;
        case UART_Baud_57600:
            baud = 57600;
            break;
        case UART_Baud_76800:
            baud = 76800;
            break;
        case UART_Baud_115200:
            baud = 115200;
            break;
        case UART_Baud_230400:
            baud = 230400;
            break;
        case UART_Baud_250000:
            baud = 250000;
            break;
        default:
            return;
    }
    brr = (uint16_t) ((F_CPU / (baud << 4)) - 1);
    *ubrrhPtr = brr >> 8;
    *ubrrlPtr = brr & 0xFF;
    return;
}

/*!
*******************************************************************************
** \brief   Prepares the uart for entering a critical section for the
**          transmit buffer.
**
**          A critical section is a piece of code where a shared resource
**          is accessed. The UDR empty interrupt is disabled in order to
**          keep the shared data consistent.
**
** \param   handlePtr
**              A handle associated with a specific AVR hardware UART.
**
*******************************************************************************
*/
static void uartEnterTxCS (uartHandleT* handlePtr)
{
    handlePtr->txIntEn = (*handlePtr->ucsrbPtr & (1 << UDRIE)) ? 1 : 0;
#if UART_INTERRUPT_SAFETY
    handlePtr->sregSave = SREG;
    cli();
#else
    UART_UDR_EMPTY_INT_OFF; // *handlePtr->ucsrbPtr &= ~(1 << UDRIE);
#endif
    return;
}

/*!
*******************************************************************************
** \brief   Restores the uart for leaving a critical section for the transmit
**          buffer.
**
** \param   handlePtr
**              A handle associated with a specific AVR hardware UART.
**
*******************************************************************************
*/
static void uartLeaveTxCS (uartHandleT* handlePtr)
{
    if (handlePtr->txIntEn)
    {
        UART_UDR_EMPTY_INT_ON; // *handlePtr->ucsrbPtr |= (1 << UDRIE);
    }
#if UART_INTERRUPT_SAFETY
    SREG = handlePtr->sregSave;
#endif
    return;
}

/*!
*******************************************************************************
** \brief   Prepares the uart for entering a critical section for the
**          receive buffer.
**
** \param   handlePtr
**              A handle associated with a specific AVR hardware UART.
**
*******************************************************************************
*/
static void uartEnterRxCS (uartHandleT* handlePtr)
{
#if UART_INTERRUPT_SAFETY
    handlePtr->sregSave = SREG;
    cli();
#else
    UART_RX_COMPLETE_INT_OFF;
#endif
    return;
}

/*!
*******************************************************************************
** \brief   Restores the uart for leaving a critical section for the receive
**          buffer.
**
** \param   handlePtr
**              A handle associated with a specific AVR hardware UART.
**
*******************************************************************************
*/
static void uartLeaveRxCS (uartHandleT* handlePtr)
{
#if UART_INTERRUPT_SAFETY
    SREG = handlePtr->sregSave;
#else
    if (*handlePtr->ucsrbPtr & (1 << RXEN))
    {
        UART_RX_COMPLETE_INT_ON;
    }
#endif
    return;
}

/*!
*******************************************************************************
** \brief   Wrapper for USART0_RX_vect and USART1_RX_vect ISR.
**
** \param   handlePtr
**              A handle associated with a specific AVR hardware UART.
**
*******************************************************************************
*/
static void uartIsrRx (uartHandleT* handlePtr)
{
    uint8_t rx; // received value
    uint8_t ii; // temporary counter
    uint8_t write_rx = 1; // indicates whether to write to the rx buffer
#if UART_ENABLE_RX_CALLBACK_NESTED_INTERRUPTS
    uint8_t sreg_save = 0;
#endif

#if UART_ERROR_HANDLING
    uint8_t status;
    int8_t  result;
#endif // UART_ERROR_HANDLING
    
    rx = *handlePtr->udrPtr;

    UART_RX_LED_ON;

#if UART_ERROR_HANDLING
    // execute error handlers:

    status = *handlePtr->ucsraPtr;

    if(status & (1 << FE))
    {
        if (handlePtr->frameErrorHandlerPtr) handlePtr->frameErrorHandlerPtr();
    }
    if(status & (1 << DOR))
    {
        if (handlePtr->dataOverRunHandlerPtr) handlePtr->dataOverRunHandlerPtr();
    }
    if(status & (1 << PE))
    {
        if (handlePtr->parityErrorHandlerPtr) handlePtr->parityErrorHandlerPtr();
    }
#endif // UART_ERROR_HANDLING
    // execute rx trigger callback (any character): 
    if (handlePtr->rxTriggerCallback.state.active)
    {
        if ((handlePtr->rxWaiting == 0)
        ||  (handlePtr->rxTriggerCallback.state.execOnRxWait))
        {
            handlePtr->rxTriggerCallback.funcPtr(
                    handlePtr->rxTriggerCallback.optArgPtr, rx);
            write_rx = handlePtr->rxTriggerCallback.state.writeRxToBuffer;
        }
    }
    // execute character-specific rx callbacks:
    for (ii = 0; ii < UART_RX_CALLBACK_COUNT; ii++)
    {
        if ((handlePtr->rxCallbackArray[ii].character == rx)
        &&  (handlePtr->rxCallbackArray[ii].state.active))
        {
            if ((handlePtr->rxWaiting == 0)
            ||  (handlePtr->rxCallbackArray[ii].state.execOnRxWait))
            {
#if UART_ENABLE_RX_CALLBACK_NESTED_INTERRUPTS
                // Allow nested interrupt temporarily for callback:
                sreg_save = SREG;
                sei();
#endif // UART_ENABLE_RX_CALLBACK_NESTED_INTERRUPTS
                handlePtr->rxCallbackArray[ii].funcPtr(
                        handlePtr->rxCallbackArray[ii].optArgPtr);
                write_rx =
                        handlePtr->rxCallbackArray[ii].state.writeRxToBuffer;
#if UART_ENABLE_RX_CALLBACK_NESTED_INTERRUPTS
                SREG = sreg_save;
#endif // UART_ENABLE_RX_CALLBACK_NESTED_INTERRUPTS
            }
        }
    }
    if (write_rx)
    {
#if UART_ERROR_HANDLING
        BUFFER_WriteByte(&handlePtr->rxBuffer, rx, &result);
        if ((result == BUFFER_ERR_FULL) 
        &&  (handlePtr->rxBufferOverflowHandlerPtr))
        {
            handlePtr->rxBufferOverflowHandlerPtr();
        }
#else // UART_ERROR_HANDLING
        BUFFER_WriteByte(&handlePtr->rxBuffer, rx, NULL);
#endif // UART_ERROR_HANDLING
    }
    UART_RX_LED_OFF;
    return;
}

/*!
*******************************************************************************
** \brief   Wrapper for USART0_UDRE_vect and USART1_UDRE_vect ISR.
**
** \param   handlePtr
**              A handle associated with a specific AVR hardware UART.
**
*******************************************************************************
*/
static void uartIsrUdre (uartHandleT* handlePtr)
{
    volatile uint8_t txByte;
    txByte = BUFFER_ReadByte(&handlePtr->txBuffer, NULL);
    *handlePtr->udrPtr = txByte;

    // disable transmission if buffer is empty now:
    if (BUFFER_GetUsedSize(&handlePtr->txBuffer) == 0)
    {
        UART_UDR_EMPTY_INT_OFF;
        UART_TX_COMPLETE_INT_ON; // enable tx complete interrupt
    }
    return;
}

/*!
*******************************************************************************
** \brief   Wrapper for USART0_TX_vect and USART1_TX_vect ISR.
**
** \param   handlePtr
**              A handle associated with a specific AVR hardware UART.
**
*******************************************************************************
*/
static void uartIsrTx (uartHandleT* handlePtr)
{
    UART_TX_COMPLETE_INT_OFF; // disable tx complete interrupt
    UART_TX_LED_OFF; // disable tx LED
    handlePtr->txActive = 0; // reset flag for active transmission
}

//*****************************************************************************
//*************************** PUBLIC FUNCTIONS ********************************
//*****************************************************************************

/*!
*******************************************************************************
** \brief   Initializes the USART hardware as a UART interface.
**
** \param   id          UART identifier. UART 0 is the first UART interface.
** \param   baudRate    Baud rate setting, indicates the speed in bps.
** \param   parityMode  Indicates whether a parity bit will be transmitted
**                      (transceiver) and expected (receiver). It can be
**                      switched off, odd or even.
** \param   stopBitMode Indicates whether one or two stop bits will be sent
**                      when transmitting data.
** \param   charSizeMode
**                      Indicates the number of bits per transmission.
** \param   transceiveMode
**                      Indicates whether the receiver, the transmitter or
**                      both will be enabled.
** \param   ledParamsPtr
**                      Optional pointer. Specifies setup for activity LEDs.
**
** \return
**          - A valid UART_HandleT handle on success.
**          - NULL if a bad parameter has been passed.
**
*******************************************************************************
*/
UART_HandleT UART_Init (uint8_t          id,
                        UART_BaudT       baudRate,
                        UART_ParityT     parityMode,
                        UART_StopBitT    stopBitMode,
                        UART_CharSizeT   charSizeMode,
                        UART_TransceiveT transceiveMode,
                        UART_LedParamsT* ledParamsPtr)
{
    uartHandleT* handlePtr = NULL;
    uint8_t sreg_save = 0;
    uint8_t ucsra     = 0;
    uint8_t ucsrb     = 0;
    uint8_t ucsrc     = 0;

    // no need to store these in the handle:
    volatile uint8_t* ucsrc_ptr;
    volatile uint8_t* ubrrh_ptr;
    volatile uint8_t* ubrrl_ptr;

    // Get the handle for the identified port:
    if ((id) < (sizeof(uartHandleArr) / sizeof(uartHandleArr[0])))
    {
        handlePtr = &uartHandleArr[id];
    }
    else
    {
        return (NULL);
    }

    // Reset handle:
    memset(handlePtr, 0, sizeof(uartHandleT));

    // Initialize register addresses for specified port:
#if (UART_PORT_FACTOR == UART_SINGLEPORT)
    handlePtr->udrPtr   = &UDR;
    handlePtr->ucsraPtr = &UCSRA;
    handlePtr->ucsrbPtr = &UCSRB;
    ucsrc_ptr            = &UCSRC;
    ubrrh_ptr            = &UBRRH;
    ubrrl_ptr            = &UBRRL;
#endif

#if (UART_PORT_FACTOR == UART_MULTIPORT_1)
    handlePtr->udrPtr   = &UDR0;
    handlePtr->ucsraPtr = &UCSR0A;
    handlePtr->ucsrbPtr = &UCSR0B;
    ucsrc_ptr            = &UCSR0C;
    ubrrh_ptr            = &UBRR0H;
    ubrrl_ptr            = &UBRR0L;
#endif

#if (UART_PORT_FACTOR == UART_MULTIPORT_2)
    if(id == 0)
    {
        handlePtr->udrPtr   = &UDR0;
        handlePtr->ucsraPtr = &UCSR0A;
        handlePtr->ucsrbPtr = &UCSR0B;
        ucsrc_ptr            = &UCSR0C;
        ubrrh_ptr            = &UBRR0H;
        ubrrl_ptr            = &UBRR0L;
    }
    if(id == 1)
    {
        handlePtr->udrPtr   = &UDR1;
        handlePtr->ucsraPtr = &UCSR1A;
        handlePtr->ucsrbPtr = &UCSR1B;
        ucsrc_ptr            = &UCSR1C;
        ubrrh_ptr            = &UBRR1H;
        ubrrl_ptr            = &UBRR1L;
    }
#endif

    // Disable interrupts temporarily:
    sreg_save = SREG;
    cli();

    /* normal transmission speed, no MPCM */
    *handlePtr->ucsraPtr = 0x00;

    // init UCSRB and UCSRC with 0 first:
    *handlePtr->ucsrbPtr = 0x00;
#if (UART_PORT_FACTOR == UART_SINGLEPORT)
    // select UCSRC register and clear
    *ucsrc_ptr = (1 << URSEL);
#else
    // clear UCSRC, also: UMSEL0 1:0 = b00 => asynchronous UART
    *ucsrc_ptr = 0x00;
#endif

    // set up the register configuration:
    uartSetBaudRate(ubrrh_ptr, ubrrl_ptr, baudRate);

    // parity setup
    switch (parityMode)
    {
        case UART_Parity_off:    // UPM1:0 = 00
            break;
        case UART_Parity_odd:    // UPM1:0 = 11
            ucsrc |= (1 << UPM1) | (1 << UPM0);
            break;
        case UART_Parity_even:   // UPM1:0 = 10
            ucsrc |= (1 << UPM1);
            break;
        default:
            SREG = sreg_save;
            return (NULL);
    }

    // stop bit setup
    switch (stopBitMode)
    {
        case UART_StopBit_1: // USBS = 0
            break;
        case UART_StopBit_2: // USBS = 1
            ucsrc |= (1 << USBS);
            break;
        default:
            SREG = sreg_save;
            return (NULL);
    }

    // character size setup

    // avoid 9-bit setting: UCSZ2 in ucsrb is already cleared.
    // UCSZ1:0 settings:
    switch (charSizeMode)
    {
        case UART_CharSize_5:    // UCSZ1:0 = 00
            break;
        case UART_CharSize_6:    // UCSZ1:0 = 01
            ucsrc |= (1 << UCSZ0);
            break;
        case UART_CharSize_7:    // UCSZ1:0 = 10
            ucsrc |= (1 << UCSZ1);
            break;
        case UART_CharSize_8:    // UCSZ1:0 = 11
            ucsrc |= ((1 << UCSZ1) | (1 << UCSZ0));
            break;
        default:
            SREG = sreg_save;
            return (NULL);
    }

    // receiver/transmitter setup
    switch (transceiveMode)
    {
        case UART_Transceive_Rx:
            // also enable receive interrupt:
            ucsrb |= (1 << RXCIE) | (1 << RXEN);
            break;
        case UART_Transceive_Tx:
            // transmit interrupt will be enabled automatically when sending
            ucsrb |= (1 << TXEN);
            break;
        case UART_Transceive_RxTx:
            // also enable receive interrupt:
            ucsrb |= (1 << RXCIE) | (1 << RXEN) | (1 << TXEN);
            break;
        default:
            SREG = sreg_save;
            return (NULL);
    }

    // apply configuration register settings
    *handlePtr->ucsraPtr = ucsra;
    *handlePtr->ucsrbPtr = ucsrb;

#if (UART_PORT_FACTOR == UART_SINGLEPORT)
    *ucsrc_ptr = (1 << URSEL) | ucsrc;
#else
    *ucsrc_ptr = ucsrc;
#endif

    // initialize the buffers:
    // realize as static arrays to avoid allocation from heap:
    BUFFER_InitBuffer(&handlePtr->rxBuffer,
                      handlePtr->rxBufferArray,
                      UART_BUFFER_LENGTH_RX);
    BUFFER_InitBuffer(&handlePtr->txBuffer,
                      handlePtr->txBufferArray,
                      UART_BUFFER_LENGTH_TX);

    // set up LEDs:
    if (ledParamsPtr)
    {
        if (ledParamsPtr->txLedPortPtr && ledParamsPtr->txLedDdrPtr)
        {
            handlePtr->txLedPortPtr = ledParamsPtr->txLedPortPtr;
            handlePtr->txLedIdx     = ledParamsPtr->txLedIdx;
            handlePtr->txLedActive  = 1;
            UART_TX_LED_OFF;
            // finally set pin as output:
            *ledParamsPtr->txLedDdrPtr |= (1 << ledParamsPtr->txLedIdx);
        }
        if (ledParamsPtr->rxLedPortPtr && ledParamsPtr->rxLedDdrPtr)
        {
            handlePtr->rxLedPortPtr = ledParamsPtr->rxLedPortPtr;
            handlePtr->rxLedIdx     = ledParamsPtr->rxLedIdx;
            handlePtr->rxLedActive  = 1;
            UART_RX_LED_OFF;
            // finally set pin as output:
            *ledParamsPtr->rxLedDdrPtr |= (1 << ledParamsPtr->rxLedIdx);
        }
    }

    // initialize local variables:
    memset(handlePtr->rxCallbackArray, 0, 
           UART_RX_CALLBACK_COUNT * sizeof(uartRxCallbackT));
    handlePtr->rxWaiting = 0;
    handlePtr->initialized = 1;

    // restore SREG:
    SREG = sreg_save;

    return ((UART_HandleT)handlePtr);
}

/*!
*******************************************************************************
** \brief   Test whether the UART is initialized.
**
** \param   handle      A handle associated with a specific AVR hardware UART.
**
** \return
**          - #UART_ERR_BAD_PARAMETER if handle is invalid.
**          - 1 if the UART is initialized.
**          - 0 if the UART is not initialized.
**
*******************************************************************************
*/
int8_t UART_IsInitialized (UART_HandleT handle)
{
    uartHandleT* handlePtr = (uartHandleT*)handle;

    if (handlePtr == NULL)
    {
        return (UART_ERR_BAD_PARAMETER);
    }
    return (handlePtr->initialized);
}

/*!
*******************************************************************************
** \brief   Register a callback function that is executed on the reception of
**          any character. 
**
**          The registered callback function will be executed every time
**          when any character arrives on the rx input.
**          If the UART is in active waiting state for UART_RxByte,
**          the callback will only execute if it has been registered
**          with #execOnRxWait as 1.
**
** \attention
**          Callbacks are executed within ISR context and are time critical!
**
** \param   handle  A handle associated with a specific AVR hardware UART.
** \param   funcPtr Points to the callback function.
** \param   optArgPtr   Optional argument that will be passed to the callback
**                      every time it is executed.
** \param   options     Additional options for the callback function, see
**                      #UART_RxCallbackOptionsT.
**
** \return
**          - #UART_OK on success
**
*******************************************************************************
*/
int8_t UART_RegisterRxTriggerCallback (UART_HandleT handle,
                                       void (*funcPtr)(void* optArgPtr, 
                                                       uint8_t rxByte), 
                                       void* optArgPtr,
                                       UART_RxCallbackOptionsT options)
{
    uartHandleT* handlePtr = (uartHandleT*)handle;
    
    if ((handlePtr == NULL) || (funcPtr == NULL))
    {
        return (UART_ERR_BAD_PARAMETER);
    }

    ////////////////
    uartEnterRxCS(handlePtr);
    ////////////////

    handlePtr->rxTriggerCallback.funcPtr = funcPtr;
    handlePtr->rxTriggerCallback.optArgPtr = optArgPtr;
    handlePtr->rxTriggerCallback.state.execOnRxWait = options.execOnRxWait;
    handlePtr->rxTriggerCallback.state.writeRxToBuffer = options.writeRxToBuffer;
    handlePtr->rxTriggerCallback.state.active = 1;

    ////////////////
    uartLeaveRxCS(handlePtr);
    ////////////////

    return (UART_OK);
}

/*!
*******************************************************************************
** \brief   Unregister the rx trigger callback function.
**
**          All rx callbacks that have been registered with the #rxByte 
**          argument will be unregistered.
**
** \param   handle  A handle associated with a specific AVR hardware UART.
**
** \return
**          - #UART_OK on success.
**          - #UART_ERR_BAD_PARAMETER if a bad parameter has been passed.
**          - #UART_CALLBACK_NOT_FOUND if there has no trigger callback been
**              registered.
**
*******************************************************************************
*/
int8_t UART_UnregisterRxTriggerCallback (UART_HandleT handle)
{
    uint8_t found = 0; // indicates if a callback has been registered
    uartHandleT* handlePtr = (uartHandleT*) handle;

    if (handlePtr == NULL)
    {
        return (UART_ERR_BAD_PARAMETER);
    }

    ////////////////
    uartEnterRxCS(handlePtr);
    ////////////////

    if (handlePtr->rxTriggerCallback.funcPtr) found = 1;
    memset(&handlePtr->rxTriggerCallback, 0, 
           sizeof(handlePtr->rxTriggerCallback));

    ////////////////
    uartLeaveRxCS(handlePtr);
    ////////////////

    if (found)
    {
        return (UART_OK);
    }
    else
    {
        return (UART_ERR_CALLBACK_NOT_FOUND);
    }
}

/*!
*******************************************************************************
** \brief   Register a callback function that is executed on the reception of
**          a specific character. 
**
**          The registered callback function will be executed every time
**          when the specified character arrives on the rx input.
**          If the UART is in active waiting state for UART_RxByte,
**          the callback will only execute if it has been registered
**          with #execOnRxWait as 1.
**          The callback must accept a void pointer as optional argument
**          and must be robust for this argument pointer to be NULL in case 
**          that the optArgPtr argument is NULL.
**
** \attention
**          Callbacks are executed within ISR context and are time critical!
**
** \param   handle      A handle associated with a specific AVR hardware UART.
** \param   rxByte      Specifies the value on which the callback will be
**                      executed.
** \param   funcPtr     Points to the callback function.
** \param   optArgPtr   Optional argument that will be passed to the callback
**                      every time it is executed.
** \param   options     Additional options for the callback function, see
**                      #UART_RxCallbackOptionsT.
**
** \return
**          - #UART_OK on success
**          - #UART_ERR_BAD_PARAMETER if a bad parameter has been passed.
**          - #UART_ERR_NO_CALLBACK_SLOT if there is no callback slot free
**
*******************************************************************************
*/
int8_t UART_RegisterRxCallback (UART_HandleT handle, 
                                uint8_t rxByte,
                                void (*funcPtr)(void* optArgPtr), 
                                void* optArgPtr,
                                UART_RxCallbackOptionsT options)
{
    uint8_t ii;
    uartHandleT* handlePtr = (uartHandleT*) handle;

    if ((handlePtr == NULL) || (funcPtr == NULL))
    {
        return (UART_ERR_BAD_PARAMETER);
    }

    ////////////////
    uartEnterRxCS(handlePtr);
    ////////////////

    // search for free slot:
    for (ii = 0; ii < UART_RX_CALLBACK_COUNT; ii++)
    {
        if (handlePtr->rxCallbackArray[ii].funcPtr == NULL)
        {
            break;
        }
    }
    if (ii < UART_RX_CALLBACK_COUNT) // empty slot found
    {
        handlePtr->rxCallbackArray[ii].character = rxByte;
        handlePtr->rxCallbackArray[ii].funcPtr = funcPtr;
        handlePtr->rxCallbackArray[ii].optArgPtr = optArgPtr;
        handlePtr->rxCallbackArray[ii].state.execOnRxWait =
                options.execOnRxWait;
        handlePtr->rxCallbackArray[ii].state.writeRxToBuffer =
                options.writeRxToBuffer;
        handlePtr->rxCallbackArray[ii].state.active = 1;
        ////////////////
        uartLeaveRxCS(handlePtr);
        ////////////////
        return (UART_OK);
    }
    else
    {
        ////////////////
        uartLeaveRxCS(handlePtr);
        ////////////////
        return (UART_ERR_NO_CALLBACK_SLOT);
    }
}

/*!
*******************************************************************************
** \brief   Unregister a previously registered rx callback function.
**
**          All rx callbacks that have been registered with the #rxByte 
**          argument will be unregistered.
**
** \param   handle  A handle associated with a specific AVR hardware UART.
** \param   rxByte  The value on which the callback to remove has been executed.
**
** \return
**          - #UART_OK on success.
**          - #UART_ERR_BAD_PARAMETER if a bad parameter has been passed.
**          - #UART_CALLBACK_NOT_FOUND if there has no callback been
**              registered with rxByte.
**
*******************************************************************************
*/
int8_t UART_UnregisterRxCallback (UART_HandleT handle, uint8_t rxByte)
{
    uint8_t ii; // temporary counter
    uint8_t found = 0; // indicates the count of callbacks found
    uartHandleT* handlePtr = (uartHandleT*) handle;

    if (handlePtr == NULL)
    {
        return (UART_ERR_BAD_PARAMETER);
    }

    ////////////////
    uartEnterRxCS(handlePtr);
    ////////////////

    for (ii = 0; ii < UART_RX_CALLBACK_COUNT; ii++)
    {
        if (handlePtr->rxCallbackArray[ii].character == rxByte)
        {
            memset(&handlePtr->rxCallbackArray[ii], 0, sizeof(uartRxCallbackT));
            found++;
        }
    }

    ////////////////
    uartLeaveRxCS(handlePtr);
    ////////////////

    if (found)
    {
        return (UART_OK);
    }
    else
    {
        return (UART_ERR_CALLBACK_NOT_FOUND);
    }
}

/*!
*******************************************************************************
** \brief   Predefined callback for 'backspace' character (ASCII: 0x08).
**
**          This function can be registered as callback for the UART in order
**          to recognize the backspace key on the terminal. It removes the
**          latest written value from the buffer. Make sure that the option
**          writeRxToBuffer is disabled, otherwise the backspace character
**          itself will be written to the buffer. In combination with
**          scanf it is recommended to set the execOnRxWait option to 0,
**          since scanf will empty the buffer causing this callback not
**          to take effect.
**          The optArgPtr must contain the addressed UART's handle. 
**          Register this function the following way in your application:
** \code
**          UART_RxCallbackOptionsT options;
**          options.execOnRxWait = 0;
**          options.writeRxToBuffer = 0;
**          UART_RegisterRxCallback(0x08, UART_RxCallbackOnBackspace, 
**                                  uartHandle, options);
** \endcode
**
** \param   optArgPtr   A handle associated with a specific AVR hardware UART.
**
*******************************************************************************
*/
void UART_RxCallbackOnBackspace (void* optArgPtr)
{
    uartHandleT* handlePtr = (uartHandleT*)optArgPtr;

    if (handlePtr == NULL) return;
    (void)BUFFER_ReadByteFromTail(&handlePtr->rxBuffer, NULL);
    return;
}

/*!
*******************************************************************************
** \brief   Transmit a single character.
**
** \param   handle  A handle associated with a specific AVR hardware UART.
** \param   byte    The value that will be transmitted. If the UART is driven
**                  with less than 8 bit, the most significant bits in the
**                  byte will not be transmitted.
**
*******************************************************************************
*/
void UART_TxByte (UART_HandleT handle, uint8_t byte)
{
    uint8_t result = 0;
    uartHandleT* handlePtr = (uartHandleT*) handle;

    if (handlePtr == NULL) return;

    ////////////////
    uartEnterTxCS(handlePtr);
    ////////////////

    // active waiting if buffer impends to overflow:
    do
    {
        ////////////////
        uartLeaveTxCS(handlePtr);
        ////////////////

        ; // no-op -> allow TX interrupt

        ////////////////
        uartEnterTxCS(handlePtr);
        ////////////////

        result = BUFFER_GetFreeSize(&handlePtr->txBuffer);
    } while (result == 0);

    // avoid txActive flag to be reset:
    UART_TX_COMPLETE_INT_OFF;

    BUFFER_WriteByte(&handlePtr->txBuffer, byte, NULL);
    handlePtr->txActive = 1;
    UART_TX_LED_ON;

    // action UART_UDR_EMPTY_INT_ON when leaving the CS:
    handlePtr->txIntEn = 1;

    ////////////////
    uartLeaveTxCS(handlePtr);
    ////////////////

    return;
}

/*!
*******************************************************************************
** \brief   Read a single character from the receive buffer.
**
**          If the receive buffer is empty, active waiting will be performed
**          until data arrives at the UART. This is useful in combination
**          with stdio.
**
**          The read character will be removed from the receive buffer.
**
** \param   handle  A handle associated with a specific AVR hardware UART.
**
** \return  The read character.
**
*******************************************************************************
*/
uint8_t UART_RxByte (UART_HandleT handle)
{
    uint8_t rxByte;
    uint8_t result;
    uartHandleT* handlePtr = (uartHandleT*)handle;

    if (handlePtr == NULL) return ('\0');

    ////////////////
    uartEnterRxCS(handlePtr);
    ////////////////

    // active waiting if buffer is empty, wait for valid input data
    do
    {
        ////////////////
        uartLeaveRxCS(handlePtr);
        ////////////////

        ; // no-op -> allow RX interrupt

        ////////////////
        uartEnterRxCS(handlePtr);
        ////////////////

        result = BUFFER_GetUsedSize(&handlePtr->rxBuffer);
        handlePtr->rxWaiting = result ? 0 : 1;
    } while (result == 0);
    rxByte = BUFFER_ReadByte(&handlePtr->rxBuffer, NULL);

    ////////////////
    uartLeaveRxCS(handlePtr);
    ////////////////

    return (rxByte);
}

/*!
*******************************************************************************
** \brief   Transmit a couple of characters.
**
** \attention
**          Pointer arguments are not checked for NULL pointers. Make sure
**          that valid pointers are passed.
**
** \param   handle      A handle associated with a specific AVR hardware UART.
** \param   fieldPtr    Points to the location of values that will be
**                      transmitted.
** \param   byteCount   The number of bytes that will be transmitted.
**                      Set to 0 if you want to fill the whole tx buffer.
**                      Anyway, make sure that the fieldPtr provides at least
**                      #UART_BUFFER_LENGTH_TX valid bytes for transmission!
**
** \return  The number of bytes that have actually been written to the
**          tx buffer. This value may be less than the passed byteCount if
**          the tx buffer was not able to store byteCount values. If the
**          tx buffer is full, 0 is returned.
**
*******************************************************************************
*/
uint8_t UART_TxField (UART_HandleT handle, uint8_t* fieldPtr, uint8_t byteCount)
{
    uint8_t txCount;
    uartHandleT* handlePtr = (uartHandleT*)handle;

    if (handlePtr == NULL) return (0);

    ////////////////
    uartEnterTxCS(handlePtr);
    ////////////////

    // avoid txActive flag to be reset:
    UART_TX_COMPLETE_INT_OFF;

    txCount = BUFFER_WriteField(&handlePtr->txBuffer, fieldPtr, byteCount, NULL);
    handlePtr->txActive = 1;
    UART_TX_LED_ON;

    // action UART_UDR_EMPTY_INT_ON when leaving the CS:
    handlePtr->txIntEn = 1;

    ////////////////
    uartLeaveTxCS(handlePtr);
    ////////////////

    return (txCount);
}

/*!
*******************************************************************************
** \brief   Reads a couple of characters from the receive buffer.
**
** \attention
**          Pointer arguments are not checked for NULL pointers. Make sure
**          that valid pointers are passed.
**
** \param   handle      A handle associated with a specific AVR hardware UART.
** \param   fieldPtr    Points to the location where the values from the
**                      receive buffer will be copied to.
** \param   byteCount   The number of bytes that will be read.
**                      Set to 0 if you want to read the whole rx buffer.
**                      However, make sure that the fieldPtr provides at least
**                      #UART_BUFFER_LENGTH_RX valid bytes for reception!
**
** \return  The number of bytes that have actually been read from the
**          rx buffer. This value may be less than the passed byteCount if
**          the rx buffer did not contain byteCount values. If the receive
**          buffer was empty, 0 is returned.
**
*******************************************************************************
*/
uint8_t UART_RxField (UART_HandleT handle, uint8_t* fieldPtr, uint8_t byteCount)
{
    uint8_t rxCount;
    uartHandleT* handlePtr = (uartHandleT*)handle;

    if (handlePtr == NULL) return (0);

    ////////////////
    uartEnterRxCS(handlePtr);
    ////////////////

    rxCount = BUFFER_ReadField(&handlePtr->rxBuffer, fieldPtr, byteCount, NULL);

    ////////////////
    uartLeaveRxCS(handlePtr);
    ////////////////

    return (rxCount);
}

/*!
*******************************************************************************
** \brief   Discard the rx buffer.
**
**          All values in the receive buffer are discarded.
**
** \param   handle  A handle associated with a specific AVR hardware UART.
**
*******************************************************************************
*/
void UART_RxDiscard (UART_HandleT handle)
{
    uartHandleT* handlePtr = (uartHandleT*)handle;

    if (handlePtr == NULL) return;

    ////////////////
    uartEnterRxCS(handlePtr);
    ////////////////

    BUFFER_Discard(&handlePtr->rxBuffer);

    ////////////////
    uartLeaveRxCS(handlePtr);
    ////////////////

    return;
}

/*!
*******************************************************************************
** \brief   Flush the tx buffer.
**
**          Active waiting until all characters in the transmit buffer
**          have been transmitted.
**
** \param   handle      A handle associated with a specific AVR hardware UART.
**
*******************************************************************************
*/
void UART_TxFlush (UART_HandleT handle)
{
    uartHandleT* handlePtr = (uartHandleT*)handle;

    if (handlePtr == NULL) return;

    while (handlePtr->txActive);
    return;
}

#if UART_ERROR_HANDLING

/*!
*******************************************************************************
** \brief   Register a handler for frame errors.
**
** \param   handle
**              A handle associated with a specific AVR hardware UART.
** \param   frameErrorHandlerPtr
**              New handler for frame errors.
**              Set to NULL if you do not need this handler.
**
*******************************************************************************
*/
void UART_SetFrameErrorHandler (UART_HandleT handle,
                                void (*frameErrorHandlerPtr) (void))
{
    uartHandleT* handlePtr = (uartHandleT*)handle;

    if(handlePtr == NULL) return;

    ////////////////
    uartEnterRxCS(handlePtr);
    ////////////////

    handlePtr->frameErrorHandlerPtr = frameErrorHandlerPtr;

    ////////////////
    uartLeaveRxCS(handlePtr);
    ////////////////

    return;
}

/*!
*******************************************************************************
** \brief   Register a handler for data overruns.
**
** \param   handle
**              A handle associated with a specific AVR hardware UART.
** \param   dataOverRunHandlerPtr
**              New handler for data overruns.
**              Set to NULL if you do not need this handler.
**
*******************************************************************************
*/
void UART_SetDataOverRunHandler (UART_HandleT handle,
                                 void (*dataOverRunHandlerPtr) (void))
{
    uartHandleT* handlePtr = (uartHandleT*)handle;

    if(handlePtr == NULL) return;

    ////////////////
    uartEnterRxCS(handlePtr);
    ////////////////

    handlePtr->dataOverRunHandlerPtr = dataOverRunHandlerPtr;

    ////////////////
    uartLeaveRxCS(handlePtr);
    ////////////////

    return;
}

/*!
*******************************************************************************
** \brief   Register a handler for parity errors.
**
** \param   handle
**              A handle associated with a specific AVR hardware UART.
** \param   parityErrorHandlerPtr
**              New handler for parity errors.
**              Set to NULL if you do not need this handler.
**
*******************************************************************************
*/
void UART_SetParityErrorHandler (UART_HandleT handle,
                                 void (*parityErrorHandlerPtr) (void))
{
    uartHandleT* handlePtr = (uartHandleT*)handle;

    if(handlePtr == NULL) return;

    ////////////////
    uartEnterRxCS(handlePtr);
    ////////////////

    handlePtr->parityErrorHandlerPtr = parityErrorHandlerPtr;

    ////////////////
    uartLeaveRxCS(handlePtr);
    ////////////////

    return;
}

/*!
*******************************************************************************
** \brief   Register a handler for buffer overflow of the rx buffer.
**
** \param   handle
**              A handle associated with a specific AVR hardware UART.
** \param   rxBufferOverflowHandlerPtr
**              New handler for rx buffer overflow.
**              Set to NULL if you do not need this handler.
**
*******************************************************************************
*/
void UART_SetRxBufferOverflowHandler (UART_HandleT handle,
        void (*rxBufferOverflowHandlerPtr) (void))
{
    uartHandleT* handlePtr = (uartHandleT*)handle;

    if(handlePtr == NULL) return;

    ////////////////
    uartEnterRxCS(handlePtr);
    ////////////////

    handlePtr->rxBufferOverflowHandlerPtr = rxBufferOverflowHandlerPtr;

    ////////////////
    uartLeaveRxCS(handlePtr);
    ////////////////

    return;
}

#endif // UART_ERROR_HANDLING

//*****************************************************************************
//*********************** INTERRUPT SERVICE ROUTINES **************************
//*****************************************************************************

/*!
*******************************************************************************
** \brief   ISR for rx complete.
**
**          Reads the received value into the receive buffer and executes
**          rx callbacks if not in active-waiting state.
**
*******************************************************************************
*/
ISR (USART0_RX_vect, ISR_BLOCK)
{
    uartHandleT* handlePtr = &uartHandleArr[0];
    uartIsrRx(handlePtr);
    return;
}

#if (UART_PORT_FACTOR == UART_MULTIPORT_2)
ISR (USART1_RX_vect, ISR_BLOCK)
{
    uartHandleT* handlePtr = &uartHandleArr[1];
    uartIsrRx(handlePtr);
    return;
}
#endif

/*!
*******************************************************************************
** \brief   ISR for tx ready.
**
**          Will be executed when the Uart Data Register (UDR) is
**          indicated to be empty and the tx buffer is not empty.
**
*******************************************************************************
*/
ISR (USART0_UDRE_vect, ISR_BLOCK)
{
    uartHandleT* handlePtr = &uartHandleArr[0];
    uartIsrUdre(handlePtr);
    return;
}

#if (UART_PORT_FACTOR == UART_MULTIPORT_2)
ISR (USART1_UDRE_vect, ISR_BLOCK)
{
    uartHandleT* handlePtr = &uartHandleArr[1];
    uartIsrUdre(handlePtr);
    return;
}
#endif

/*!
*******************************************************************************
** \brief   ISR for tx complete.
**
**          Will be executed when the transmission of the last character in
**          the transmit buffer is complete. This ISR resets the txActive
**          flag.
**
*******************************************************************************
*/
ISR (USART0_TX_vect, ISR_BLOCK)
{
    uartHandleT* handlePtr = &uartHandleArr[0];
    uartIsrTx(handlePtr);
}

#if (UART_PORT_FACTOR == UART_MULTIPORT_2)
ISR (USART1_TX_vect, ISR_BLOCK)
{
    uartHandleT* handlePtr = &uartHandleArr[1];
    uartIsrTx(handlePtr);
}
#endif
