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
**          supported devices:
**          atmega644p, atmega644, atmega16
**
** \attention
**          DO NOT call UART_TxByte() or UART_TxField() when the global 
**          interrupt is disabled (SREG_I bit in SREG)! 
**          Otherwise the MCU may hang up.
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

#if UART_LED_MODE
    //! Enable UDR empty interrupt and light the LED
    #define UART_UDR_EMPTY_INT_ON       UCSRB |=  (1 << UDRIE);   \
                                        SET_HIGH(UART_TX_LED)
    //! Disable UDR empty interrupt and turn off LED
    #define UART_UDR_EMPTY_INT_OFF      UCSRB &= ~(1 << UDRIE);   \
                                        SET_LOW(UART_TX_LED)
    #define UART_RX_LED_ON              SET_HIGH(UART_RX_LED)
    #define UART_RX_LED_OFF             SET_LOW(UART_RX_LED)
#else // UART_LED_MODE
    #define UART_UDR_EMPTY_INT_ON       UCSRB |=  (1 << UDRIE)
    #define UART_UDR_EMPTY_INT_OFF      UCSRB &= ~(1 << UDRIE)
#endif // UART_LED_MODE

    #define UART_TX_COMPLETE_INT_ON     UCSRB |=  (1 << TXCIE)
    #define UART_TX_COMPLETE_INT_OFF    UCSRB &= ~(1 << TXCIE)

#define atmega_multiport_mcu (atmega644 || atmega644p)

// For single-port MCUs with a port selector set UART_PORT to 0:
#if atmega644
    #undef  UART_PORT
    #define UART_PORT   0
#endif

// TODO: This would be the code for a generic UART port selection:
//#if atmega_multiport
//    #define UART_UCSRB(PRT)         _xUART_UCSRB(PRT)
//    #define _xUART_UCSRB(PRT)       UCSR  ## PRT ## B
//    #define UCSRB   UART_UCSRB(UART_PORT)
//
//    #define UART_UDRIE(PRT)         _xUART_UDRIE(PRT)
//    #define _xUART_UDRIE(PRT)       UDRIE ## PRT
//    #define UDRIE   UART_UDRIE(UART_PORT)
//
//    ...
//
//#endif // atmega_multiport


#if atmega_multiport_mcu
    // register names:
    #define UDR     UDR0
    #define UCSRA   UCSR0A
    #define UCSRB   UCSR0B
    #define UCSRC   UCSR0C
    #define UBRR    UBRR0

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

    // interrupt handlers:
    #define USART_RXC_vect  USART0_RX_vect
    #define USART_TXC_vect  USART0_TX_vect
    #define USART_UDRE_vect USART0_UDRE_vect

#endif // atmega_multiport_mcu

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
} uartCallbackRxStateT;

/*! UART specific callback structure. */
typedef struct
{
    uint8_t character; //<! the byte on which the rx will run the callback
    void (*funcPtr) (void* optArgPtr); //<! the callback function
    void* optArgPtr; //<! optional arguments passed to the callback on execution
    uartCallbackRxStateT state; //<! additional options and state
} uartCallbackRxT;

//*****************************************************************************
//**************************** LOCAL VARIABLES ********************************
//*****************************************************************************

static BUFFER_BufT uartRxBuffer;
static uint8_t uartRxBufferArray[UART_BUFFER_LENGTH_RX];

static BUFFER_BufT uartTxBuffer;
static uint8_t uartTxBufferArray[UART_BUFFER_LENGTH_TX];

static uartCallbackRxT uartCallbackRxArray[UART_CALLBACK_COUNT];

volatile static struct
{
    uint8_t initialized : 1; //<! Indicates whether UART has been initialized
    uint8_t rxWaiting   : 1; //<! Indicates active waiting state in UART_RxByte
    uint8_t txIntEn     : 1; //<! Stores tx interrupt enable bit temporarily
    uint8_t rxIntEn     : 1; //<! Stores rx interrupt enable bit temporarily
    uint8_t txActive    : 1; //<! Indicates an ongoing transmission
} uartState = {0, 0, 0, 0, 0};

#ifdef UART_ERROR_HANDLING
void (*uartFrameErrorHandlerPtr) (void);
void (*uartDataOverRunHandlerPtr) (void);
void (*uartParityErrorHandlerPtr) (void);
void (*uartRxBufferOverflowHandlerPtr) (void);
#endif // UART_ERROR_HANDLING

//*****************************************************************************
//********************** LOCAL FUNCTION DECLARATIONS **************************
//*****************************************************************************

static void uartEnterTxCS(void);
static void uartLeaveTxCS(void);
static void uartEnterRxCS(void);
static void uartLeaveRxCS(void);

//*****************************************************************************
//**************************** LOCAL FUNCTIONS ********************************
//*****************************************************************************

/*!
*******************************************************************************
** \brief   Prepares the uart for entering a critical section for the
**          transmit buffer.
**
**          A critical section is a piece of code where a shared resource
**          is accessed. The UDR empty interrupt is disabled in order to
**          keep the shared data consistent.
**
*******************************************************************************
*/
static void uartEnterTxCS(void)
{
    uartState.txIntEn = (UCSRB & (1 << UDRIE)) ? 1 : 0;
    UCSRB &= ~(1 << UDRIE);
    return;
}

/*!
*******************************************************************************
** \brief   Restores the uart for leaving a critical section for the transmit
**          buffer.
**
*******************************************************************************
*/
static void uartLeaveTxCS(void)
{
    if(uartState.txIntEn)
    {
        UCSRB |=  (1 << UDRIE);
    }
    return;
}

/*!
*******************************************************************************
** \brief   Prepares the uart for entering a critical section for the
**          receive buffer.
**
*******************************************************************************
*/
static void uartEnterRxCS(void)
{
    uartState.rxIntEn = (UCSRB & (1 << RXCIE)) ? 1 : 0;
    UCSRB &= ~(1 << RXCIE);
    return;
}

/*!
*******************************************************************************
** \brief   Restores the uart for leaving a critical section for the receive
**          buffer.
**
*******************************************************************************
*/
static void uartLeaveRxCS(void)
{
    if(uartState.rxIntEn)
    {
        UCSRB |= (1 << RXCIE);
    }
    return;
}

//*****************************************************************************
//*************************** PUBLIC FUNCTIONS ********************************
//*****************************************************************************

/*!
*******************************************************************************
** \brief   Initializes the USART hardware for a UART interface.
**
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
**                      both will be used by this driver.
**
** \return
**          - #UART_OK on success.
**          - #UART_ERR_BAD_PARAMETER if a bad parameter has been passed.
**
*******************************************************************************
*/
int8_t UART_Init (UART_BaudT       baudRate,
                  UART_ParityT     parityMode,
                  UART_StopBitT    stopBitMode,
                  UART_CharSizeT   charSizeMode,
                  UART_TransceiveT transceiveMode)
{
    uint8_t uart_sreg = 0;
    int8_t  result = 0;

    // Disable interrupts temporarily:
    uart_sreg = SREG;
    cli();

    /* normal transmission speed, no MPCM */
    UCSRA = 0x00;

    // init UCSRB and UCSRC with 0 first:
    UCSRB = 0x00;
#if atmega_multiport_mcu
    UCSRC = 0x00; // also: UMSEL0 1:0 = b00 => asynchronous UART
#endif // atmega_multiport_mcu
#if atmega16
    UCSRC = (1 << URSEL);
#endif // atmega16

    // set up the register configuration:
    result  = UART_SetBaudRate(baudRate);
    result |= UART_SetParity(parityMode);
    result |= UART_SetStopBit(stopBitMode);
    result |= UART_SetCharSize(charSizeMode);
    result |= UART_SetTransceiveMode(transceiveMode);

    if(result)
    {
        return(UART_ERR_BAD_PARAMETER);
    }

    // initialize the buffers:
    BUFFER_InitBuffer(&uartRxBuffer, uartRxBufferArray, UART_BUFFER_LENGTH_RX);
    BUFFER_InitBuffer(&uartTxBuffer, uartTxBufferArray, UART_BUFFER_LENGTH_TX);

    // initialize local variables:
    memset(uartCallbackRxArray, 0, UART_CALLBACK_COUNT * sizeof(uartCallbackRxT));
    uartState.rxWaiting   = 0;
    uartState.initialized = 1;

    // set up LEDs:
#if UART_LED_MODE
    SET_LOW(UART_TX_LED);
    SET_OUTPUT(UART_TX_LED);
    SET_LOW(UART_RX_LED);
    SET_OUTPUT(UART_RX_LED);
#endif // UART_LED_MODE

    // restore SREG:
    SREG = uart_sreg;

    return(UART_OK);
}

/*!
*******************************************************************************
** \brief   Test whether the UART is initialized.
**
** \return
**          - 1 if the UART is initialized.
**          - 0 if the UART is not initialized.
**
*******************************************************************************
*/
int8_t UART_IsInitialized(void)
{
    return(uartState.initialized);
}

/*!
*******************************************************************************
** \brief   Sets the Baud Rate Register in the Hardware according to the
**          given baudRate argument.
**
** \param   baudRate    New baud rate setting, indicates the speed in bps.
**
** \return
**          - #UART_OK on success.
**          - #UART_ERR_BAD_PARAMETER if a bad parameter has been passed.
**
*******************************************************************************
*/
int8_t UART_SetBaudRate(UART_BaudT baudRate)
{
    uint32_t baud;
#if atmega16
    uint16_t brr;
#endif

    switch(baudRate)
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
            return (UART_ERR_BAD_PARAMETER);
    }
#if atmega_multiport_mcu
    UBRR = (uint16_t) ((F_CPU / (baud << 4)) - 1);
#endif // atmega_multiport_mcu
#if atmega16
    brr = (uint16_t) ((F_CPU / (baud << 4)) - 1);
    UBRRH = brr >> 8;
    UBRRL = brr & 0xFF;
#endif // atmega16
    return (UART_OK);
}

/*!
*******************************************************************************
** \brief   Sets the Parity Mode in the Hardware according to the
**          given parityMode argument.
**
** \param   parityMode  New parity mode.
**
** \return
**          - #UART_OK on success.
**          - #UART_ERR_BAD_PARAMETER if a bad parameter has been passed.
**
*******************************************************************************
*/
int8_t UART_SetParity (UART_ParityT parityMode)
{
#if atmega_multiport_mcu
    switch(parityMode)
    {
        case UART_Parity_off:    // UPM1:0 = 00
            UCSRC &= ~((1 << UPM1) | (1 << UPM0));
            break;
        case UART_Parity_odd:    // UPM1:0 = 11
            UCSRC |= (1 << UPM1) | (1 << UPM0);
            break;
        case UART_Parity_even:   // UPM1:0 = 10
            UCSRC |=  (1 << UPM1);
            UCSRC &= ~(1 << UPM0);
            break;
        default:
            return(UART_ERR_BAD_PARAMETER);
    }
#endif // atmega_multiport_mcu

#if atmega16
    uint8_t ucsrc;
    uint8_t sreg_save;

    sreg_save = SREG;
    cli();
    ucsrc = UBRRH;
    ucsrc = UCSRC;

    switch(parityMode)
    {
        case UART_Parity_off:    // UPM1:0 = 00
            UCSRC = (ucsrc | (1 << URSEL)) & (~((1 << UPM1) | (1 << UPM0)));
            break;
        case UART_Parity_odd:    // UPM1:0 = 11
            UCSRC = ucsrc | (1 << UPM1) | (1 << UPM0) | (1 << URSEL);
            break;
        case UART_Parity_even:   // UPM1:0 = 10
            UCSRC = (ucsrc | (1 << UPM1) | (1 << URSEL)) & (~(1 << UPM0));
            break;
        default:
            SREG = sreg_save;
            return(UART_ERR_BAD_PARAMETER);
    }
    SREG = sreg_save;
#endif // atmega16

    return(UART_OK);
}

/*!
*******************************************************************************
** \brief   Sets the number of stop bits.
**
** \param   stopBitMode New number of stop bits. May indicate 1 or 2.
**
** \return
**          - #UART_OK on success.
**          - #UART_ERR_BAD_PARAMETER if a bad parameter has been passed.
**
*******************************************************************************
*/
int8_t UART_SetStopBit(UART_StopBitT stopBitMode)
{
#if atmega_multiport_mcu
    switch(stopBitMode)
    {
        case UART_StopBit_1:
            UCSRC &= ~(1 << USBS);
            break;
        case UART_StopBit_2:
            UCSRC |= (1 << USBS);
            break;
        default:
            return(UART_ERR_BAD_PARAMETER);
    }
#endif // atmega_multiport_mcu
#if atmega16
    uint8_t ucsrc;
    uint8_t sreg_save;

    sreg_save = SREG;
    cli();
    ucsrc = UBRRH;
    ucsrc = UCSRC;

    switch(stopBitMode)
    {
        case UART_StopBit_1:
            UCSRC = (ucsrc | (1 << URSEL)) & (~(1 << USBS));
            break;
        case UART_StopBit_2:
            UCSRC = ucsrc | (1 << USBS) | (1 << URSEL);
            break;
        default:
            SREG = sreg_save;
            return(UART_ERR_BAD_PARAMETER);
    }
    SREG = sreg_save;
#endif // atmega16
    return(UART_OK);
}

/*!
*******************************************************************************
** \brief   Sets the count of bits per transmission.
**
** \param   charSizeMode    New Mode to apply for the character size.
**
** \return
**          - #UART_OK on success.
**          - #UART_ERR_BAD_PARAMETER if a bad parameter has been passed.
**
*******************************************************************************
*/
int8_t UART_SetCharSize(UART_CharSizeT charSizeMode)
{
#if atmega_multiport_mcu
    // avoid 9-bit setting:
    UCSRB &= ~(1 << UCSZ2);

    // UCSZ1:0 settings:
    switch(charSizeMode)
    {
        case UART_CharSize_5:    // UCSZ1:0 = 00
            UCSRC &= ~((1 << UCSZ1) | (1 << UCSZ0));
            break;
        case UART_CharSize_6:    // UCSZ1:0 = 01
            UCSRC &= ~(1 << UCSZ1);
            UCSRC |=  (1 << UCSZ0);
            break;
        case UART_CharSize_7:    // UCSZ1:0 = 10
            UCSRC |=  (1 << UCSZ1);
            UCSRC &= ~(1 << UCSZ0);
            break;
        case UART_CharSize_8:    // UCSZ1:0 = 11
            UCSRC |=  ((1 << UCSZ1) | (1 << UCSZ0));
            break;
        default:
            return(UART_ERR_BAD_PARAMETER);
    }
#endif // atmega_multiport_mcu
#if atmega16
    uint8_t ucsrc;
    uint8_t sreg_save;

    sreg_save = SREG;
    cli();
    ucsrc = UBRRH;
    ucsrc = UCSRC;

    // UCSZ1:0 settings:
    switch(charSizeMode)
    {
        case UART_CharSize_5:    // UCSZ1:0 = 00
            UCSRC = (ucsrc | (1 << URSEL)) & (~((1 << UCSZ1) | (1 << UCSZ0)));
            break;
        case UART_CharSize_6:    // UCSZ1:0 = 01
            UCSRC = (ucsrc | (1 << UCSZ0) | (1 << URSEL)) & (~(1 << UCSZ1));
            break;
        case UART_CharSize_7:    // UCSZ1:0 = 10
            UCSRC = (ucsrc | (1 << UCSZ1) | (1 << URSEL)) & (~(1 << UCSZ0));
            break;
        case UART_CharSize_8:    // UCSZ1:0 = 11
            UCSRC = (ucsrc | (1 << UCSZ1) | (1 << UCSZ0) | (1 << URSEL));
            break;
        default:
            SREG = sreg_save;
            return(UART_ERR_BAD_PARAMETER);
    }
    SREG = sreg_save;
#endif // atmega16
    return(UART_OK);
}

/*!
*******************************************************************************
** \brief   Enable/disable the Receiver/Transmitter.
**
** \param   transceiveMode  Specifices the new mode to apply.
**
** \return
**          - #UART_OK on success.
**          - #UART_ERR_BAD_PARAMETER if a bad parameter has been passed.
**
*******************************************************************************
*/
int8_t UART_SetTransceiveMode (UART_TransceiveT transceiveMode)
{
    switch(transceiveMode)
    {
        case UART_Transceive_Rx:
            UCSRB |=  (1 << RXEN);
            UCSRB &= ~(1 << TXEN);
            // enable receive interrupt:
            UCSRB |= (1 << RXCIE);
            break;
        case UART_Transceive_Tx:
            UCSRB |=  (1 << TXEN);
            UCSRB &= ~(1 << RXEN);
            // disable receive interrupt:
            UCSRB &= ~(1 << RXCIE);
            break;
        case UART_Transceive_RxTx:
            UCSRB |= ((1 << RXEN) | (1 << TXEN));
            // enable receive interrupt:
            UCSRB |= (1 << RXCIE);
            break;
        default:
            return(UART_ERR_BAD_PARAMETER);
    }
    return(UART_OK);
}

/*!
*******************************************************************************
** \brief   Register a callback function for a specific character on rx.
**
**          The registered callback function will be executed every time
**          when the specified character arrives on the rx input.
**          If the UART is in active waiting state for UART_RxByte,
**          the callback will only execute if it has been registered
**          with #execOnRxWait as 1.
**          The callback must accept a void pointer as optional argument
**          and must be robust for this argument pointer to be NULL if the
**          optArgPtr argument is NULL.
**
** \attention
**          Callbacks are executed within ISR context and are time critical!
**
** \param   byte    Specifies the value on which the callback will be executed.
** \param   funcPtr Points to the callback function.
** \param   optArgPtr
**                  Optional argument that will be passed to the callback
**                  every time it is executed.
** \param   options
**                  Additional options for the callback function, see
**                  #UART_CallbackRxOptionsT.
**
** \return
**          - #UART_OK on success
**          - #UART_ERR_BAD_PARAMETER if the funcPtr is NULL
**          - #UART_ERR_NO_CALLBACK_SLOT if there is no callback slot free
**
*******************************************************************************
*/
int8_t UART_CallbackRxRegister(uint8_t byte,
                               void (*funcPtr) (void* optArgPtr),
                               void* optArgPtr,
                               UART_CallbackRxOptionsT options)
{
    uint8_t ii;

    if(funcPtr == NULL)
    {
        return(UART_ERR_BAD_PARAMETER);
    }

    ////////////////
    uartEnterRxCS();
    ////////////////

    // search for free slot:
    for(ii = 0; ii < UART_CALLBACK_COUNT; ii++)
    {
        if (uartCallbackRxArray[ii].funcPtr == NULL)
        {
            break;
        }
    }
    if (ii < UART_CALLBACK_COUNT) // empty slot found
    {
        uartCallbackRxArray[ii].character = byte;
        uartCallbackRxArray[ii].funcPtr = funcPtr;
        uartCallbackRxArray[ii].optArgPtr = optArgPtr;
        uartCallbackRxArray[ii].state.execOnRxWait = options.execOnRxWait;
        uartCallbackRxArray[ii].state.writeRxToBuffer = options.writeRxToBuffer;
        uartCallbackRxArray[ii].state.active = 1;
        ////////////////
        uartLeaveRxCS();
        ////////////////
        return(UART_OK);
    }
    else
    {
        ////////////////
        uartLeaveRxCS();
        ////////////////
        return(UART_ERR_NO_CALLBACK_SLOT);
    }
}

/*!
*******************************************************************************
** \brief   Unregister a previously registered rx callback function.
**
**          All rx callbacks that have been registered with the #byte argument
**          will be unregistered.
**
** \param   byte    The value on which the callback to remove has been executed.
**
** \return
**          - #UART_OK on success
**          - #UART_CALLBACK_NOT_FOUND if there has no callback been registered
**              with byte.
**
*******************************************************************************
*/
int8_t UART_CallbackRxUnregister(uint8_t byte)
{
    uint8_t ii; // temporary counter
    uint8_t found = 0; // indicates the count of callbacks found

    ////////////////
    uartEnterRxCS();
    ////////////////

    for(ii = 0; ii < UART_CALLBACK_COUNT; ii++)
    {
        if(uartCallbackRxArray[ii].character == byte)
        {
            memset(&uartCallbackRxArray[ii], 0, sizeof(uartCallbackRxT));
            //callback_ptr->character = 0x00;
            //callback_ptr->funcPtr = NULL;
            //callback_ptr->optArgPtr = NULL;
            //callback_ptr->options = 0;
            found++;
        }
    }

    ////////////////
    uartLeaveRxCS();
    ////////////////

    if(found)
    {
        return(UART_OK);
    }
    else
    {
        return(UART_ERR_CALLBACK_NOT_FOUND);
    }
}

/*!
*******************************************************************************
** \brief   Predefined callback for 'backspace' character (ASCII: 0x08).
**
**          This function can be registered as callback for the UART in order
**          to recognize the backspace key on your terminal. It removes the
**          latest written value from the buffer. Make sure that the option
**          writeRxToBuffer is disabled, otherwise the 'backspace character'
**          itself will be written to the buffer. In combination with
**          scanf it is recommended to set the execOnRxWait option to 0,
**          since scanf will empty the buffer causing this callback not
**          to take effect.
**          Register this function the following way in your application:
** \code
**          UART_CallbackRxOptionsT options;
**          options.execOnRxWait = 0;
**          options.writeRxToBuffer = 0;
**          UART_CallbackRxRegister(0x08, UART_CallbackRxBackspace, NULL, options);
** \endcode
**
** \param   optArgPtr   Just satisfies the interface, is not used here.
**
*******************************************************************************
*/
void UART_CallbackRxBackspace(void* optArgPtr)
{
    (void)BUFFER_ReadByteFromTail(&uartRxBuffer, NULL);
    return;
}

/*!
*******************************************************************************
** \brief   Transmit a single character.
**
** \param   byte    The value that will be transmitted. If the UART is driven
**                  with less than 8 bit, the most significant bits in the
**                  byte will not be transmitted.
**
*******************************************************************************
*/
void UART_TxByte (uint8_t byte)
{
    uint8_t result;

    // active waiting if buffer impends to overflow:
    do
    {
        uartEnterTxCS();
        result = BUFFER_GetFreeSize(&uartTxBuffer);
        uartLeaveTxCS();
    } while(result == 0);

    ////////////////
    uartEnterTxCS();
    ////////////////

    BUFFER_WriteByte(&uartTxBuffer, byte, NULL);
    uartState.txActive = 1;

    ////////////////
    uartLeaveTxCS();
    ////////////////

    // enable Tx interrupt:
    UART_UDR_EMPTY_INT_ON;
    return;
}

/*!
*******************************************************************************
** \brief   Read a single character from the receive buffer.
**
**          If the receive buffer is empty, active waiting will be performed
**          until data arrives on the UART. This is useful in combination
**          with stdio.
**
**          The read character will be removed from the receiver buffer.
**
** \return  The read character.
**
*******************************************************************************
*/
uint8_t UART_RxByte (void)
{
    uint8_t rxByte;
    uint8_t result;

    // active waiting if buffer is empty, wait for valid input data
    do
    {
        ////////////////
        uartEnterRxCS();
        ////////////////
        result = BUFFER_GetUsedSize(&uartRxBuffer);
        ////////////////
        uartLeaveRxCS();
        ////////////////
        uartState.rxWaiting = result ? 0 : 1;
    }while(result == 0);

    ////////////////
    uartEnterRxCS();
    ////////////////
    rxByte = BUFFER_ReadByte(&uartRxBuffer, NULL);
    ////////////////
    uartLeaveRxCS();
    ////////////////

    return(rxByte);
}

/*!
*******************************************************************************
** \brief   Transmit a couple of characters.
**
** \attention
**          Pointer arguments are not checked for NULL pointers. Make sure that
**          valid pointers are passed.
**
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
uint8_t UART_TxField (uint8_t* fieldPtr, uint8_t byteCount)
{
    uint8_t txCount;


    ////////////////
    uartEnterTxCS();
    ////////////////

    txCount = BUFFER_WriteField(&uartTxBuffer, fieldPtr, byteCount, NULL);
    uartState.txActive = 1;

    ////////////////
    uartLeaveTxCS();
    ////////////////

    // enable Tx interrupt:
    UART_UDR_EMPTY_INT_ON;
    return(txCount);
}

/*!
*******************************************************************************
** \brief   Reads a couple of characters from the receive buffer.
**
** \attention
**          Pointer arguments are not checked for NULL pointers. Make sure that
**          valid pointers are passed.
**
** \param   fieldPtr    Points to the location where the values from the receive
**                      buffer will be copied to.
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
uint8_t UART_RxField (uint8_t* fieldPtr, uint8_t byteCount)
{
    uint8_t rxCount;

    ////////////////
    uartEnterRxCS();
    ////////////////

    rxCount = BUFFER_ReadField(&uartRxBuffer, fieldPtr, byteCount, NULL);

    ////////////////
    uartLeaveRxCS();
    ////////////////

    return(rxCount);
}

/*!
*******************************************************************************
** \brief   Discard the rx buffer.
**
**          All values in the receive buffer are discarded.
**
*******************************************************************************
*/
void UART_RxDiscard(void)
{
    ////////////////
    uartEnterRxCS();
    ////////////////

    BUFFER_Discard(&uartRxBuffer);

    ////////////////
    uartLeaveRxCS();
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
*******************************************************************************
*/
void UART_TxFlush(void)
{
    while(uartState.txActive);
    return;
}

#if UART_ERROR_HANDLING

/*!
*******************************************************************************
** \brief   Register a handler for frame errors.
**
** \param   frameErrorHandlerPtr
**              New handler for frame errors.
**              Set to NULL if you do not need this handler.
**
*******************************************************************************
*/
void UART_SetFrameErrorHandler(void (*frameErrorHandlerPtr) (void))
{
    uartFrameErrorHandlerPtr = frameErrorHandlerPtr;
    return;
}

/*!
*******************************************************************************
** \brief   Register a handler for data overruns.
**
** \param   dataOverRunHandlerPtr
**              New handler for data overruns.
**              Set to NULL if you do not need this handler.
**
*******************************************************************************
*/
void UART_SetDataOverRunHandler(void (*dataOverRunHandlerPtr) (void))
{
    uartDataOverRunHandlerPtr = dataOverRunHandlerPtr;
    return;
}

/*!
*******************************************************************************
** \brief   Register a handler for parity errors.
**
** \param   parityErrorHandlerPtr
**              New handler for parity errors.
**              Set to NULL if you do not need this handler.
**
*******************************************************************************
*/
void UART_SetParityErrorHandler(void (*parityErrorHandlerPtr) (void))
{
    uartParityErrorHandlerPtr = parityErrorHandlerPtr;
    return;
}

/*!
*******************************************************************************
** \brief   Register a handler for buffer overflow of the rx buffer.
**
** \param   rxBufferOverflowHandlerPtr
**              New handler for rx buffer overflow.
**              Set to NULL if you do not need this handler.
**
*******************************************************************************
*/
void UART_SetRxBufferOverflowHandler(void (*rxBufferOverflowHandlerPtr) (void))
{
    uartRxBufferOverflowHandlerPtr = rxBufferOverflowHandlerPtr;
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
ISR(USART_RXC_vect, ISR_BLOCK)
{
    uint8_t rx; // received value
    uint8_t ii; // temporary counter
    uint8_t write_rx = 1; // indicates whether to write to the rx buffer

#if UART_ERROR_HANDLING
    uint8_t status;
    int8_t  result;
    status = UCSRA;
#endif // UART_ERROR_HANDLING

    rx = UDR;

#if UART_LED_MODE
    UART_RX_LED_ON;
#endif // UART_LED_MODE

#if UART_ERROR_HANDLING
    // execute error handlers:
    if(status & (1 << FE))
    {
        if (uartFrameErrorHandlerPtr) uartFrameErrorHandlerPtr();
    }
    if(status &  (1 << DOR))
    {
        if (uartDataOverRunHandlerPtr) uartDataOverRunHandlerPtr();
    }
    if(status & (1 << PE))
    {
        if (uartParityErrorHandlerPtr) uartParityErrorHandlerPtr();
    }
#endif // UART_ERROR_HANDLING

    // execute callbacks on rx:
    for(ii = 0; ii < UART_CALLBACK_COUNT; ii++)
    {
        if((uartCallbackRxArray[ii].character == rx) && (uartCallbackRxArray[ii].state.active))
        {
            if((uartState.rxWaiting == 0) || (uartCallbackRxArray[ii].state.execOnRxWait))
            {
                uartCallbackRxArray[ii].funcPtr(uartCallbackRxArray[ii].optArgPtr);
                write_rx = uartCallbackRxArray[ii].state.writeRxToBuffer;
            }
        }
    }
    if(write_rx)
    {
#if UART_ERROR_HANDLING
        BUFFER_WriteByte(&uartRxBuffer, rx, &result);
        if(result == BUFFER_ERR_FULL)
        {
            if (uartRxBufferOverflowHandlerPtr) uartRxBufferOverflowHandlerPtr();
        }
#else // UART_ERROR_HANDLING
        BUFFER_WriteByte(&uartRxBuffer, rx, NULL);
#endif // UART_ERROR_HANDLING
    }
#if UART_LED_MODE
    UART_RX_LED_OFF;
#endif // UART_LED_MODE
    return;
}

/*!
*******************************************************************************
** \brief   ISR for tx ready.
**
**          Will be executed when the Uart Data Register (UDR) is
**          indicated to be empty and the tx buffer is not empty.
**
*******************************************************************************
*/
ISR (USART_UDRE_vect, ISR_BLOCK)
{
    volatile uint8_t txByte;

    txByte = BUFFER_ReadByte(&uartTxBuffer, NULL);
    UDR = txByte;

    // disable transmission if buffer is empty now:
    if(BUFFER_GetUsedSize(&uartTxBuffer) == 0)
    {
        UART_UDR_EMPTY_INT_OFF;
        UART_TX_COMPLETE_INT_ON; // enable tx complete interrupt
    }
    return;
}

/*!
*******************************************************************************
** \brief   ISR for tx complete.
**
**          Will be executed when the transmission of the last character in
**          the transmit buffer is complete. This ISR resets the txActive flag.
**
*******************************************************************************
*/
ISR (USART_TXC_vect, ISR_BLOCK)
{
    UART_TX_COMPLETE_INT_OFF; // disable tx complete interrupt
    uartState.txActive = 0; // reset flag for active transmission
}
