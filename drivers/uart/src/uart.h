/*!
*******************************************************************************
*******************************************************************************
** \brief   UART interface declarations
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

#ifndef UART_H
#define UART_H

#include <stdint.h>

//*****************************************************************************
//*************************** DEFINES AND MACROS ******************************
//*****************************************************************************

//! Determines whether error handling is active.
#ifndef UART_ERROR_HANDLING
    #define UART_ERROR_HANDLING     1
#endif // UART_ERROR_HANDLING

//! rx buffer length, must be in range [1 ... 255]
#ifndef UART_BUFFER_LENGTH_RX
    #define UART_BUFFER_LENGTH_RX   64
#endif // UART_BUFFER_LENGTH_RX

//! tx buffer length, must be in range [1 ... 255]
#ifndef UART_BUFFER_LENGTH_TX
    #define UART_BUFFER_LENGTH_TX   64
#endif // UART_BUFFER_LENGTH_TX

//! count of callback functions that can be assigned to specific rx values
#ifndef UART_CALLBACK_COUNT
    #define UART_CALLBACK_COUNT     3
#endif // UART_CALLBACK_COUNT

//! enable/disable rx and tx LEDs for the UART
#ifndef UART_LED_MODE
    #define UART_LED_MODE           1
#endif // UART_LED_MODE

//! UART rx LED output: (PORT,PIN)
#ifndef UART_RX_LED
    #define UART_RX_LED             B,2
#endif // UART_RX_LED

//! UART tx LED output: (PORT,PIN)
#ifndef UART_TX_LED
    #define UART_TX_LED             B,3
#endif // UART_TX_LED

//! Port selector for multiport MCUs
#ifndef UART_PORT
    #define UART_PORT               0
#endif // UART_PORT

//! CPU frequency
#ifndef F_CPU
    #define F_CPU                   8000000
#endif // F_CPU

//*****************************************************************************
//************************* UART SPECIFIC ERROR CODES *************************
//*****************************************************************************

/*! UART specific error base */
#define UART_ERR_BASE               0

/*! UART returns with no errors. */
#define UART_OK                     0

/*! A bad parameter has been passed. */
#define UART_ERR_BAD_PARAMETER      UART_ERR_BASE - 1

/*! There is no callback slot left. */
#define UART_ERR_NO_CALLBACK_SLOT   UART_ERR_BASE - 2

/*! The callback function has not been found. */
#define UART_ERR_CALLBACK_NOT_FOUND UART_ERR_BASE - 3

//*****************************************************************************
//******************************** DATA TYPES *********************************
//*****************************************************************************

/*! Specifies baud rate settings that can be used for driving the
**  UART interface.
*/
typedef enum
{
    UART_Baud_2400 = 0,
    UART_Baud_4800,
    UART_Baud_9600,
    UART_Baud_14400,
    UART_Baud_19200,
    UART_Baud_28800,
    UART_Baud_38400,
    UART_Baud_57600,
    UART_Baud_76800,
    UART_Baud_115200,
    UART_Baud_230400,
    UART_Baud_250000
} UART_BaudT;

/*! Specifies parity modes. */
typedef enum
{
    UART_Parity_off = 0,
    UART_Parity_odd,
    UART_Parity_even
} UART_ParityT;

/*! Specifies possible modes for stop bit transmission. */
typedef enum
{
    UART_StopBit_1 = 1,
    UART_StopBit_2 = 2
} UART_StopBitT;

/*! Specifies modes for the count of bits per transmission/reception. */
typedef enum
{
    UART_CharSize_5 = 0,
    UART_CharSize_6,
    UART_CharSize_7,
    UART_CharSize_8,
    // UART_CharSize_9 // not supported!
} UART_CharSizeT;

/*! Specifies if the receiver, the transmitter or both will be used. */
typedef enum
{
    UART_Transceive_Rx = 0, //<! Rx only
    UART_Transceive_Tx,     //<! Tx only
    UART_Transceive_RxTx    //<! Rx and Tx enabled
} UART_TransceiveT;

/*! UART specific callback options. */
typedef struct
{
    //! Indicates whether to execute on active rx waiting in UART_RxByte.
    uint8_t execOnRxWait : 1;

    /*! Indicates whether the read value will still be inserted into the
    **  rx buffer or if it will be discarded. Only takes effect if the
    **  callback is executed.
    */
    uint8_t writeRxToBuffer : 1;
} UART_CallbackRxOptionsT;

//*****************************************************************************
//************************* FUNCTION DECLARATIONS *****************************
//*****************************************************************************

int8_t  UART_Init (UART_BaudT       baudRate,       \
                   UART_ParityT     parityMode,     \
                   UART_StopBitT    stopBitMode,    \
                   UART_CharSizeT   charSizeMode,   \
                   UART_TransceiveT transceiveMode);
int8_t  UART_IsInitialized (void);
int8_t  UART_SetBaudRate(UART_BaudT baudRate);
int8_t  UART_SetParity (UART_ParityT parityMode);
int8_t  UART_SetStopBit (UART_StopBitT stopBitMode);
int8_t  UART_SetCharSize (UART_CharSizeT charSizeMode);
int8_t  UART_SetTransceiveMode (UART_TransceiveT transceiveMode);
int8_t  UART_CallbackRxRegister (uint8_t byte,                      \
                                 void (*funcPtr) (void* optArgPtr), \
                                 void* optArgPtr,                   \
                                 UART_CallbackRxOptionsT options);
int8_t  UART_CallbackRxUnregister (uint8_t byte);
void    UART_CallbackRxBackspace (void* optArgPtr);
void    UART_TxByte (uint8_t byte);
uint8_t UART_RxByte (void);
uint8_t UART_TxField (uint8_t* fieldPtr, uint8_t byteCount);
uint8_t UART_RxField (uint8_t* fieldPtr, uint8_t byteCount);
void    UART_RxDiscard (void);
void    UART_TxFlush (void);

#if UART_ERROR_HANDLING
void UART_SetFrameErrorHandler(void (*frameErrorHandlerPtr) (void));
void UART_SetDataOverRunHandler(void (*dataOverRunHandlerPtr) (void));
void UART_SetParityErrorHandler(void (*parityErrorHandlerPtr) (void));
void UART_SetRxBufferOverflowHandler(void (*rxBufferOverflowHandlerPtr) (void));
#endif // UART_ERROR_HANDLING

#endif // UART_H
