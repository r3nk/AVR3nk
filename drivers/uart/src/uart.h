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
#define UART_ERROR_HANDLING     0
#endif

//! rx buffer length, must be in range [1 ... 255]
#ifndef UART_BUFFER_LENGTH_RX
#define UART_BUFFER_LENGTH_RX   64
#endif

//! tx buffer length, must be in range [1 ... 255]
#ifndef UART_BUFFER_LENGTH_TX
#define UART_BUFFER_LENGTH_TX   128
#endif

//! Count of callback functions that can be assigned to specific rx values.
#ifndef UART_RX_CALLBACK_COUNT
#define UART_RX_CALLBACK_COUNT  3
#endif

//! Set to 1 if UART functions should be called from within ISRs.
#ifndef UART_INTERRUPT_SAFETY
#define UART_INTERRUPT_SAFETY   1
#endif

//! Set to 1 to enable nested interrupts within rx callback functions. 
#ifndef UART_ENABLE_RX_CALLBACK_NESTED_INTERRUPTS
#define UART_ENABLE_RX_CALLBACK_NESTED_INTERRUPTS   1
#endif

//! CPU frequency
#ifndef F_CPU
#define F_CPU                   18432000
#endif

//*****************************************************************************
//************************* UART SPECIFIC ERROR CODES *************************
//*****************************************************************************

/*! UART specific error base */
#define UART_ERR_BASE               0

/*! UART returns with no errors. */
#define UART_OK                     0

/*! A bad parameter has been passed. */
#define UART_ERR_BAD_PARAMETER      UART_ERR_BASE - 1

/*! There is no callback slot free. */
#define UART_ERR_NO_CALLBACK_SLOT   UART_ERR_BASE - 2

/*! The callback function has not been found. */
#define UART_ERR_CALLBACK_NOT_FOUND UART_ERR_BASE - 3


//*****************************************************************************
//******************************** DATA TYPES *********************************
//*****************************************************************************

/*! UART handle, which corresponds to a particular UART port. */
typedef void* UART_HandleT;

typedef enum
{
    UART_InterfaceId0 = 0,
    UART_InterfaceId1
} UART_InterfaceIdT;

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

/*! Specifies a PIN which will be lit on rx and tx activity respectively. */
typedef struct
{
    volatile uint8_t* txLedPortPtr;
    volatile uint8_t* rxLedPortPtr;
    volatile uint8_t* txLedDdrPtr;
    volatile uint8_t* rxLedDdrPtr;
    uint8_t           txLedIdx : 3;
    uint8_t           rxLedIdx : 3;
} UART_LedParamsT;

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
} UART_RxCallbackOptionsT;

//*****************************************************************************
//************************* FUNCTION DECLARATIONS *****************************
//*****************************************************************************

UART_HandleT UART_Init (UART_InterfaceIdT id,
                        UART_BaudT        baudRate,
                        UART_ParityT      parityMode,
                        UART_StopBitT     stopBitMode,
                        UART_CharSizeT    charSizeMode,
                        UART_TransceiveT  transceiveMode,
                        UART_LedParamsT*  ledParamsPtr);
int8_t  UART_IsInitialized(UART_HandleT handle);
int8_t  UART_RegisterRxTriggerCallback(UART_HandleT handle,
                                       void (*funcPtr)(void* optArgPtr, 
                                                       uint8_t rxByte), 
                                       void* optArgPtr,
                                       UART_RxCallbackOptionsT options);
int8_t  UART_UnregisterRxTriggerCallback(UART_HandleT handle);
int8_t  UART_RegisterRxCallback(UART_HandleT handle, 
                                uint8_t rxByte,
                                void (*funcPtr)(void* optArgPtr), 
                                void* optArgPtr,
                                UART_RxCallbackOptionsT options);
int8_t  UART_UnregisterRxCallback(UART_HandleT handle, uint8_t rxByte);
void    UART_RxCallbackOnBackspace(void* optArgPtr); 

void    UART_TxByte(UART_HandleT handle, uint8_t byte);
uint8_t UART_RxByte(UART_HandleT handle);
uint8_t UART_TxField(UART_HandleT handle, uint8_t* fieldPtr, uint8_t byteCount);
uint8_t UART_RxField(UART_HandleT handle, uint8_t* fieldPtr, uint8_t byteCount);
void    UART_RxDiscard(UART_HandleT handle);
void    UART_TxFlush(UART_HandleT handle);

#if UART_ERROR_HANDLING
void UART_SetFrameErrorHandler      (UART_HandleT handle,
                                     void (*frameErrorHandlerPtr) (void));
void UART_SetDataOverRunHandler     (UART_HandleT handle,
                                     void (*dataOverRunHandlerPtr) (void));
void UART_SetParityErrorHandler     (UART_HandleT handle,
                                     void (*parityErrorHandlerPtr) (void));
void UART_SetRxBufferOverflowHandler(UART_HandleT handle,
                                     void (*rxBufferOverflowHandlerPtr) (void));
#endif // UART_ERROR_HANDLING

#endif // UART_H
