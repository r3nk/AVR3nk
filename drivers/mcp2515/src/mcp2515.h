/*!
*******************************************************************************
*******************************************************************************
** \brief   Interface declarations of MCP2515 CAN bus driver.
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

#ifndef MCP2515_H
#define MCP2515_H

#include <stdint.h>

//*****************************************************************************
//*************************** DEFINES AND MACROS ******************************
//*****************************************************************************

/*! Specify whether to support messages with extended identifier.
**  More program and data RAM may be consumed when this option is enabled.
*/
#ifndef MCP2515_CAN_2_B_SUPPORT
    #define MCP2515_CAN_2_B_SUPPORT         0
#endif // MCP2515_CAN_2_B_SUPPORT

/*! Determines whether error callbacks should be provided by the driver.
**  Disabling this option will speed up message handling.
*/
#ifndef MCP2515_ERROR_CALLBACK_SUPPORT
    #define MCP2515_ERROR_CALLBACK_SUPPORT  0
#endif

//! Chip Select (port,pin) for MCP2515 (required)
#ifndef MCP2515_CS
    #define MCP2515_CS              B,4
#endif // MCP2515_CS

//! Main interrupt (port,pin) for MCP2515 (required)
#ifndef MCP2515_INT_MAIN
    #define MCP2515_INT_MAIN        B,2 // pin for external interrupt
    #define MCP2515_INTNO_MAIN      2   // corresponding external interrupt number
#endif

//! Specify whether MCP2515 RX buffer interrupt lines are connected
#ifndef MCP2515_USE_RX_INT
    #define MCP2515_USE_RX_INT      0   // disabled by default
#endif

//! If RX buffer interrupt lines are connected, specify the interrupt for RX buffer 0:
#ifndef MCP2515_INT_RXB0
    #define MCP2515_INT_RXB0        D,2 // pin for external interrupt
    #define MCP2515_INTNO_RXB0      0   // corresponding external interrupt number
#endif

//! If RX buffer interrupt lines are connected, specify the interrupt for RX buffer 1:
#ifndef MCP2515_INT_RXB1
    #define MCP2515_INT_RXB1        D,3 // pin for external interrupt
    #define MCP2515_INTNO_RXB1      1   // corresponding external interrupt number
#endif

//! MCP2515 debug mode switch
#ifndef MCP2515_DEBUG
    #define MCP2515_DEBUG           0
#endif // MCP2515_DEBUG

//! print label
#ifndef MCP2515_LABEL
    #define MCP2515_LABEL           "[MCP2515] "
#endif // MCP2515_LABEL

//! debug print label
#ifndef MCP2515_LABEL_DEBUG
    #define MCP2515_LABEL_DEBUG     "[MCP2515/dbg] "
#endif // MCP2515_LABEL_DEBUG


//*****************************************************************************
//*********************** MCP2515 SPECIFIC ERROR CODES ************************
//*****************************************************************************

/*! MCP2515 specific error base */
#define MCP2515_ERR_BASE                            0

/*! MCP2515 returns with no errors. */
#define MCP2515_OK                                  0

/*! A bad parameter has been passed. */
#define MCP2515_ERR_BAD_PARAMETER                   MCP2515_ERR_BASE - 1

/*! The SPI driver has not been initialized. */
#define MCP2515_ERR_SPI_NOT_INITIALIZED             MCP2515_ERR_BASE - 2

/*! The MCP2515 driver has already been initialized. */
#define MCP2515_ERR_ALREADY_INITIALIZED             MCP2515_ERR_BASE - 3

/*! Register verification failed. */
#define MCP2515_ERR_VERIFY_FAIL                     MCP2515_ERR_BASE - 4

/*! There is no transmit buffer free. */
#define MCP2515_ERR_NO_TRANSMIT_BUFFER_FREE         MCP2515_ERR_BASE - 5

/*! There has no message been received. */
#define MCP2515_ERR_NO_MESSAGE_RECEIVED             MCP2515_ERR_BASE - 6


//*****************************************************************************
//******************************** DATA TYPES *********************************
//*****************************************************************************

/*!
*******************************************************************************
** \brief   Defines possible synchronisation jump width values.
**
** \sa      MCP 2515-I-P.pdf Chapter 5 pages 38 and 39
*******************************************************************************
*/
typedef enum
{
    MCP2515_SJW_1TQ = 0,
    MCP2515_SJW_2TQ,
    MCP2515_SJW_3TQ,
    MCP2515_SJW_4TQ
} MCP2515_SynchronisationJumpWidthT;

/*!
*******************************************************************************
** \brief   Defines whether to use one or three sample points.
**
**          In case of one sample point, the sample point is located at the end
**          of phase segment 1.
**          In case of three sample points, while the bit is still sampled at
**          the end of phase segment 1, two additional samples are taken at
**          one-half TQ intervals prior to the end of phase segment 1, with the
**          value of the bit being determined by a majority decision.

** \sa      MCP 2515-I-P.pdf Chapter 5, page 38
*******************************************************************************
*/
typedef enum
{
    MCP2515_SAM_1 = 0,
    MCP2515_SAM_3
} MCP2515_SamplePointCountT;

/*!
*******************************************************************************
** \brief   Defines the length of the propagation segment in terms of TQ.
**
**          The propagation segment is programmable from 1 - 8 TQ.
**
** \sa      MCP 2515-I-P.pdf Chapter 5, page 38
*******************************************************************************
*/
typedef enum
{
    MCP2515_PRSEG_1TQ = 0,
    MCP2515_PRSEG_2TQ,
    MCP2515_PRSEG_3TQ,
    MCP2515_PRSEG_4TQ,
    MCP2515_PRSEG_5TQ,
    MCP2515_PRSEG_6TQ,
    MCP2515_PRSEG_7TQ,
    MCP2515_PRSEG_8TQ
} MCP2515_PropagationSegmentLengthT;

/*!
*******************************************************************************
** \brief   Defines the length of phase segment 1 in terms of TQ.
**
**          Phase segment 1 is programmable from 1 - 8 TQ.
**
** \sa      MCP 2515-I-P.pdf Chapter 5, page 38
*******************************************************************************
*/
typedef enum
{
    MCP2515_PHSEG1_1TQ = 0,
    MCP2515_PHSEG1_2TQ,
    MCP2515_PHSEG1_3TQ,
    MCP2515_PHSEG1_4TQ,
    MCP2515_PHSEG1_5TQ,
    MCP2515_PHSEG1_6TQ,
    MCP2515_PHSEG1_7TQ,
    MCP2515_PHSEG1_8TQ
} MCP2515_PhaseSegment1LengthT;

/*!
*******************************************************************************
** \brief   Defines the length of phase segment 2 in terms of TQ.
**
**          Phase segment 2 is programmable from 2 - 8 TQ.
**
** \sa      MCP 2515-I-P.pdf Chapter 5, page 38
*******************************************************************************
*/
typedef enum
{
    MCP2515_PHSEG2_2TQ = 1,
    MCP2515_PHSEG2_3TQ,
    MCP2515_PHSEG2_4TQ,
    MCP2515_PHSEG2_5TQ,
    MCP2515_PHSEG2_6TQ,
    MCP2515_PHSEG2_7TQ,
    MCP2515_PHSEG2_8TQ
} MCP2515_PhaseSegment2LengthT;

/*!
*******************************************************************************
** \brief   Defines whether the receive buffer 0 is allowed to pass messages
**          into receive buffer 1. This option helps avoiding an receive
**          buffer overflow when receive buffer 0 is full and receive buffer 1
**          is empty.
**
** \sa      MCP 2515-I-P.pdf 4.2.1
*******************************************************************************
*/
typedef enum
{
    MCP2515_ROLLOVER_DISABLE = 0,
    MCP2515_ROLLOVER_ENABLE  = 1
} MCP2515_RolloverModeT;

/*!
*******************************************************************************
** \brief   Defines whether a transmission is attempted only once.
**
**          If the oneshot mode is enabled and a message in a transmit buffer
**          fails the bus arbitration, it will be discarded and the buffer is
**          free again.
**
** \sa      MCP 2515-I-P.pdf 3.4
*******************************************************************************
*/
typedef enum
{
    MCP2515_ONESHOT_DISABLE = 0,
    MCP2515_ONESHOT_ENABLE  = 1
} MCP2515_OneShotModeT;

/*!
*******************************************************************************
** \brief   This structure provides the CAN 2.0A header bits.
**
** \sa      MCP 2515-I-P.pdf 4.2.2
*******************************************************************************
*/
typedef struct
{
    uint16_t standardIdBits : 11;
} MCP2515_CAN2aHeaderBitsT;

/*!
*******************************************************************************
** \brief   This structure provides the CAN 2.0B header bits.
**
** \sa      MCP 2515-I-P.pdf 4.2.2
*******************************************************************************
*/
typedef struct
{
    uint32_t standardIdBits : 11;
    uint32_t extendedIdEnbl :  1;
    uint32_t extendedIdBits : 18;
} MCP2515_CAN2bHeaderBitsT;

/*!
*******************************************************************************
** \brief   Initialization parameters for the MCP2515 in CAN2.0A configuration.
**
**          The baudRatePrescaler (BRP) is a programmable prescaler which
**          defines the Time Quantum period (TQ) with respect to the frequency
**          of the connected oscillator (F_MCP2515).
**          TQ = 2 * (BRP + 1) / F_MCP2515
**
**          See mcp2515_config.h for automatic configuration settings of the
**          bit time generator. You want to use MCP2515_AUTO_BRP,
**          MCP2515_AUTO_SJW, MCP2515_AUTO_PRSEG, MCP2515_AUTO_PHSEG1,
**          and MCP2515_AUTO_PHSEG2 for configuration.
**
** \sa      MCP 2515-I-P.pdf Chapter 5, page 38
*******************************************************************************
*/
typedef struct
{
    uint8_t                             initSPI                  :  1; //!< defines whether the driver inits the SPI
    uint8_t                             wakeupLowPassFilter      :  1; //!< see MCP2515-I-P.pdf 10.2
    uint8_t                             baudRatePrescaler        :  5; //!< see MCP2515-I-P.pdf chapter 5
    MCP2515_SynchronisationJumpWidthT   synchronisationJumpWidth :  2; //!< see MCP2515-I-P.pdf chapter 5
    MCP2515_PropagationSegmentLengthT   propagationSegmentLength :  3; //!< see MCP2515-I-P.pdf chapter 5
    MCP2515_PhaseSegment1LengthT        phaseSegment1Length      :  3; //!< see MCP2515-I-P.pdf chapter 5
    MCP2515_PhaseSegment2LengthT        phaseSegment2Length      :  3; //!< see MCP2515-I-P.pdf chapter 5
    MCP2515_SamplePointCountT           samplePointCount         :  1; //!< see MCP2515-I-P.pdf chapter 5
    MCP2515_RolloverModeT               rolloverMode             :  1; //!< see MCP2515-I-P.pdf 4.2.1
    MCP2515_OneShotModeT                oneShotMode              :  1; //!< see MCP2515-I-P.pdf 3.3
    uint16_t                            rxBuffer0Mask            : 11; //!< see MCP2515-I-P.pdf 4.5
    uint16_t                            rxBuffer0Filter0         : 11; //!< see MCP2515-I-P.pdf 4.5
    uint16_t                            rxBuffer0Filter1         : 11; //!< see MCP2515-I-P.pdf 4.5
    uint16_t                            rxBuffer1Mask            : 11; //!< see MCP2515-I-P.pdf 4.5
    uint16_t                            rxBuffer1Filter2         : 11; //!< see MCP2515-I-P.pdf 4.5
    uint16_t                            rxBuffer1Filter3         : 11; //!< see MCP2515-I-P.pdf 4.5
    uint16_t                            rxBuffer1Filter4         : 11; //!< see MCP2515-I-P.pdf 4.5
    uint16_t                            rxBuffer1Filter5         : 11; //!< see MCP2515-I-P.pdf 4.5
} MCP2515_CanAInitParamsT;

/*!
*******************************************************************************
** \brief   Initialization parameters for the MCP2515 in CAN2.0B configuration.
**
**          The baudRatePrescaler (BRP) is a programmable prescaler which
**          defines the Time Quantum period (TQ) with respect to the frequency
**          of the connected oscillator (F_MCP2515).
**          TQ = 2 * (BRP + 1) / F_MCP2515
**
**          See mcp2515_config.h for automatic configuration settings of the
**          bit time generator. You want to use MCP2515_AUTO_BRP,
**          MCP2515_AUTO_SJW, MCP2515_AUTO_PRSEG, MCP2515_AUTO_PHSEG1,
**          and MCP2515_AUTO_PHSEG2 for configuration.
**
** \sa      MCP 2515-I-P.pdf Chapter 5, page 38
*******************************************************************************
*/
typedef struct
{
    uint8_t                             initSPI                  :  1; //!< defines whether the driver inits the SPI
    uint8_t                             wakeupLowPassFilter      :  1; //!< see MCP2515-I-P.pdf 10.2
    uint8_t                             baudRatePrescaler        :  5; //!< see MCP2515-I-P.pdf chapter 5
    MCP2515_SynchronisationJumpWidthT   synchronisationJumpWidth :  2; //!< see MCP2515-I-P.pdf chapter 5
    MCP2515_PropagationSegmentLengthT   propagationSegmentLength :  3; //!< see MCP2515-I-P.pdf chapter 5
    MCP2515_PhaseSegment1LengthT        phaseSegment1Length      :  3; //!< see MCP2515-I-P.pdf chapter 5
    MCP2515_PhaseSegment2LengthT        phaseSegment2Length      :  3; //!< see MCP2515-I-P.pdf chapter 5
    MCP2515_SamplePointCountT           samplePointCount         :  1; //!< see MCP2515-I-P.pdf chapter 5
    MCP2515_RolloverModeT               rolloverMode             :  1; //!< see MCP2515-I-P.pdf 4.2.1
    MCP2515_OneShotModeT                oneShotMode              :  1; //!< see MCP2515-I-P.pdf 3.3
    uint32_t                            rxBuffer0MaskSid         : 11; //!< see MCP2515-I-P.pdf 4.5
    uint32_t                            rxBuffer0MaskEid         : 18; //!< see MCP2515-I-P.pdf 4.5
    uint32_t                            rxBuffer0Filter0Sid      : 11; //!< see MCP2515-I-P.pdf 4.5
    uint32_t                            rxBuffer0Filter0Ext      :  1; //!< see MCP2515-I-P.pdf 4.5
    uint32_t                            rxBuffer0Filter0Eid      : 18; //!< see MCP2515-I-P.pdf 4.5
    uint32_t                            rxBuffer0Filter1Sid      : 11; //!< see MCP2515-I-P.pdf 4.5
    uint32_t                            rxBuffer0Filter1Ext      :  1; //!< see MCP2515-I-P.pdf 4.5
    uint32_t                            rxBuffer0Filter1Eid      : 18; //!< see MCP2515-I-P.pdf 4.5
    uint32_t                            rxBuffer1MaskSid         : 11; //!< see MCP2515-I-P.pdf 4.5
    uint32_t                            rxBuffer1MaskEid         : 18; //!< see MCP2515-I-P.pdf 4.5
    uint32_t                            rxBuffer1Filter2Sid      : 11; //!< see MCP2515-I-P.pdf 4.5
    uint32_t                            rxBuffer1Filter2Ext      :  1; //!< see MCP2515-I-P.pdf 4.5
    uint32_t                            rxBuffer1Filter2Eid      : 18; //!< see MCP2515-I-P.pdf 4.5
    uint32_t                            rxBuffer1Filter3Sid      : 11; //!< see MCP2515-I-P.pdf 4.5
    uint32_t                            rxBuffer1Filter3Ext      :  1; //!< see MCP2515-I-P.pdf 4.5
    uint32_t                            rxBuffer1Filter3Eid      : 18; //!< see MCP2515-I-P.pdf 4.5
    uint32_t                            rxBuffer1Filter4Sid      : 11; //!< see MCP2515-I-P.pdf 4.5
    uint32_t                            rxBuffer1Filter4Ext      :  1; //!< see MCP2515-I-P.pdf 4.5
    uint32_t                            rxBuffer1Filter4Eid      : 18; //!< see MCP2515-I-P.pdf 4.5
    uint32_t                            rxBuffer1Filter5Sid      : 11; //!< see MCP2515-I-P.pdf 4.5
    uint32_t                            rxBuffer1Filter5Ext      :  1; //!< see MCP2515-I-P.pdf 4.5
    uint32_t                            rxBuffer1Filter5Eid      : 18; //!< see MCP2515-I-P.pdf 4.5
} MCP2515_CanBInitParamsT;

/*!
*******************************************************************************
** \brief   Initialization parameters for the MCP2515.
**
**          It depends on the MCP2515_CAN_2_B_SUPPORT mode switch whether the
**          message is a CAN2.0A or CAN2.0B structure.
**
*******************************************************************************
*/
#if MCP2515_CAN_2_B_SUPPORT
typedef MCP2515_CanBInitParamsT MCP2515_InitParamsT;
#else
typedef MCP2515_CanAInitParamsT MCP2515_InitParamsT;
#endif // MCP2515_CAN_2_B_SUPPORT

/*!
*******************************************************************************
** \brief   CAN message structure with a standard identifier (CAN 2.0A).
**
*******************************************************************************
*/
typedef struct
{
    uint16_t sid        : 11; //!< standard identifier
    uint16_t rtr        :  1; //!< remote transmission request
    uint16_t dlc        :  4; //!< data length code
    uint8_t  dataArray[8];    //!< message data
} MCP2515_CanAMessageT;

/*!
*******************************************************************************
** \brief   CAN message structure with an extended identifier (CAN 2.0B).
**
*******************************************************************************
*/
typedef struct
{
    uint32_t sid        : 11; //!< standard identifier
    uint32_t ief        :  1; //!< identifier extended flag
    uint32_t eid        : 18; //!< extended identifier
    uint8_t  rtr        :  1; //!< remote transmission request
    uint8_t  dlc        :  4; //!< data length code
    uint8_t  dataArray[8];    //!< message data
} MCP2515_CanBMessageT;

/*!
*******************************************************************************
** \brief   CAN message structure.
**
**          It depends on the MCP2515_CAN_2_B_SUPPORT mode switch whether the
**          message is a CAN2.0A or CAN2.0B structure.
**
*******************************************************************************
*/
#if MCP2515_CAN_2_B_SUPPORT
typedef MCP2515_CanBMessageT MCP2515_CanMessageT;
#else
typedef MCP2515_CanAMessageT MCP2515_CanMessageT;
#endif // MCP2515_CAN_2_B_SUPPORT

/*!
*******************************************************************************
**  \brief  Enumeration of the three transmit buffers.
**
*******************************************************************************
*/
typedef enum
{
    MCP2515_TX_BUFFER_0 = (uint8_t)0x01,
    MCP2515_TX_BUFFER_1 = (uint8_t)0x02,
    MCP2515_TX_BUFFER_2 = (uint8_t)0x04
} MCP2515_TxBufferIdT;

/*!
*******************************************************************************
**  \brief  Enumeration of the transmit priorities.
**          The higher the number, the higher the priority.
**
*******************************************************************************
*/
typedef enum
{
    MCP2515_TX_PRIORITY_0 = 0,
    MCP2515_TX_PRIORITY_1 = 1,
    MCP2515_TX_PRIORITY_2 = 2,
    MCP2515_TX_PRIORITY_3 = 3
} MCP2515_TxPriorityT;

/*!
*******************************************************************************
**  \brief  This structure holds a bitfield bufferId which may contain
**          MCP2515_TxBufferIdT buffer IDs. It is allowed to specify the
**          bufferId as the bitwise OR of several MCP2515_TxBufferIdT IDs.
**          The priority determines a transmission priority
**          (see MCP2515_TxPriorityT).
**
*******************************************************************************
*/
typedef struct
{
    uint8_t bufferId : 3;
    MCP2515_TxPriorityT priority : 2;
} MCP2515_TxParamsT;


/*!
*******************************************************************************
** \brief   Interface for receiver callback
*******************************************************************************
*/
typedef void (*MCP2515_RxCallbackT) (MCP2515_CanMessageT* canMessagePtr);

/*!
*******************************************************************************
** \brief   Interface for transmitter callback
*******************************************************************************
*/
typedef void (*MCP2515_TxCallbackT) (MCP2515_TxBufferIdT bufferId);

/*!
*******************************************************************************
** \brief   Interface for callback with void argument
*******************************************************************************
*/
typedef void (*MCP2515_VoidCallbackT) (void);

/*!
*******************************************************************************
** \brief   Interface for error callback
**
**          The error callback accepts an uin8_t as argument which is a bitfield
**          with the following flags:
**          bit 0: EWARN: (TXWAR == 1) or (RXWAR == 1)
**          bit 1: RXWAR: Receive error warning flag (REC >= 96)
**          bit 2: TXWAR: Transmit error warning flag (TEC >= 96)
**          bit 3: RXEP: Receive error-passive flag (REC >= 128)
**          bit 4: TXEP: Transmit error-passive flag (TEC >= 128)
**          bit 5: TXBO: Bus off
**          bit 6: RX0OVR: Receive buffer 0 overflow bit
**          bit 7: RX1OVR: Receive buffer 1 overflow bit
**
*******************************************************************************
*/
typedef void (*MCP2515_ErrorCallbackT) (uint8_t errState);


//*****************************************************************************
//************************* FUNCTION DECLARATIONS *****************************
//*****************************************************************************

int8_t  MCP2515_Init(const MCP2515_InitParamsT* initParamsPtr);
int8_t  MCP2515_Exit(void);
void    MCP2515_SetRxCallback(MCP2515_RxCallbackT rxCallback);
void    MCP2515_SetTxCallback(MCP2515_TxCallbackT txCallback);
int8_t  MCP2515_Transmit(MCP2515_CanMessageT* messagePtr, \
                         MCP2515_TxParamsT txParams);

#if MCP2515_ERROR_CALLBACK_SUPPORT
void    MCP2515_SetMessageErrorCallback(MCP2515_VoidCallbackT callback);
void    MCP2515_SetWakeupCallback(MCP2515_VoidCallbackT callback);
void    MCP2515_SetErrorCallback(MCP2515_ErrorCallbackT callback);
#endif // MCP2515_ERROR_CALLBACK_SUPPORT

#endif // MCP2515_H
