/*!
*******************************************************************************
*******************************************************************************
** \brief   MCP2515 CAN bus driver for AVR microcontrollers
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
#include <util/delay.h>
#include <drivers/macros_pin.h>
#include <drivers/spi_m.h>
#include "mcp2515.h"
#include "mcp2515_priv.h"
#include "mcp2515_config.h"

#if MCP2515_DEBUG
#include <stdio.h>
#endif // MCP2515_DEBUG

//*****************************************************************************
//*************************** DEFINES AND MACROS ******************************
//*****************************************************************************

// Critical Section handling by disabling the corresponding interrupts:
#if MCP2515_USE_RX_INT
#define MCP2515_ENTER_CS    EIMSK &= ~(1 << MCP2515_INTNO_MAIN)
#define MCP2515_LEAVE_CS    EIMSK |=  (1 << MCP2515_INTNO_MAIN)
#else
#define MCP2515_ENTER_CS    EIMSK &= ~((1 << MCP2515_INTNO_MAIN) | \
                                       (1 << MCP2515_INTNO_RXB0) | \
                                       (1 << MCP2515_INTNO_RXB1));
#define MCP2515_LEAVE_CS    EIMSK |=  ((1 << MCP2515_INTNO_MAIN) | \
                                       (mcp2515State.rxIrqEnable << MCP2515_INTNO_RXB0) | \
                                       (mcp2515State.rxIrqEnable << MCP2515_INTNO_RXB1));
#endif // MCP2515_USE_RX_INT

// Debugging print:
#if MCP2515_DEBUG
#define PRINT_DEBUG(arg)    printf("\n" MCP2515_LABEL_DEBUG); printf(arg)
#else
#define PRINT_DEBUG(arg)
#endif // MCP2515_DEBUG

// Interrupt vector macros:

#if   MCP2515_INTNO_MAIN == 0
#define MCP2515_INT_MAIN_vect INT0_vect
#elif MCP2515_INTNO_MAIN == 1
#define MCP2515_INT_MAIN_vect INT1_vect
#elif MCP2515_INTNO_MAIN == 2
#define MCP2515_INT_MAIN_vect INT2_vect
#else
#define MCP2515_INT_MAIN_vect BADISR_vect
#endif

#if   MCP2515_INTNO_RXB0 == 0
#define MCP2515_INT_RXB0_vect INT0_vect
#elif MCP2515_INTNO_RXB0 == 1
#define MCP2515_INT_RXB0_vect INT1_vect
#elif MCP2515_INTNO_RXB0 == 2
#define MCP2515_INT_RXB0_vect INT2_vect
#else
#define MCP2515_INT_RXB0_vect BADISR_vect
#endif

#if   MCP2515_INTNO_RXB1 == 0
#define MCP2515_INT_RXB1_vect INT0_vect
#elif MCP2515_INTNO_RXB1 == 1
#define MCP2515_INT_RXB1_vect INT1_vect
#elif MCP2515_INTNO_RXB1 == 2
#define MCP2515_INT_RXB1_vect INT2_vect
#else
#define MCP2515_INT_RXB1_vect BADISR_vect
#endif

//******** ATmega16 setup ********

#ifdef atmega16
    
    /* NOTE: The registers of atmega16 are not bit-compatible to the 
    **       corresponding registers of atmega644 and atmega644p.
    **       In order to write to GICR on atmega16, the macros
    **       MCP2515_INTNO_MAIN, MCP2515_INTNO_RXB0, and MCP2515_INTNO_RXB1
    **       should be set to 6, 7, and 5, respectively.
    **       However, MCUCR allows to configure interrupt sense control 
    **       only for INT0 and INT1 (the same bits as for atmega644p), 
    **       but not for INT2. 
    */
    #define EIMSK   GICR
    #define EICRA   MCUCR

#endif // atmega16

//*****************************************************************************
//**************************** LOCAL DATA TYPES *******************************
//*****************************************************************************


//*****************************************************************************
//**************************** LOCAL VARIABLES ********************************
//*****************************************************************************

static MCP2515_RxCallbackT    mcp2515RxCallback           = NULL;
static MCP2515_TxCallbackT    mcp2515TxCallback           = NULL;

#if MCP2515_ERROR_CALLBACK_SUPPORT
static MCP2515_VoidCallbackT  mcp2515MessageErrorCallback = NULL;
static MCP2515_VoidCallbackT  mcp2515WakeupCallback       = NULL;
static MCP2515_ErrorCallbackT mcp2515ErrorCallback        = NULL;
#endif // MCP2515_ERROR_CALLBACK_SUPPORT

//! Internal state of the MCP2515 driver.
static struct
{
    uint8_t             initialized  : 1; //!< set while the MCP2515 is initialized
    uint8_t             rxIrqEnable  : 1; //!< determines RX interrupt enable
    MCP2515_TxPriorityT txb0Priority : 2; //!< current tx buffer 0 priority level
    MCP2515_TxPriorityT txb1Priority : 2; //!< current tx buffer 1 priority level
    MCP2515_TxPriorityT txb2Priority : 2; //!< current tx buffer 2 priority level
} mcp2515State;

//*****************************************************************************
//********************** LOCAL FUNCTION DECLARATIONS **************************
//*****************************************************************************

static inline void mcp2515CmdReadAddressBurst (uint8_t address, uint8_t num, uint8_t* destPtr);
static inline void mcp2515CmdWriteAddressBurst(uint8_t address, uint8_t num, uint8_t* srcPtr);
static inline void mcp2515CmdBitModify(uint8_t address, uint8_t mask, uint8_t data);

#if MCP2515_CAN_2_B_SUPPORT
static void mcp2515SetHeaderFormat(uint8_t address, \
                                   uint32_t sid,    \
                                   uint32_t ext,    \
                                   uint32_t eid);
#else
static void mcp2515SetHeaderFormat(uint8_t address, uint16_t sid);
#endif // MCP2515_CAN_2_B_SUPPORT

//*****************************************************************************
//**************************** LOCAL FUNCTIONS ********************************
//*****************************************************************************

/*!
*******************************************************************************
** \brief   Read registers from the MCP2515.
**
** \param   address
**              The address in the MCP2515 which will be read first.
** \param   num
**              The number of subsequent addresses that will be read.
** \param   destPtr
**              The buffer in which the read register values will be
**              written. Make sure that 'num' bytes are allocated for reception!
**
** \sa      MCP 2515-I-P.pdf 12.3
*******************************************************************************
*/
static inline void mcp2515CmdReadAddressBurst (uint8_t address, uint8_t num, uint8_t* destPtr)
{
    SET_LOW(MCP2515_CS);
    (void)SPI_M_Transceive(MCP2515_SPI_READ);
    (void)SPI_M_Transceive(address);
    while(num--)
    {
        *destPtr++ = SPI_M_Transceive(0xFF);
    }
    SET_HIGH(MCP2515_CS);
    return;
}

/*!
*******************************************************************************
** \brief   Write registers to the MCP2515.
**
** \param   address
**              The address in the MCP2515 which will be written first.
** \param   num
**              The number of subsequent addresses that will be written.
** \param   srcPtr
**              The buffer from which the values will be shifted out to the
**              MCP2515. It must provide 'num' valid register values.
**
** \sa      MCP2515-I-P.pdf 12.5
*******************************************************************************
*/
static inline void mcp2515CmdWriteAddressBurst(uint8_t address, uint8_t num, uint8_t* srcPtr)
{
    SET_LOW(MCP2515_CS);
    (void)SPI_M_Transceive(MCP2515_SPI_WRITE);
    (void)SPI_M_Transceive(address);
    while(num--)
    {
        (void)SPI_M_Transceive(*srcPtr++);
    }
    SET_HIGH(MCP2515_CS);
    return;
}

/*!
*******************************************************************************
** \brief   Modify bits in a register of the MCP2515.
**
** \param   address
**              The address of the register which will be modified.
** \param   mask
**              Masks those bits that will be modified.
**              All bits that are set to 0 will not be modified.
** \param   data
**              The new bit values.
**
** \sa      MCP2515-I-P.pdf 12.10
*******************************************************************************
*/
static inline void mcp2515CmdBitModify(uint8_t address, uint8_t mask, uint8_t data)
{
    SET_LOW(MCP2515_CS);
    (void)SPI_M_Transceive(MCP2515_SPI_BIT_MODIFY);
    (void)SPI_M_Transceive(address);
    (void)SPI_M_Transceive(mask);
    (void)SPI_M_Transceive(data);
    SET_HIGH(MCP2515_CS);
    return;
}

#if MCP2515_CAN_2_B_SUPPORT
/*!
*******************************************************************************
** \brief   Write flag or mask registers of the MCP2515.
**
**          All mask and filter registers are organized in the same layout.
**          This procedure stores a given mask or filter with standard ID,
**          extension flag and extended ID at the specified address.
**          Make sure that the device is in the configuration state when
**          using this procedure!
**
** \param   address
**              The address of the standard identifier high register.
** \param   sid
**              Standard ID.
** \param   ext
**              Extension flag.
** \param   eid
**              Extended ID.
**
** \sa      MCP2515-I-P.pdf 4.5
*******************************************************************************
*/
static void mcp2515SetHeaderFormat(uint8_t address, \
                                   uint32_t sid,    \
                                   uint32_t ext,    \
                                   uint32_t eid)
{
    SET_LOW(MCP2515_CS);
    (void)SPI_M_Transceive(MCP2515_SPI_WRITE);
    (void)SPI_M_Transceive(address);
    (void)SPI_M_Transceive(sid >> 3);
    (void)SPI_M_Transceive(((sid & 0x07) << 5) | (ext << 3) | (eid >> 16));
    (void)SPI_M_Transceive((eid >> 8) & 0xFF);
    (void)SPI_M_Transceive(eid & 0xFF);
    SET_HIGH(MCP2515_CS);
    return;
}

#else // MCP2515_CAN_2_B_SUPPORT

/*!
*******************************************************************************
** \brief   Write flag or mask registers of the MCP2515.
**
**          All mask and filter registers are organized in the same layout.
**          This procedure stores a given standard ID mask or filter at the
**          specified address. Extended ID related bits are forced to 0.
**          Make sure that the device is in the configuration state when
**          using this procedure!
**
** \param   address
**              The address of the standard identifier high register.
** \param   sid
**              The standard identifier mask or filter to use.
**
** \sa      MCP2515-I-P.pdf 4.5
*******************************************************************************
*/
static void mcp2515SetHeaderFormat(uint8_t address, uint16_t sid)
{
    SET_LOW(MCP2515_CS);
    (void)SPI_M_Transceive(MCP2515_SPI_WRITE);
    (void)SPI_M_Transceive(address);
    (void)SPI_M_Transceive((uint8_t) (sid >> 3));
    (void)SPI_M_Transceive((uint8_t) (sid & 0x0007) << 5);
    (void)SPI_M_Transceive(0x00);
    (void)SPI_M_Transceive(0x00);
    SET_HIGH(MCP2515_CS);
    return;
}
#endif // MCP2515_CAN_2_B_SUPPORT


//*****************************************************************************
//*************************** PUBLIC FUNCTIONS ********************************
//*****************************************************************************

/*!
*******************************************************************************
** \brief   Initializes the MCP2515 hardware with given parameters.
**
** \param   initParamsPtr
**              Points to a structure with all required initialization
**              parameters.
**
** \return
**          - #MCP2515_OK on success.
**          - #MCP2515_ERR_SPI_NOT_INITIALIZED if the SPI driver is not
**              initialized.
**          - #MCP2515_ERR_VERIFY_REGISTER if the verification
**              of the MCP2515_CNF1 register failed.
**
*******************************************************************************
*/
int8_t MCP2515_Init(const MCP2515_InitParamsT* initParamsPtr)
{
    uint8_t val;

    if(mcp2515State.initialized)
    {
        return(MCP2515_ERR_ALREADY_INITIALIZED);
    }
    // Disable interrupt(s):
    EIMSK &= ~(1 << MCP2515_INTNO_MAIN);
#if MCP2515_USE_RX_INT
    EIMSK &= ~((1 << MCP2515_INTNO_RXB0) | (1 << MCP2515_INTNO_RXB1));
#endif // MCP2515_USE_RX_INT

    // Initialize state variable:
    memset(&mcp2515State, 0, sizeof(mcp2515State));

    // initialize SPI
    if(initParamsPtr->initSPI)
    {
        (void)SPI_M_Init(MCP2515_SPI_CLOCK_DIVIDER,   \
                         MCP2515_SPI_DATA_ORDER,      \
                         MCP2515_SPI_CLOCK_PARITY,    \
                         MCP2515_SPI_CLOCK_PHASE);
    }

    // check if SPI is initialized:
    if(!SPI_M_IsInitialized())
    {
        return(MCP2515_ERR_SPI_NOT_INITIALIZED);
    }

    // set up CS line:
    SET_OUTPUT(MCP2515_CS);
    SET_HIGH(MCP2515_CS);

    // reset the MCP2515 and enter configuration mode
    // see MCP2515-I-P.pdf chapter 9.0 for RESET pin
    _delay_us(10);
    SET_LOW(MCP2515_CS);
    (void)SPI_M_Transceive(MCP2515_SPI_RESET);
    SET_HIGH(MCP2515_CS);
    _delay_us(10);

    // set up bit timing configuration registers:
    // set up CNF1:
    val  = ((uint8_t)initParamsPtr->synchronisationJumpWidth) << 6;
    val |= initParamsPtr->baudRatePrescaler;
    mcp2515CmdWriteAddressBurst(MCP2515_CNF1, 1, &val);
    // set up CNF2:
    val  = (1 << MCP2515_BTLMODE); // see MCP2515-I-P.pdf 5.5
    val |= ((uint8_t)initParamsPtr->samplePointCount) << 6;
    val |= ((uint8_t)initParamsPtr->phaseSegment1Length) << 3;
    val |= ((uint8_t)initParamsPtr->propagationSegmentLength);
    mcp2515CmdWriteAddressBurst(MCP2515_CNF2, 1, &val);
    // set up CNF3:
    val  = ((uint8_t)initParamsPtr->wakeupLowPassFilter) << 6;
    val |= ((uint8_t)initParamsPtr->phaseSegment2Length);
    mcp2515CmdWriteAddressBurst(MCP2515_CNF3, 1, &val);
    // Verify CNF1 register:
    mcp2515CmdReadAddressBurst(MCP2515_CNF1, 1, &val);
    if(val != ((((uint8_t)initParamsPtr->synchronisationJumpWidth) << 6) | \
               initParamsPtr->baudRatePrescaler))
    {
#if MCP2515_DEBUG
        printf("CNF1 = 0x%02x\n", val);
        printf("expected: 0x%02x\n", \
              ((((uint8_t)initParamsPtr->synchronisationJumpWidth) << 6) | \
               initParamsPtr->baudRatePrescaler));
#endif // MCP2515_DEBUG
        return(MCP2515_ERR_VERIFY_FAIL);
    }
    if(mcp2515RxCallback)
    {
        mcp2515State.rxIrqEnable = 1;
    }

    // enable interrupts: see MCP2515-I-P.pdf chapter 7
    val = 0x00;
#if MCP2515_ERROR_CALLBACK_SUPPORT
    if(mcp2515MessageErrorCallback)
    {
        val |= (1 << MCP2515_MERRE);
    }
    if(mcp2515WakeupCallback)
    {
        val |=  (1 << MCP2515_WAKIE);
    }
    if(mcp2515ErrorCallback)
    {
        val |= (1 << MCP2515_ERRIE);
    }
#endif // MCP2515_ERROR_CALLBACK_SUPPORT
    if(mcp2515TxCallback)
    {
        val |= (1 << MCP2515_TX2IE) | \
               (1 << MCP2515_TX1IE) | \
               (1 << MCP2515_TX0IE);
    }
#if !MCP2515_USE_RX_INT
    if(mcp2515State.rxIrqEnable)
    {
        val |= (1 << MCP2515_RX1IE) | \
               (1 << MCP2515_RX0IE);
    }
#endif // !MCP2515_USE_RX_INT

    mcp2515CmdWriteAddressBurst(MCP2515_CANINTE, 1, &val);

    // configure RXnBF pins (see MCP2515-I-P.pdf 4.4)
#if MCP2515_USE_RX_INT // enable RX interrupt pins
    val = (1 << MCP2515_B0BFM) |
          (1 << MCP2515_B1BFM) |
          (1 << MCP2515_B0BFE) |
          (1 << MCP2515_B1BFE);
#else
    val = 0x00; // disable RX interrupt pins
#endif // MCP2515_USE_RX_INT
    mcp2515CmdWriteAddressBurst(MCP2515_BFPCTRL, 1, &val);

    // disable TXnRTS pins (not used) - see MCP2515-I-P.pdf chapter 3.5
    val = 0x00;
    mcp2515CmdWriteAddressBurst(MCP2515_TXRTSCTRL, 1, &val);

    // set up message reception mode - see MCP2515-I-P.pdf 4.2.1 and 4.2.2
#if MCP2515_CAN_2_B_SUPPORT
    val = 0x00; // standard and extended frames
#else
    val = 0x01 << MCP2515_RXM0; // standard frames only
#endif // MCP2515_CAN_2_B_SUPPORT
    mcp2515CmdWriteAddressBurst(MCP2515_RXB1CTRL, 1, &val);
    val |= ((uint8_t)initParamsPtr->rolloverMode) << MCP2515_BUKT;
    mcp2515CmdWriteAddressBurst(MCP2515_RXB0CTRL, 1, &val);

    // set up filters and masks - see MCP2515-I-P.pdf 4.5
#if MCP2515_CAN_2_B_SUPPORT
    mcp2515SetHeaderFormat(MCP2515_RXM0SIDH, \
                           initParamsPtr->rxBuffer0MaskSid, 0, \
                           initParamsPtr->rxBuffer0MaskEid);
    mcp2515SetHeaderFormat(MCP2515_RXF0SIDH, \
                           initParamsPtr->rxBuffer0Filter0Sid, \
                           initParamsPtr->rxBuffer0Filter0Ext, \
                           initParamsPtr->rxBuffer0Filter0Eid);
    mcp2515SetHeaderFormat(MCP2515_RXF1SIDH, \
                           initParamsPtr->rxBuffer0Filter1Sid, \
                           initParamsPtr->rxBuffer0Filter1Ext, \
                           initParamsPtr->rxBuffer0Filter1Eid);
    mcp2515SetHeaderFormat(MCP2515_RXM1SIDH, \
                           initParamsPtr->rxBuffer1MaskSid, 0, \
                           initParamsPtr->rxBuffer1MaskEid);
    mcp2515SetHeaderFormat(MCP2515_RXF2SIDH, \
                           initParamsPtr->rxBuffer1Filter2Sid, \
                           initParamsPtr->rxBuffer1Filter2Ext, \
                           initParamsPtr->rxBuffer1Filter2Eid);
    mcp2515SetHeaderFormat(MCP2515_RXF3SIDH, \
                           initParamsPtr->rxBuffer1Filter3Sid, \
                           initParamsPtr->rxBuffer1Filter3Ext, \
                           initParamsPtr->rxBuffer1Filter3Eid);
    mcp2515SetHeaderFormat(MCP2515_RXF4SIDH, \
                           initParamsPtr->rxBuffer1Filter4Sid, \
                           initParamsPtr->rxBuffer1Filter4Ext, \
                           initParamsPtr->rxBuffer1Filter4Eid);
    mcp2515SetHeaderFormat(MCP2515_RXF5SIDH, \
                           initParamsPtr->rxBuffer1Filter5Sid, \
                           initParamsPtr->rxBuffer1Filter5Ext, \
                           initParamsPtr->rxBuffer1Filter5Eid);
#else // MCP2515_CAN_2_B_SUPPORT
    mcp2515SetHeaderFormat(MCP2515_RXM0SIDH, initParamsPtr->rxBuffer0Mask);
    mcp2515SetHeaderFormat(MCP2515_RXF0SIDH, initParamsPtr->rxBuffer0Filter0);
    mcp2515SetHeaderFormat(MCP2515_RXF1SIDH, initParamsPtr->rxBuffer0Filter1);
    mcp2515SetHeaderFormat(MCP2515_RXM1SIDH, initParamsPtr->rxBuffer1Mask);
    mcp2515SetHeaderFormat(MCP2515_RXF2SIDH, initParamsPtr->rxBuffer1Filter2);
    mcp2515SetHeaderFormat(MCP2515_RXF3SIDH, initParamsPtr->rxBuffer1Filter3);
    mcp2515SetHeaderFormat(MCP2515_RXF4SIDH, initParamsPtr->rxBuffer1Filter4);
    mcp2515SetHeaderFormat(MCP2515_RXF5SIDH, initParamsPtr->rxBuffer1Filter5);
#endif // MCP2515_CAN_2_B_SUPPORT

    // enter normal operation mode and set up one shot mode, disable CLKOUT pin:
    // see MCP2515-I-P.pdf page 58
    val = (initParamsPtr->oneShotMode << MCP2515_OSM);
    mcp2515CmdWriteAddressBurst(MCP2515_CANCTRL, 1, &val);

    // clear error flags
    /*
    val = 0x00;
    mcp2515CmdWriteAddressBurst(MCP2515_EFLG, 1, &val);
    mcp2515CmdWriteAddressBurst(MCP2515_CANINTF, 1, &val);
    */

    mcp2515State.initialized = 1;

    // set up main interrupt line and enable interrupt:
    SET_INPUT(MCP2515_INT_MAIN);
    SET_HIGH(MCP2515_INT_MAIN); // activate pullup for interrupt line
    EICRA &= ~(0x03 << (MCP2515_INTNO_MAIN * 2)); // Interrupt Sense Control, see ATmega644 11.1.1
    // EIFR  |= (1 << MCP2515_INTNO_MAIN); // not necessary for level interrupts, see ATmega644 11.1.3
    EIMSK |= (1 << MCP2515_INTNO_MAIN); // enable external interrupt

#if MCP2515_USE_RX_INT
    // set up optional RX buffer interrupt lines and enable interrupts:
    SET_INPUT(MCP2515_INT_RXB0);
    SET_HIGH(MCP2515_INT_RXB0);
    SET_INPUT(MCP2515_INT_RXB1);
    SET_HIGH(MCP2515_INT_RXB1);
    EICRA &= ~((0x03 << (MCP2515_INTNO_RXB0 * 2)) | \
               (0x03 << (MCP2515_INTNO_RXB1 * 2)));
    if(mcp2515State.rxIrqEnable)
    {
        EIMSK |= ((1 << MCP2515_INTNO_RXB0) | (1 << MCP2515_INTNO_RXB1));
    }
#endif // MCP2515_USE_RX_INT
    return (MCP2515_OK);
}

/*!
*******************************************************************************
** \brief   Exit the driver.
**
**          The MCP2515 CAN controller is reset. All pending transmissions
**          are aborted. All connected interrupts are disabled.
**
** \return
**          - #MCP2515_OK on success.
**
*******************************************************************************
*/
int8_t  MCP2515_Exit(void)
{
    uint8_t val;

    if(!mcp2515State.initialized)
    {
        return(MCP2515_OK);
    }
    // Disable interrupt(s):
    EIMSK &= ~(1 << MCP2515_INTNO_MAIN);
#if MCP2515_USE_RX_INT
    EIMSK &= ~((1 << MCP2515_INTNO_RXB0) | (1 << MCP2515_INTNO_RXB1));
#endif // MCP2515_USE_RX_INT

    // abort all transmissions:
    val = (1 << MCP2515_ABAT);
    mcp2515CmdWriteAddressBurst(MCP2515_CANCTRL, 1, &val);

    // reset MCP2515 device:
    SET_LOW(MCP2515_CS);
    (void)SPI_M_Transceive(MCP2515_SPI_RESET);
    SET_HIGH(MCP2515_CS);
    _delay_us(10);

    // clear callbacks and state:
    mcp2515RxCallback           = NULL;
    mcp2515TxCallback           = NULL;

#if MCP2515_ERROR_CALLBACK_SUPPORT
    mcp2515MessageErrorCallback = NULL;
    mcp2515WakeupCallback       = NULL;
    mcp2515ErrorCallback        = NULL;
#endif // MCP2515_ERROR_CALLBACK_SUPPORT

    memset(&mcp2515State, 0, sizeof(mcp2515State));

    // disable clock output pin (power saving) and enter sleep mode:
    mcp2515CmdBitModify(MCP2515_CANCTRL, \
                        (1 << MCP2515_REQOP0) | (1 << MCP2515_CLKEN), \
                        (1 << MCP2515_REQOP0) );

    // BFPCTRL output pins already set to high-impedance by default (reset)
    return(MCP2515_OK);
}

/*!
*******************************************************************************
** \brief   Set the receiver callback function.
**
** \param   rxCallback
**              The callback function that will be registered with the driver.
**
**          The receiver callback function will be executed every time a CAN
**          message is received by the MCP2515 device.
**          The callback function runs in interrupt context and is therefore
**          time critical. However, nested interrupts are allowed for this
**          function so that other hardware blocks which use interrupts
**          may be utilized within the receiver callback.
**          Its argument is a pointer to the received message and is only
**          valid during the runtime of this function. If you need to access
**          the received message at a later date you will have to copy the
**          received message into a buffer of the application.
**          It is recommended to set the receiver callback before
**          initialization of the driver. However, it may still be modified
**          during runtime.
**          It is allowed to set the rxCallback to NULL. The receive interrupt
**          will be disabled then accordingly.
**
*******************************************************************************
*/
void MCP2515_SetRxCallback(MCP2515_RxCallbackT rxCallback)
{
#if MCP2515_USE_RX_INT
    MCP2515_ENTER_CS;
    mcp2515RxCallback = rxCallback;
    if(mcp2515State.initialized)
    {
        mcp2515State.rxIrqEnable = rxCallback ? 1 : 0;
    }
    MCP2515_LEAVE_CS;
    return;
#else
    uint8_t eimsk_save;

    // DO NOT use LEAVE_CS here, since interrupts may still be disabled
    eimsk_save = EIMSK;
    MCP2515_ENTER_CS;
    mcp2515RxCallback = rxCallback;
    if(mcp2515State.initialized)
    {
        mcp2515State.rxIrqEnable = rxCallback ? 1 : 0;

        // clear rx buffers
        mcp2515CmdBitModify( MCP2515_CANINTF, \
                            (1 << MCP2515_RX1IF) | (1 << MCP2515_RX0IF), 0);

        // modify interrupt mask
        mcp2515CmdBitModify( \
            MCP2515_CANINTE, \
            (1 << MCP2515_RX1IE) | (1 << MCP2515_RX0IE), \
            rxCallback ? (1 << MCP2515_RX1IE) | (1 << MCP2515_RX0IE) : 0);
    }
    EIMSK  = eimsk_save;
    return;
#endif // MCP2515_USE_RX_INT
}

/*!
*******************************************************************************
** \brief   Set the transmitter callback function.
**
** \param   txCallback
**              The callback function that will be registered with the driver.
**
**          The transmitter callback function will be executed every time a CAN
**          message has been transmitted successfully by the MCP2515 device.
**          The callback function runs in interrupt context and is therefore
**          time critical. However, nested interrupts are allowed for this
**          function so that other hardware blocks which use interrupts
**          may be utilized within the receiver callback.
**          Its argument is the ID of the transmit buffer which transmitted
**          its message.
**          It is recommended to set the transmitter callback before
**          initialization of the driver. However, it may still be modified
**          during runtime.
**          It is allowed to set the txCallback to NULL. The transmit interrupt
**          will be disabled then accordingly.
**
*******************************************************************************
*/
void MCP2515_SetTxCallback(MCP2515_TxCallbackT txCallback)
{
    uint8_t eimsk_save;

    // DO NOT use LEAVE_CS here, since interrupts may still be disabled
    eimsk_save = EIMSK;
    MCP2515_ENTER_CS;
    mcp2515TxCallback = txCallback;
    if(mcp2515State.initialized)
    {   // modify interrupt mask:
        mcp2515CmdBitModify( \
            MCP2515_CANINTE, \
            (1 << MCP2515_TX2IE) | (1 << MCP2515_TX1IE) | (1 << MCP2515_TX0IE), \
            txCallback ? \
            (1 << MCP2515_TX2IE) | (1 << MCP2515_TX1IE) | (1 << MCP2515_TX0IE) : 0);
    }
    EIMSK = eimsk_save;
    return;
}

/*!
*******************************************************************************
** \brief   Transmit a message over the CAN bus.
**
** \param   msgPtr
**              A pointer to the message which will be sent via CAN bus.
** \param   txParams
**              Transmit parameters.
**              The buffer member defines which buffer(s) may be used for
**              transmitting a message. It is allowed to use the bitwise OR of
**              several buffer IDs. The driver will then select one buffer
**              out of these. If all specified buffers are currently in use, a
**              corresponding error code will be returned.
**              The priority denotes the transmission priority of a message.
**              The transmit buffer with the highest priority will send the
**              next message on the CAN bus, independently of the content of
**              the CAN message (such as the message ID which is used as
**              priority only during bus arbitration). If several buffers with
**              pending messages have the same priority level, the buffer
**              with the highest ID will send first.
** \return
**          - A MCP2515_TxBufferIdT member, denoting the buffer to which
**            the message was loaded.
**          - MCP2515_ERR_NO_TRANSMIT_BUFFER_FREE
**            if all transmit buffers are occupied.
**
** \sa      MCP2515-I-P.pdf 12.8
**
*******************************************************************************
*/
int8_t MCP2515_Transmit(MCP2515_CanMessageT* messagePtr, \
                        MCP2515_TxParamsT txParams)
{
    uint8_t val, command, ii;
    MCP2515_TxPriorityT current_prio;

    // quickly read status bits:
    MCP2515_ENTER_CS;
    SET_LOW(MCP2515_CS);
    (void)SPI_M_Transceive(MCP2515_SPI_READ_STATUS);
    val = SPI_M_Transceive(0xFF);
    SET_HIGH(MCP2515_CS);

    // see MCP2515-I-P.pdf figure 12.8 for bit assignments
    if ((txParams.bufferId & MCP2515_TX_BUFFER_2) && \
        ((val & (1 << MCP2515_RS_TX2REQ)) == 0))
    {
        val = 2; // tx buffer index
        current_prio = mcp2515State.txb2Priority;
        command = MCP2515_SPI_WRITE_TXB2SIDH;
    }
    else if ((txParams.bufferId & MCP2515_TX_BUFFER_1) && \
             ((val & (1 << MCP2515_RS_TX1REQ)) == 0))
    {
        val = 1; // tx buffer index
        current_prio = mcp2515State.txb1Priority;
        command = MCP2515_SPI_WRITE_TXB1SIDH;
    }
    else if ((txParams.bufferId & MCP2515_TX_BUFFER_0) && \
             ((val & (1 << MCP2515_RS_TX0REQ)) == 0))
    {
        val = 0; // tx buffer index
        current_prio = mcp2515State.txb0Priority;
        command = MCP2515_SPI_WRITE_TXB0SIDH;
    }
    else
    {
        return(MCP2515_ERR_NO_TRANSMIT_BUFFER_FREE);
    }

    // transmit sid/eid, rtr, dlc and data:
    SET_LOW(MCP2515_CS);
    (void)SPI_M_Transceive(command);
#if MCP2515_CAN_2_B_SUPPORT
    (void)SPI_M_Transceive((messagePtr->sid >> 3) & 0xFF);
    if(messagePtr->ief)
    {
        (void)SPI_M_Transceive(((messagePtr->sid << 5) & 0xE0) | \
                               (1 << MCP2515_EXIDE) | \
                               ((messagePtr->eid >> 16) & 0x03));
        (void)SPI_M_Transceive((messagePtr->eid >> 8) & 0xFF);
        (void)SPI_M_Transceive(messagePtr->eid & 0xFF);
    }
    else
    {
        (void)SPI_M_Transceive((messagePtr->sid << 5) & 0xE0);
        (void)SPI_M_Transceive(0xFF); // override extended identifier
        (void)SPI_M_Transceive(0xFF); // override extended identifier
    }
    (void)SPI_M_Transceive((messagePtr->rtr << MCP2515_RTR) | \
                           (messagePtr->dlc & 0x0F));
#else // CAN 2.0A only
    (void)SPI_M_Transceive((messagePtr->sid >> 3) & 0xFF);
    (void)SPI_M_Transceive((messagePtr->sid << 5) & 0xE0);
    (void)SPI_M_Transceive(0xFF); // override extended identifier
    (void)SPI_M_Transceive(0xFF); // override extended identifier
    (void)SPI_M_Transceive((messagePtr->rtr << MCP2515_RTR) | \
                           (messagePtr->dlc & 0x0F));
#endif // MCP2515_CAN_2_B_SUPPORT
    if( ! messagePtr->rtr )
    {
        for(ii = 0; ii < messagePtr->dlc; ii++)
        {
            (void)SPI_M_Transceive(messagePtr->dataArray[ii]);
        }
    }
    SET_HIGH(MCP2515_CS);

    // Check if priority level is already correctly set, otherwise set it:
    if(current_prio == txParams.priority)
    {
        // set only ready-to-send bit:
        switch(val)
        {
            case(2):
                command = MCP2515_SPI_RTS_TXB2;
                break;
            case(1):
                command = MCP2515_SPI_RTS_TXB1;
                break;
            case(0):
                command = MCP2515_SPI_RTS_TXB0;
                break;
        }
        SET_LOW(MCP2515_CS);
        (void)SPI_M_Transceive(command);
        SET_HIGH(MCP2515_CS);
    }
    else
    {
        // set ready-to-send bit and transmit buffer priority:
        switch(val)
        {
            case(2):
                command = MCP2515_TXB2CTRL;
                mcp2515State.txb2Priority = txParams.priority;
                break;
            case(1):
                command = MCP2515_TXB1CTRL;
                mcp2515State.txb1Priority = txParams.priority;
                break;
            case(0):
                command = MCP2515_TXB0CTRL;
                mcp2515State.txb0Priority = txParams.priority;
                break;
        }
        SET_LOW(MCP2515_CS);
        (void)SPI_M_Transceive(MCP2515_SPI_WRITE);
        (void)SPI_M_Transceive(command);
        (void)SPI_M_Transceive((1 << MCP2515_TXREQ) | txParams.priority);
        SET_HIGH(MCP2515_CS);
    }
    MCP2515_LEAVE_CS;

    return((int8_t)val);
}

#if MCP2515_ERROR_CALLBACK_SUPPORT

/*!
*******************************************************************************
** \brief   Set the message error callback function.
**
** \param   callback
**              The callback function that will be registered with the driver.
**
**          The message error callback will be executed every time an error
**          was detected while transmitting or receiving a message.
**          This feature is intended to be used with automatic baud rate
**          detection.
**          The callback function runs in interrupt context and is therefore
**          time critical. However, nested interrupts are allowed for this
**          function so that other hardware blocks which use interrupts
**          may be utilized within the receiver callback.
**          It is recommended to set the message error callback before
**          initialization of the driver. However, it may still be modified
**          during runtime.
**          It is allowed to set the callback to NULL. The message error
**          interrupt will be disabled then accordingly.
**
*******************************************************************************
*/
void MCP2515_SetMessageErrorCallback(MCP2515_VoidCallbackT callback)
{
    uint8_t eimsk_save;

    // DO NOT use LEAVE_CS here, since interrupts may still be disabled
    eimsk_save = EIMSK;
    MCP2515_ENTER_CS;
    mcp2515MessageErrorCallback = callback;
    if(mcp2515State.initialized)
    {   // modify interrupt mask:
        mcp2515CmdBitModify(MCP2515_CANINTE, \
                    (1 << MCP2515_MERRE), callback ? 0xFF : 0x00);
    }
    EIMSK = eimsk_save;
    return;
}

/*!
*******************************************************************************
** \brief   Set the wakeup callback function.
**
** \param   callback
**              The callback function that will be registered with the driver.
**
**          The wakeup callback will be executed every time the MCP2515 device
**          is beeing woken up by an incoming CAN message.
**          The callback function runs in interrupt context and is therefore
**          time critical. However, nested interrupts are allowed for this
**          function so that other hardware blocks which use interrupts
**          may be utilized within the receiver callback.
**          It is recommended to set the wakeup callback before
**          initialization of the driver. However, it may still be modified
**          during runtime.
**          It is allowed to set the callback to NULL. The wakeup
**          interrupt will be disabled then accordingly.
**
*******************************************************************************
*/
void MCP2515_SetWakeupCallback(MCP2515_VoidCallbackT callback)
{
    uint8_t eimsk_save;

    // DO NOT use LEAVE_CS here, since interrupts may still be disabled
    eimsk_save = EIMSK;
    MCP2515_ENTER_CS;
    mcp2515WakeupCallback = callback;
    if(mcp2515State.initialized)
    {   // modify interrupt mask:
        mcp2515CmdBitModify(MCP2515_CANINTE, \
                    (1 << MCP2515_WAKIE), callback ? 0xFF : 0x00);
    }
    EIMSK = eimsk_save;
    return;
}

/*!
*******************************************************************************
** \brief   Set the error callback function.
**
** \param   callback
**              The callback function that will be registered with the driver.
**
**          The error callback will be executed every time a receiver buffer
**          overflow occurs or when the internal error state of the MCP2515
**          changes.
**          The callback function runs in interrupt context and is therefore
**          time critical. However, nested interrupts are allowed for this
**          function so that other hardware blocks which use interrupts
**          may be utilized within the receiver callback.
**          It is recommended to set the error callback before
**          initialization of the driver. However, it may still be modified
**          during runtime.
**          It is allowed to set the callback to NULL. The error
**          interrupt will be disabled then accordingly.
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
void MCP2515_SetErrorCallback(MCP2515_ErrorCallbackT callback)
{
    uint8_t eimsk_save;

    // DO NOT use LEAVE_CS here, since interrupts may still be disabled
    eimsk_save = EIMSK;
    MCP2515_ENTER_CS;
    mcp2515ErrorCallback = callback;
    if(mcp2515State.initialized)
    {   // modify interrupt mask:
        mcp2515CmdBitModify(MCP2515_CANINTE, \
                    (1 << MCP2515_ERRIE), callback ? 0xFF : 0x00);
    }
    EIMSK = eimsk_save;
    return;
}

#endif // MCP2515_ERROR_CALLBACK_SUPPORT


//*****************************************************************************
//*********************** INTERRUPT SERVICE ROUTINES **************************
//*****************************************************************************

/*!
*******************************************************************************
** \brief   MCP2515 main interrupt service routine.
**
*******************************************************************************
*/
ISR(MCP2515_INT_MAIN_vect, ISR_BLOCK)
{
    uint8_t interrupt_code, tmp;
#if !MCP2515_USE_RX_INT
    MCP2515_CanMessageT can_msg;
#endif // !MCP2515_USE_RX_INT

    EIMSK &= ~(1 << MCP2515_INTNO_MAIN);
#if MCP2515_USE_RX_INT
    // disable other MCP2515 interrupts
    EIMSK &= ~((1 << MCP2515_INTNO_RXB0) | (1 << MCP2515_INTNO_RXB1));
#endif // MCP2515_USE_RX_INT
    sei();

#if MCP2515_ERROR_CALLBACK_SUPPORT
// *************  ISR WITH ERROR CALLBACK SUPPORT *************

    SET_LOW(MCP2515_CS);
    (void)SPI_M_Transceive(MCP2515_SPI_READ);
    (void)SPI_M_Transceive(MCP2515_CANINTF);
    interrupt_code = SPI_M_Transceive(0xFF);
    SET_HIGH(MCP2515_CS);

    if(mcp2515MessageErrorCallback && (interrupt_code & (1 << MCP2515_MERRF)))
    {
        PRINT_DEBUG("message error interrupt\n");
        // tmp_b = 0x00;
        // (void)SPI_M_Transceive(MCP2515_SPI_READ);
        // (void)SPI_M_Transceive(MCP2515_TXB0CTRL);
        // tmp_a = SPI_M_Transceive(0xFF);
        // if(tmp_a & (1 << MCP2515_TXERR))
        // {
        //     tmp_b |= MCP2515_MSG_ERR_TXERR_0;
        // }
        // (void)SPI_M_Transceive(MCP2515_SPI_READ);
        // (void)SPI_M_Transceive(MCP2515_TXB1CTRL);
        // tmp_a = SPI_M_Transceive(0xFF);
        // if(tmp_a & (1 << MCP2515_TXERR))
        // {
        //     tmp_b |= MCP2515_MSG_ERR_TXERR_1;
        // }
        // (void)SPI_M_Transceive(MCP2515_SPI_READ);
        // (void)SPI_M_Transceive(MCP2515_TXB2CTRL);
        // tmp_a = SPI_M_Transceive(0xFF);
        // if(tmp_a & (1 << MCP2515_TXERR))
        // {
        //     tmp_b |= MCP2515_MSG_ERR_TXERR_2;
        // }
        mcp2515MessageErrorCallback();
    }
    if(mcp2515WakeupCallback && (interrupt_code & (1 << MCP2515_WAKIF)))
    {
        PRINT_DEBUG("wakeup interrupt\n");
        mcp2515WakeupCallback();
    }
    if(mcp2515ErrorCallback && (interrupt_code & (1 << MCP2515_ERRIF)))
    {
        PRINT_DEBUG("error interrupt\n");
        (void)SPI_M_Transceive(MCP2515_SPI_READ);
        (void)SPI_M_Transceive(MCP2515_EFLG);
        tmp = SPI_M_Transceive(0xFF);
        mcp2515ErrorCallback(tmp);
        // clear flags:
        if(tmp & ((1 << MCP2515_RX1OVR) | (1 << MCP2515_RX0OVR)))
        {
            mcp2515CmdBitModify(MCP2515_EFLG, \
                (1 << MCP2515_RX1OVR) | (1 << MCP2515_RX0OVR), 0);
        }
    }
    if(mcp2515TxCallback)
    {
        if(interrupt_code & (1 << MCP2515_TX0IF))
        {
            PRINT_DEBUG("tx0 interrupt\n");
            mcp2515TxCallback(MCP2515_TX_BUFFER_0);
        }
        if(interrupt_code & (1 << MCP2515_TX1IF))
        {
            PRINT_DEBUG("tx1 interrupt\n");
            mcp2515TxCallback(MCP2515_TX_BUFFER_1);
        }
        if(interrupt_code & (1 << MCP2515_TX2IF))
        {
            PRINT_DEBUG("tx2 interrupt\n");
            mcp2515TxCallback(MCP2515_TX_BUFFER_2);
        }
    }
    tmp = (1 << MCP2515_MERRF) | \
          (1 << MCP2515_WAKIF) | \
          (1 << MCP2515_ERRIF) | \
          (1 << MCP2515_TX0IF) | \
          (1 << MCP2515_TX1IF) | \
          (1 << MCP2515_TX2IF);
    if(interrupt_code & tmp)
    {
        mcp2515CmdBitModify(MCP2515_CANINTF, tmp, 0x00);
    }
    if(mcp2515RxCallback && \
       (interrupt_code & ((1 << MCP2515_RX0IF) | (1 << MCP2515_RX1IF))))
    {
#if !MCP2515_USE_RX_INT
        if(interrupt_code & (1 << MCP2515_RX0IF))
        {
            tmp = MCP2515_SPI_READ_RXB0SIDH;
        }
        else
        {
            tmp = MCP2515_SPI_READ_RXB1SIDH;
        }
        SET_LOW(MCP2515_CS);
        (void)SPI_M_Transceive(tmp);
#if MCP2515_CAN_2_B_SUPPORT
        can_msg.sid  = ((uint32_t)SPI_M_Transceive(0xFF)) << 3; // SIDH
        tmp = SPI_M_Transceive(0xFF); // SIDL
        can_msg.sid |= (uint32_t) (tmp >> 5);
        if(tmp & (1 << MCP2515_IDE))
        {
            // extended frame
            can_msg.ief  = 1;
            can_msg.eid  = ((uint32_t) (tmp & 0x03)) << 16;
            can_msg.eid |= ((uint32_t) SPI_M_Transceive(0xFF)) << 8; // EIDH
            can_msg.eid |= (uint32_t) SPI_M_Transceive(0xFF); // EIDL
            tmp = SPI_M_Transceive(0xFF); // DLC
            can_msg.rtr  = (tmp & (1 << MCP2515_RTR)) ? 1 : 0;
            can_msg.dlc  = tmp & 0x0F;
        }
        else
        {
            // standard frame
            can_msg.ief  = 0;
            can_msg.rtr  = (tmp & (1 << MCP2515_SRR)) ? 1 : 0;
            (void)SPI_M_Transceive(0xFF); // EIDH
            (void)SPI_M_Transceive(0xFF); // EIDL
            can_msg.dlc  = SPI_M_Transceive(0xFF) & 0x0F; // DLC
        }
#else // CAN 2.0A only
        can_msg.sid  = ((uint16_t) SPI_M_Transceive(0xFF)) << 3; // SIDH
        tmp = SPI_M_Transceive(0xFF); // SIDL
        can_msg.sid |= (uint16_t) (tmp >> 5);
        can_msg.rtr  = (uint16_t) ((tmp & (1 << MCP2515_SRR)) ? 1 : 0);
        (void)SPI_M_Transceive(0xFF); // EIDH
        (void)SPI_M_Transceive(0xFF); // EIDL
        can_msg.dlc  = (uint16_t) (SPI_M_Transceive(0xFF) & 0x0F); // DLC
#endif // MCP2515_CAN_2_B_SUPPORT
        if(can_msg.rtr == 0)
        {
            for(tmp = 0; tmp < can_msg.dlc; tmp++)
            {
                can_msg.dataArray[tmp] = SPI_M_Transceive(0xFF);
            }
        }
        SET_HIGH(MCP2515_CS);
        // CANINTF.RXnIF is being cleared automatically (see 12.4).
        mcp2515RxCallback(&can_msg);
#else
        PRINT_DEBUG("ISR: unexpected RX0/RX1 handling!\n");
#endif // !MCP2515_USE_RX_INT
    }

#else // MCP2515_ERROR_CALLBACK_SUPPORT
// *************  ISR WITHOUT ERROR CALLBACK SUPPORT *************

    SET_LOW(MCP2515_CS);
    (void)SPI_M_Transceive(MCP2515_SPI_READ_STATUS);
    interrupt_code = SPI_M_Transceive(0xFF);
    SET_HIGH(MCP2515_CS);

    if(mcp2515TxCallback)
    {
        if(interrupt_code & (1 << MCP2515_RS_TX0IF))
        {
            PRINT_DEBUG("tx0 interrupt\n");
            mcp2515TxCallback(MCP2515_TX_BUFFER_0);
        }
        if(interrupt_code & (1 << MCP2515_RS_TX1IF))
        {
            PRINT_DEBUG("tx1 interrupt\n");
            mcp2515TxCallback(MCP2515_TX_BUFFER_1);
        }
        if(interrupt_code & (1 << MCP2515_RS_TX2IF))
        {
            PRINT_DEBUG("tx2 interrupt\n");
            mcp2515TxCallback(MCP2515_TX_BUFFER_2);
        }
        tmp = (1 << MCP2515_RS_TX0IF) | \
              (1 << MCP2515_RS_TX1IF) | \
              (1 << MCP2515_RS_TX2IF);
        if(interrupt_code & tmp)
        {
            mcp2515CmdBitModify(MCP2515_CANINTF, tmp, 0x00);
        }
    }
    if(mcp2515RxCallback && \
       (interrupt_code & ((1 << MCP2515_RS_RX0IF) | (1 << MCP2515_RS_RX1IF))))
    {
#if !MCP2515_USE_RX_INT
        if(interrupt_code & (1 << MCP2515_RS_RX0IF))
        {
            tmp = MCP2515_SPI_READ_RXB0SIDH;
        }
        else
        {
            tmp = MCP2515_SPI_READ_RXB1SIDH;
        }
        SET_LOW(MCP2515_CS);
        (void)SPI_M_Transceive(tmp);
#if MCP2515_CAN_2_B_SUPPORT
        can_msg.sid  = ((uint32_t)SPI_M_Transceive(0xFF)) << 3; // SIDH
        tmp = SPI_M_Transceive(0xFF); // SIDL
        can_msg.sid |= (uint32_t) (tmp >> 5);
        if(tmp & (1 << MCP2515_IDE))
        {
            // extended frame
            can_msg.ief  = 1;
            can_msg.eid  = ((uint32_t) (tmp & 0x03)) << 16;
            can_msg.eid |= ((uint32_t) SPI_M_Transceive(0xFF)) << 8; // EIDH
            can_msg.eid |= (uint32_t) SPI_M_Transceive(0xFF); // EIDL
            tmp = SPI_M_Transceive(0xFF); // DLC
            can_msg.rtr  = (tmp & (1 << MCP2515_RTR)) ? 1 : 0;
            can_msg.dlc  = tmp & 0x0F;
        }
        else
        {
            // standard frame
            can_msg.ief  = 0;
            can_msg.rtr  = (tmp & (1 << MCP2515_SRR)) ? 1 : 0;
            (void)SPI_M_Transceive(0xFF); // EIDH
            (void)SPI_M_Transceive(0xFF); // EIDL
            can_msg.dlc  = SPI_M_Transceive(0xFF) & 0x0F; // DLC
        }
#else // CAN 2.0A only
        can_msg.sid  = ((uint16_t) SPI_M_Transceive(0xFF)) << 3; // SIDH
        tmp = SPI_M_Transceive(0xFF); // SIDL
        can_msg.sid |= (uint16_t) (tmp >> 5);
        can_msg.rtr  = (uint16_t) ((tmp & (1 << MCP2515_SRR)) ? 1 : 0);
        (void)SPI_M_Transceive(0xFF); // EIDH
        (void)SPI_M_Transceive(0xFF); // EIDL
        can_msg.dlc  = (uint16_t) (SPI_M_Transceive(0xFF) & 0x0F); // DLC
#endif // MCP2515_CAN_2_B_SUPPORT
        if(can_msg.rtr == 0)
        {
            for(tmp = 0; tmp < can_msg.dlc; tmp++)
            {
                can_msg.dataArray[tmp] = SPI_M_Transceive(0xFF);
            }
        }
        SET_HIGH(MCP2515_CS);
        // CANINTF.RXnIF is being cleared automatically (see 12.4).
        mcp2515RxCallback(&can_msg);
#else
        PRINT_DEBUG("ISR: unexpected RX0/RX1 handling!\n");
#endif // !MCP2515_USE_RX_INT
    }

#endif // MCP2515_ERROR_CALLBACK_SUPPORT
// **********  END OF ISR WITHOUT ERROR CALLBACK SUPPORT *********

    cli();
    EIMSK |= (1 << MCP2515_INTNO_MAIN);
#if MCP2515_USE_RX_INT
    // recover other MCP2515 interrupts
    if(mcp2515State.rxIrqEnable)
    {
        EIMSK |= ((1 << MCP2515_INTNO_RXB0) | (1 << MCP2515_INTNO_RXB1));
    }
#endif // MCP2515_USE_RX_INT
    return;
}

#if MCP2515_USE_RX_INT

/*!
*******************************************************************************
** \brief   MCP2515 receive buffer 0 interrupt service routine.
**
*******************************************************************************
*/
ISR(MCP2515_INT_RXB0_vect, ISR_BLOCK)
{
    uint8_t ii, tmp;
    MCP2515_CanMessageT can_msg;

    EIMSK &= ~((1 << MCP2515_INTNO_MAIN) | \
               (1 << MCP2515_INTNO_RXB0) | \
               (1 << MCP2515_INTNO_RXB1));
    sei();
    SET_LOW(MCP2515_CS);
    (void)SPI_M_Transceive(MCP2515_SPI_READ_RXB0SIDH);
#if MCP2515_CAN_2_B_SUPPORT
    can_msg.sid  = ((uint32_t)SPI_M_Transceive(0xFF)) << 3; // SIDH
    tmp = SPI_M_Transceive(0xFF); // SIDL
    can_msg.sid |= (uint32_t) (tmp >> 5);
    if(tmp & (1 << MCP2515_IDE))
    {
        // extended frame
        can_msg.ief  = 1;
        can_msg.eid  = ((uint32_t) (tmp & 0x03)) << 16;
        can_msg.eid |= ((uint32_t) SPI_M_Transceive(0xFF)) << 8; // EIDH
        can_msg.eid |= (uint32_t) SPI_M_Transceive(0xFF); // EIDL
        tmp = SPI_M_Transceive(0xFF); // DLC
        can_msg.rtr  = (tmp & (1 << MCP2515_RTR)) ? 1 : 0;
        can_msg.dlc  = tmp & 0x0F;
    }
    else
    {
        // standard frame
        can_msg.ief  = 0;
        can_msg.rtr  = (tmp & (1 << MCP2515_SRR)) ? 1 : 0;
        (void)SPI_M_Transceive(0xFF); // EIDH
        (void)SPI_M_Transceive(0xFF); // EIDL
        can_msg.dlc  = SPI_M_Transceive(0xFF) & 0x0F; // DLC
    }
#else // CAN 2.0A only
    can_msg.sid  = ((uint16_t) SPI_M_Transceive(0xFF)) << 3; // SIDH
    tmp = SPI_M_Transceive(0xFF); // SIDL
    can_msg.sid |= (uint16_t) (tmp >> 5);
    can_msg.rtr  = (uint16_t) ((tmp & (1 << MCP2515_SRR)) ? 1 : 0);
    (void)SPI_M_Transceive(0xFF); // EIDH
    (void)SPI_M_Transceive(0xFF); // EIDL
    can_msg.dlc  = (uint16_t) (SPI_M_Transceive(0xFF) & 0x0F); // DLC
#endif // MCP2515_CAN_2_B_SUPPORT
    if(can_msg.rtr == 0)
    {
        for(ii = 0; ii < can_msg.dlc; ii++)
        {
            can_msg.dataArray[ii] = SPI_M_Transceive(0xFF);
        }
    }
    SET_HIGH(MCP2515_CS);
    // CANINTF.RXnIF is being cleared automatically (see 12.4).
    mcp2515RxCallback(&can_msg);
    cli();
    EIMSK |= ((1 << MCP2515_INTNO_MAIN) | \
              (1 << MCP2515_INTNO_RXB0) | \
              (1 << MCP2515_INTNO_RXB1));
    return;
}

/*!
*******************************************************************************
** \brief   MCP2515 receive buffer 1 interrupt service routine.
**
*******************************************************************************
*/
ISR(MCP2515_INT_RXB1_vect, ISR_BLOCK)
{
    uint8_t ii, tmp;
    MCP2515_CanMessageT can_msg;

    EIMSK &= ~((1 << MCP2515_INTNO_MAIN) | \
               (1 << MCP2515_INTNO_RXB0) | \
               (1 << MCP2515_INTNO_RXB1));
    sei();
    SET_LOW(MCP2515_CS);
    (void)SPI_M_Transceive(MCP2515_SPI_READ_RXB1SIDH);
#if MCP2515_CAN_2_B_SUPPORT
    can_msg.sid  = ((uint32_t)SPI_M_Transceive(0xFF)) << 3; // SIDH
    tmp = SPI_M_Transceive(0xFF); // SIDL
    can_msg.sid |= (uint32_t) (tmp >> 5);
    if(tmp & (1 << MCP2515_IDE))
    {
        // extended frame
        can_msg.ief  = 1;
        can_msg.eid  = ((uint32_t) (tmp & 0x03)) << 16;
        can_msg.eid |= ((uint32_t) SPI_M_Transceive(0xFF)) << 8; // EIDH
        can_msg.eid |= (uint32_t) SPI_M_Transceive(0xFF); // EIDL
        tmp = SPI_M_Transceive(0xFF); // DLC
        can_msg.rtr  = (tmp & (1 << MCP2515_RTR)) ? 1 : 0;
        can_msg.dlc  = tmp & 0x0F;
    }
    else
    {
        // standard frame
        can_msg.ief  = 0;
        can_msg.rtr  = (tmp & (1 << MCP2515_SRR)) ? 1 : 0;
        (void)SPI_M_Transceive(0xFF); // EIDH
        (void)SPI_M_Transceive(0xFF); // EIDL
        can_msg.dlc  = SPI_M_Transceive(0xFF) & 0x0F; // DLC
    }
#else // CAN 2.0A only
    can_msg.sid  = ((uint16_t) SPI_M_Transceive(0xFF)) << 3; // SIDH
    tmp = SPI_M_Transceive(0xFF); // SIDL
    can_msg.sid |= (uint16_t) (tmp >> 5);
    can_msg.rtr  = (uint16_t) ((tmp & (1 << MCP2515_SRR)) ? 1 : 0);
    (void)SPI_M_Transceive(0xFF); // EIDH
    (void)SPI_M_Transceive(0xFF); // EIDL
    can_msg.dlc  = (uint16_t) (SPI_M_Transceive(0xFF) & 0x0F); // DLC
#endif // MCP2515_CAN_2_B_SUPPORT
    if(can_msg.rtr == 0)
    {
        for(ii = 0; ii < can_msg.dlc; ii++)
        {
            can_msg.dataArray[ii] = SPI_M_Transceive(0xFF);
        }
    }
    SET_HIGH(MCP2515_CS);
    // CANINTF.RXnIF is being cleared automatically (see 12.4).
    mcp2515RxCallback(&can_msg);
    cli();
    EIMSK |= ((1 << MCP2515_INTNO_MAIN) | \
              (1 << MCP2515_INTNO_RXB0) | \
              (1 << MCP2515_INTNO_RXB1));
    return;
}

#endif // MCP2515_USE_RX_INT
