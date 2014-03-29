/*!
*******************************************************************************
*******************************************************************************
** \brief   SPI driver for AVR microcontrollers
**
**          This driver runs the SPI in slave mode.
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
#include <avr/io.h>
#include <avr/interrupt.h>
#include <drivers/macros_pin.h>
#include "spi_s.h"

#if SPI_S_DEBUG
#include <stdio.h>
#endif // SPI_S_DEBUG

//*****************************************************************************
//*************************** DEFINES AND MACROS ******************************
//*****************************************************************************


//*****************************************************************************
//**************************** LOCAL DATA TYPES *******************************
//*****************************************************************************


//*****************************************************************************
//**************************** LOCAL VARIABLES ********************************
//*****************************************************************************

static struct
{
    uint8_t initialized : 1; //<! Indicates whether SPI_S has been initialized
} spisState;

static SPI_S_CallbackT spisCallback;

//*****************************************************************************
//*************************** PUBLIC FUNCTIONS ********************************
//*****************************************************************************

/*!
*******************************************************************************
** \brief   Initializes the SPI hardware as slave.
**
** \param   dataOrder
**              Defines whether to shift the MSB or the LSB first.
** \param   clockParity
**              Defines whether the leading edge is rising or falling.
** \param   clockPhase
**              Defines whether the bits are sampled at the leading or at
**              the trailing clock edge.
**
** \return
**          - #SPI_S_OK on success.
**
*******************************************************************************
*/
uint8_t SPI_S_Init (SPI_DataOrderT dataOrder,
                    SPI_ClockPolarityT clockParity,
                    SPI_ClockPhaseT clockPhase)
{
    uint8_t accu = 0;
    uint8_t spi_sreg = SREG;

    // set up pin configuration:

    // SS pin is configured as an input regardless of the setting of DD_SS.
    //DDR_SPI  &= ~(1 << DD_SS);
    //PORT_SPI &= ~(1 << PORT_SS);

    // MOSI pin is configured as an input regardless of the setting of DD_MOSI.
    //DDR_SPI  &= ~(1 << DD_MOSI);
    //PORT_SPI &= ~(1 << PORT_MOSI);

    // Set MISO pin as output.
    DDR_SPI  |=  (1 << DD_MISO);
    //PORT_SPI &= ~(1 << PORT_MISO);

    // SCK pin is configured as an input regardless of the setting of DD_SCK.
    //DDR_SPI  &= ~(1 << DD_SCK);
    //PORT_SPI &= ~(1 << PORT_SCK);

    // enable SPI block:
    accu = (1 << SPE);

    // set up data order:
    if (dataOrder == SPI_LSB_FIRST) accu |= (1 << DORD);

    // set up clock parity:
    if (clockParity == SPI_LEADING_EDGE_FALLING) accu |= (1 << CPOL);

    // set up clock phase:
    if (clockPhase == SPI_SAMPLE_TRAILING_EDGE) accu |= (1 << CPHA);

    // enable SPI interrupt:
    accu |= (1 << SPIE);

    // write out register:
    SPCR = accu;

    // Clear SPI interrupt flag by reading SPSR and SPDR consecutively:
    accu = SPSR;
    accu = SPDR;

#if SPI_S_LED_MODE
    SET_OUTPUT(SPI_S_LED);
    SET_LOW(SPI_S_LED);
#endif // SPI_S_LED_MODE

    // initialize local variables:
    spisState.initialized = 1;

    // restore SREG:
    SREG = spi_sreg;

    return(SPI_S_OK);
}

/*!
*******************************************************************************
** \brief   Test whether the SPI_S is initialized.
**
** \return
**          - 1 if the SPI_S is initialized.
**          - 0 if the SPI_S is not initialized.
**
*******************************************************************************
*/
uint8_t SPI_S_IsInitialized (void)
{
    return(spisState.initialized);
}

/*!
*******************************************************************************
** \brief   Register a callback that will be executed on each SPI activity.
**
**          The callback will be passed the received value as argument.
**
** \attention
**          This callback will be executed within ISR context and is
**          therefore time critical!
**
*******************************************************************************
*/
void SPI_S_SetCallback (SPI_S_CallbackT callback)
{
    spisCallback = callback;
    return;
}

/*!
*******************************************************************************
** \brief   Set the byte that will be sent at the next transmission time.
**
**          If a transmission is currently in progress, the HW will set the
**          WCOL bit in SPSR. The procedure will try to write the SPDR until
**          the transmission is complete and no write collision will occur.
**
*******************************************************************************
*/
void SPI_S_SetSendByte (uint8_t byte)
{
    uint8_t status;

    do
    {
        SPDR = byte;
        status = SPSR;

        // TODO: Does this read? And may one read SPDR here at all? If not so, does the ISR do the job?
        (void)SPDR;

    } while (status & (1 << WCOL));
    return;
}

////*****************************************************************************
////*********************** INTERRUPT SERVICE ROUTINES **************************
////*****************************************************************************

/*!
*******************************************************************************
** \brief   ISR for SPI serial transfer complete.
**
**          Will be executed when a serial transfer has been completed.
**
*******************************************************************************
*/
ISR(SPI_STC_vect, ISR_BLOCK)
{
    uint8_t rx;

    rx = SPDR;
    spisCallback(rx);
    return;
}
