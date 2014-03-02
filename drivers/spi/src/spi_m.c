/*!
*******************************************************************************
*******************************************************************************
** \brief   SPI driver for AVR microcontrollers
**
**          This driver runs the SPI in master mode. The chip select lines
**          must be driven by the actual device drivers.
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
#include <avr/io.h>
#include <drivers/macros_pin.h>
#include "spi_m.h"

#if SPI_M_DEBUG
#include <stdio.h>
#endif // SPI_M_DEBUG

//*****************************************************************************
//*************************** DEFINES AND MACROS ******************************
//*****************************************************************************

#if SPI_M_LED_MODE
    //! Enable SPI complete interrupt and light the LED
    #define SPI_M_COMPLETE_INTERRUPT_ON     SPCR |=  (1 << SPIE);   \
                                            SET_HIGH(SPI_M_LED)
    //! Disable SPI complete interrupt and turn off LED
    #define SPI_M_COMPLETE_INTERRUPT_OFF    SPCR &= ~(1 << SPIE);   \
                                            SET_LOW(SPI_M_LED)
#else // SPI_M_LED_MODE
    #define SPI_M_COMPLETE_INTERRUPT_ON     SPCR |=  (1 << SPIE)
    #define SPI_M_COMPLETE_INTERRUPT_OFF    SPCR &= ~(1 << SPIE)
#endif // SPI_M_LED_MODE

//*****************************************************************************
//**************************** LOCAL DATA TYPES *******************************
//*****************************************************************************


//*****************************************************************************
//**************************** LOCAL VARIABLES ********************************
//*****************************************************************************

static struct
{
    uint8_t initialized : 1; //<! Indicates whether SPI_M has been initialized
} spimState;

//*****************************************************************************
//*************************** PUBLIC FUNCTIONS ********************************
//*****************************************************************************

/*!
*******************************************************************************
** \brief   Initializes the SPI hardware as master.
**
** \param   clockDivider
**              Defines the shift clock in relation to the CPU clock.
** \param   dataOrder
**              Defines whether to shift the MSB or the LSB first.
** \param   clockParity
**              Defines whether the leading edge is rising or falling.
** \param   clockPhase
**              Defines whether the bits are sampled at the leading or at
**              the trailing clock edge.
**
** \return
**          - #SPI_M_OK on success.
**
*******************************************************************************
*/
int8_t SPI_M_Init (SPI_ClockDivisionT clockDivider,
                   SPI_DataOrderT dataOrder,
                   SPI_ClockPolarityT clockPolarity,
                   SPI_ClockPhaseT clockPhase)
{
    uint8_t accu = 0;

    // disable SPI block:
    SPCR = 0x00;
    SPSR = 0x00;

    // set up pin configuration:

    // See ATmega644p chapter 15.3
    // If set as input, the SS pin can change the SPI master to a slave.
    // This driver doesn't handle this, so set pin as output and use as CS.
    // CS pin (chip select) is then handled by the respective device driver.
    DDR_SPI  |=  (1 << DD_SS);
    PORT_SPI |=  (1 << PORT_SS);

    // Set MOSI pin as output:
    DDR_SPI  |=  (1 << DD_MOSI);
    // PORT_SPI &= ~(1 << PORT_MOSI);

    // MISO pin is configured as an input regardless of the setting of DD_MISO.
    //DDR_SPI  &= ~(1 << DD_MISO);
    //PORT_SPI &= ~(1 << PORT_MISO);

    // Enable clock output:
    DDR_SPI  |=  (1 << DD_SCK);
    //PORT_SPI &= ~(1 << PORT_SCK);

    // set up data order:
    SPI_M_SetDataOrder(dataOrder);

    // set up clock parity:
    SPI_M_SetClockParity(clockPolarity);

    // set up clock phase:
    SPI_M_SetClockPhase(clockPhase);

    // set up clock divider:
    SPI_M_SetClockDivision (clockDivider);

    // Clear SPI interrupt flag by reading SPSR and SPDR consecutively:
    accu = SPSR; // (void) SPSR;
    accu = SPDR; // (void) SPDR;

#if SPI_M_LED_MODE
    SET_LOW(SPI_M_LED);
    SET_OUTPUT(SPI_M_LED);
#endif // SPI_M_LED_MODE

    // enable SPI block & set up master mode:
    SPCR |= ((1 << SPE) | (1 << MSTR));

    // initialize local variables:
    spimState.initialized = 1;

#if SPI_M_DEBUG
    printf(SPI_M_LABEL_DEBUG "SPCR = 0x%x\n", SPCR);
#endif // SPI_M_DEBUG

    if ((SPCR & 0xFC) != ((1 << SPE) | (dataOrder << DORD) | (1 << MSTR) |
                          (clockPolarity << CPOL) | (clockPhase << CPHA)))
    {
        return(SPI_M_ERR_VERIFY_FAIL);
    }
    return(SPI_M_OK);
}

/*!
*******************************************************************************
** \brief   Test whether the SPI_M is initialized.
**
** \return
**          - 1 if the SPI_M is initialized.
**          - 0 if the SPI_M is not initialized.
**
*******************************************************************************
*/
int8_t SPI_M_IsInitialized (void)
{
    return(spimState.initialized);
}

/*!
*******************************************************************************
** \brief   Set up the data order.
**
** \param   dataOrder
**              Defines whether to shift the MSB or the LSB first.
**
*******************************************************************************
*/
void SPI_M_SetDataOrder (SPI_DataOrderT dataOrder)
{
    if (dataOrder == SPI_LSB_FIRST)
    {
        SPCR |= (1 << DORD);
    }
    else
    {
        SPCR &= ~(1 << DORD);
    }
    return;
}

/*!
*******************************************************************************
** \brief   Set up the clock parity.
**
** \param   clockParity
**              Defines whether the leading edge is rising or falling.
**
*******************************************************************************
*/
void SPI_M_SetClockParity (SPI_ClockPolarityT clockParity)
{
    if (clockParity == SPI_LEADING_EDGE_FALLING)
    {
        SPCR |= (1 << CPOL);
    }
    else
    {
        SPCR &= ~(1 << CPOL);
    }
    return;
}

/*!
*******************************************************************************
** \brief   Set up the clock phase.
**
** \param   clockPhase
**              Defines whether the bits are sampled at the leading or at
**              the trailing clock edge.
**
*******************************************************************************
*/
void SPI_M_SetClockPhase (SPI_ClockPhaseT clockPhase)
{
    if (clockPhase == SPI_SAMPLE_TRAILING_EDGE)
    {
        SPCR |= (1 << CPHA);
    }
    else
    {
        SPCR &= ~(1 << CPHA);
    }
    return;
}

/*!
*******************************************************************************
** \brief   Set up the shift clock speed.
**
** \param   clockDivider
**              Defines the clock division in relation to the CPU clock frequency.
**
*******************************************************************************
*/
void SPI_M_SetClockDivision (SPI_ClockDivisionT clockDivider)
{
    if ((clockDivider == SPI_CLK_DIV_32)
    ||  (clockDivider == SPI_CLK_DIV_64)
    ||  (clockDivider == SPI_CLK_DIV_128))
    {
        SPCR |= (1 << SPR1);
    }
    else
    {
        SPCR &= ~(1 << SPR1);
    }

    if ((clockDivider == SPI_CLK_DIV_8)
    ||  (clockDivider == SPI_CLK_DIV_16)
    ||  (clockDivider == SPI_CLK_DIV_128))
    {
        SPCR |= (1 << SPR0);
    }
    else
    {
        SPCR &= ~(1 << SPR0);
    }

    // set up SPI2X:
    if((clockDivider == SPI_CLK_DIV_2)
    || (clockDivider == SPI_CLK_DIV_8)
    || (clockDivider == SPI_CLK_DIV_32))
    {
        SPSR |= (1 << SPI2X);
    }
    else
    {
        SPSR &= ~(1 << SPI2X);
    }
    return;
}

/*!
*******************************************************************************
** \brief   Transcieve a byte via the SPI.
**
** \param   byte    The byte that will be shifted out to the slave.
**
** \return  The byte that has been shifted in from the slave.
**
*******************************************************************************
*/
uint8_t SPI_M_Transceive (uint8_t byte)
{
    uint8_t tmp;

    SPDR = byte;
    do
    {
        tmp = SPSR;
    } while(!(tmp & (1 << SPIF)));
    tmp = SPDR;
    return(tmp);
}

////*****************************************************************************
////*********************** INTERRUPT SERVICE ROUTINES **************************
////*****************************************************************************
