/*!
*******************************************************************************
*******************************************************************************
** \brief   Common interface declarations of SPI driver
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

#ifndef SPI_COMMON_H
#define SPI_COMMON_H

//*****************************************************************************
//*************************** DEFINES AND MACROS ******************************
//*****************************************************************************

// SPI port settings:

//******** ATmega644 setup ********

#if (defined atmega644) || (defined atmega644p)

#define DDR_SPI     DDRB
#define PORT_SPI    PORTB

#define DD_SS       DDB4
#define DD_MOSI     DDB5
#define DD_MISO     DDB6
#define DD_SCK      DDB7

#define PORT_SS     PORTB4
#define PORT_MOSI   PORTB5
#define PORT_MISO   PORTB6
#define PORT_SCK    PORTB7

#endif // atmega644 || atmega644p


//******** ATmega16 setup ********

#ifdef atmega16

#define DDR_SPI     DDRB
#define PORT_SPI    PORTB

#define DD_SS       DDB4
#define DD_MOSI     DDB5
#define DD_MISO     DDB6
#define DD_SCK      DDB7

#define PORT_SS     PORTB4
#define PORT_MOSI   PORTB5
#define PORT_MISO   PORTB6
#define PORT_SCK    PORTB7

#endif // atmega16

//*****************************************************************************
//******************************** DATA TYPES *********************************
//*****************************************************************************

/*! Specifies the data order in which data will be shifted out and in. */
typedef enum
{
    SPI_MSB_FIRST,
    SPI_LSB_FIRST
} SPI_DataOrderT;

/*! Defines the clock paritiy. When leading edge is defined to be a rising
**  edge, the SCK is low when the SPI is idle. Otherwise, when the leading
**  edge is a falling edge, the SCK is high when the SPI is idle.
*/
typedef enum
{
    SPI_LEADING_EDGE_RISING,
    SPI_LEADING_EDGE_FALLING
} SPI_ClockPolarityT;

/*! Determines whether data is sampled on the leading or trailing edge of
**  the SCK. Data bits are always shifted out and latched in on opposite
**  edges of the SCK signal.
*/
typedef enum
{
    SPI_SAMPLE_LEADING_EDGE,
    SPI_SAMPLE_TRAILING_EDGE
} SPI_ClockPhaseT;

/*! Determines the SCK (SPI Clock) speed in terms of the CPU frequency. */
typedef enum
{
    SPI_CLK_DIV_2,
    SPI_CLK_DIV_4,
    SPI_CLK_DIV_8,
    SPI_CLK_DIV_16,
    SPI_CLK_DIV_32,
    SPI_CLK_DIV_64,
    SPI_CLK_DIV_128
} SPI_ClockDivisionT;




#endif // SPI_COMMON_H
