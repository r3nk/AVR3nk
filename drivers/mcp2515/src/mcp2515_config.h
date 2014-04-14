/*!
*******************************************************************************
*******************************************************************************
** \brief   Automatic configuration settings for MCP2515 CAN bus driver.
**
**          This file provides default parameters for the initialization of the
**          MCP2515. The parameters depend on the given F_MCP2515 oscillator
**          frequency. You want to use the following macros for initialization
**          of the driver:
**          - MCP2515_AUTO_BRP
**          - MCP2515_AUTO_SJW
**          - MCP2515_AUTO_PRSEG
**          - MCP2515_AUTO_PHSEG1
**          - MCP2515_AUTO_PHSEG2
**
** \author  Robin Klose
**
** \sa      For bit timing settings see the example in MCP2515-I-P.pdf chapter 5.2
**
** Copyright (C) 2009-2013 Robin Klose
**
** This file is part of AVR3nk, available at https://github.com/r3nk/AVR3nk
**
*******************************************************************************
*******************************************************************************
*/

#ifndef MCP2515_CONFIG_H
#define MCP2515_CONFIG_H

#include <stdint.h>
#include <drivers/spi_common.h>

//*****************************************************************************
//*************************** DEFINES AND MACROS ******************************
//*****************************************************************************

//! CPU frequency
#ifndef F_CPU
    #define F_CPU                   8000000
#endif // F_CPU

//! MCP2515 frequency
#ifndef F_MCP2515
    #define F_MCP2515               18432000
#endif // F_MCP2515

//*****************************************************************************
//**************************** DEFAULT SETTINGS *******************************
//*****************************************************************************

// SPI parameters for the MCP2515
#define MCP2515_SPI_CLOCK_DIVIDER       SPI_CLK_DIV_2
#define MCP2515_SPI_DATA_ORDER          SPI_MSB_FIRST
#define MCP2515_SPI_CLOCK_PARITY        SPI_LEADING_EDGE_RISING
#define MCP2515_SPI_CLOCK_PHASE         SPI_SAMPLE_LEADING_EDGE

#if F_MCP2515 == 8000000
    #define MCP2515_AUTO_BRP            MCP2515_125KBIT_8000000HZ_BRP
    #define MCP2515_AUTO_SJW            MCP2515_125KBIT_8000000HZ_SJW
    #define MCP2515_AUTO_PRSEG          MCP2515_125KBIT_8000000HZ_PRSEG
    #define MCP2515_AUTO_PHSEG1         MCP2515_125KBIT_8000000HZ_PHSEG1
    #define MCP2515_AUTO_PHSEG2         MCP2515_125KBIT_8000000HZ_PHSEG2
#endif

#if F_MCP2515 == 18432000
    #define MCP2515_AUTO_BRP            MCP2515_125KBIT_18432000HZ_BRP
    #define MCP2515_AUTO_SJW            MCP2515_125KBIT_18432000HZ_SJW
    #define MCP2515_AUTO_PRSEG          MCP2515_125KBIT_18432000HZ_PRSEG
    #define MCP2515_AUTO_PHSEG1         MCP2515_125KBIT_18432000HZ_PHSEG1
    #define MCP2515_AUTO_PHSEG2         MCP2515_125KBIT_18432000HZ_PHSEG2
#endif

#if F_MCP2515 == 20000000
    #define MCP2515_AUTO_BRP            MCP2515_125KBIT_20000000HZ_BRP
    #define MCP2515_AUTO_SJW            MCP2515_125KBIT_20000000HZ_SJW
    #define MCP2515_AUTO_PRSEG          MCP2515_125KBIT_20000000HZ_PRSEG
    #define MCP2515_AUTO_PHSEG1         MCP2515_125KBIT_20000000HZ_PHSEG1
    #define MCP2515_AUTO_PHSEG2         MCP2515_125KBIT_20000000HZ_PHSEG2
#endif

//*****************************************************************************
//**************** OSCILLATOR FREQUENCY DEPENDENT SETTINGS ********************
//*****************************************************************************

// A CAN baud rate of 125 kbit/s complies with a nominal bit time (NBT) of 8µs.

// TQ = 0.5µs       NBT = 8µs           baudrate = 125kbit/s
#define MCP2515_125KBIT_8000000HZ_BRP           0x01
#define MCP2515_125KBIT_8000000HZ_SJW           MCP2515_SJW_1TQ
#define MCP2515_125KBIT_8000000HZ_PRSEG         MCP2515_PRSEG_2TQ
#define MCP2515_125KBIT_8000000HZ_PHSEG1        MCP2515_PHSEG1_7TQ
#define MCP2515_125KBIT_8000000HZ_PHSEG2        MCP2515_PHSEG2_6TQ

// TQ = 0.6510µs    NBT = 7.8125µs      baudrate = 128kbit/s
#define MCP2515_125KBIT_18432000HZ_BRP          0x05
#define MCP2515_125KBIT_18432000HZ_SJW          MCP2515_SJW_1TQ
#define MCP2515_125KBIT_18432000HZ_PRSEG        MCP2515_PRSEG_2TQ
#define MCP2515_125KBIT_18432000HZ_PHSEG1       MCP2515_PHSEG1_5TQ
#define MCP2515_125KBIT_18432000HZ_PHSEG2       MCP2515_PHSEG2_4TQ

// TQ = 0.5µs       NBT = 8µs           baudrate = 125kbit/s
#define MCP2515_125KBIT_20000000HZ_BRP          0x04
#define MCP2515_125KBIT_20000000HZ_SJW          MCP2515_SJW_1TQ
#define MCP2515_125KBIT_20000000HZ_PRSEG        MCP2515_PRSEG_2TQ
#define MCP2515_125KBIT_20000000HZ_PHSEG1       MCP2515_PHSEG1_7TQ
#define MCP2515_125KBIT_20000000HZ_PHSEG2       MCP2515_PHSEG2_6TQ


// A CAN baud rate of 500 kbit/s complies with a nominal bit time (NBT) of 2µs.

// TQ = 217.014ns   NBT = 1.953125µs    baudrate = 512kbit/s
#define MCP2515_512KBIT_18432000HZ_BRP          0x01
#define MCP2515_512KBIT_18432000HZ_SJW          MCP2515_SJW_1TQ
#define MCP2515_512KBIT_18432000HZ_PRSEG        MCP2515_PRSEG_2TQ
#define MCP2515_512KBIT_18432000HZ_PHSEG1       MCP2515_PHSEG1_3TQ
#define MCP2515_512KBIT_18432000HZ_PHSEG2       MCP2515_PHSEG2_3TQ

// TQ = 200ns       NBT = 2µs           baudrate = 500kbit/s
#define MCP2515_500KBIT_20000000HZ_BRP          0x01
#define MCP2515_500KBIT_20000000HZ_SJW          MCP2515_SJW_1TQ
#define MCP2515_500KBIT_20000000HZ_PRSEG        MCP2515_PRSEG_3TQ
#define MCP2515_500KBIT_20000000HZ_PHSEG1       MCP2515_PHSEG1_3TQ
#define MCP2515_500KBIT_20000000HZ_PHSEG2       MCP2515_PHSEG2_3TQ


#endif // MCP2515_CONFIG_H
