/*!
*******************************************************************************
*******************************************************************************
** \brief   Interface declarations of SPI in master mode with a single slave
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

#ifndef SPI_M_H
#define SPI_M_H

#include <stdint.h>
#include "spi_common.h"

//*****************************************************************************
//*************************** DEFINES AND MACROS ******************************
//*****************************************************************************

//! enable/disable trasceive LED for the SPI
#ifndef SPI_M_LED_MODE
    #define SPI_M_LED_MODE          0
#endif // SPI_M_LED_MODE

//! SPI LED output: (PORT,PIN)
#ifndef SPI_M_LED
    #define SPI_M_LED               B,1
#endif // SPI_M_LED

//! SPI_M debug mode switch
#ifndef SPI_M_DEBUG
    #define SPI_M_DEBUG             0
#endif // SPI_M_DEBUG

//! print label
#ifndef SPI_M_LABEL
    #define SPI_M_LABEL             "[SPI] "
#endif // SPI_M_LABEL

//! debug print label
#ifndef SPI_M_LABEL_DEBUG
    #define SPI_M_LABEL_DEBUG       "[SPI/dbg] "
#endif // SPI_M_LABEL_DEBUG

//! CPU frequency
#ifndef F_CPU
    #define F_CPU                   8000000
#endif // F_CPU

//*****************************************************************************
//************************* SPI_M SPECIFIC ERROR CODES ************************
//*****************************************************************************

/*! SPI_M specific error base */
#define SPI_M_ERR_BASE              0

/*! SPI_M returns with no errors. */
#define SPI_M_OK                    0

/*! A bad parameter has been passed. */
#define SPI_M_ERR_BAD_PARAMETER     SPI_M_ERR_BASE - 1

/*! Register verification after init failed. */
#define SPI_M_ERR_VERIFY_FAIL       SPI_M_ERR_BASE - 2

//*****************************************************************************
//******************************** DATA TYPES *********************************
//*****************************************************************************


//*****************************************************************************
//************************* FUNCTION DECLARATIONS *****************************
//*****************************************************************************

int8_t  SPI_M_Init (SPI_ClockDivisionT clockDivider,
                    SPI_DataOrderT dataOrder,
                    SPI_ClockPolarityT clockPolarity,
                    SPI_ClockPhaseT clockPhase);
int8_t  SPI_M_IsInitialized (void);
void    SPI_M_SetDataOrder (SPI_DataOrderT dataOrder);
void    SPI_M_SetClockParity (SPI_ClockPolarityT clockParity);
void    SPI_M_SetClockPhase (SPI_ClockPhaseT clockPhase);
void    SPI_M_SetClockDivision (SPI_ClockDivisionT clockDivider);
uint8_t SPI_M_Transceive (uint8_t byte);

#endif // SPI_M_H
