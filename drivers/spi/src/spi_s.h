/*!
*******************************************************************************
*******************************************************************************
** \brief   Interface declarations of SPI driver in slave mode
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

#ifndef SPI_S_H
#define SPI_S_H

#include <stdint.h>
#include "spi_common.h"

//*****************************************************************************
//*************************** DEFINES AND MACROS ******************************
//*****************************************************************************

//! enable/disable trasceive LED for the SPI
#ifndef SPI_S_LED_MODE
    #define SPI_S_LED_MODE          0
#endif // SPI_S_LED_MODE

//! SPI LED output: (PORT,PIN)
#ifndef SPI_S_LED
    #define SPI_S_LED               B,1
#endif // SPI_S_LED

//! SPI_S debug mode switch
#ifndef SPI_S_DEBUG
    #define SPI_S_DEBUG             0
#endif // SPI_S_DEBUG

#ifdef  APP_DEBUG
    #undef  SPI_S_DEBUG
    #define SPI_S_DEBUG             1
#endif // APP_DEBUG

#ifndef SPI_S_LABEL
    #define SPI_S_LABEL             "[SPI] "
#endif // SPI_S_LABEL

#ifndef SPI_S_LABEL_DEBUG
    #define SPI_S_LABEL_DEBUG       "[SPI/dbg] "
#endif // SPI_S_LABEL_DEBUG

//! CPU frequency
#ifndef F_CPU
    #define F_CPU                   8000000
#endif // F_CPU

//*****************************************************************************
//************************* SPI_S SPECIFIC ERROR CODES ************************
//*****************************************************************************

/*! SPI_S specific error base */
#ifndef SPI_S_ERR_BASE
#define SPI_S_ERR_BASE              35
#endif


/*! SPI_S returns with no errors. */
#define SPI_S_OK                    0

/*! A bad parameter has been passed. */
#define SPI_S_ERR_BAD_PARAMETER     SPI_S_ERR_BASE + 0


//*****************************************************************************
//******************************** DATA TYPES *********************************
//*****************************************************************************

typedef void (*SPI_S_CallbackT) (uint8_t byte);

//*****************************************************************************
//************************* FUNCTION DECLARATIONS *****************************
//*****************************************************************************

uint8_t SPI_S_Init (SPI_DataOrderT dataOrder,
                    SPI_ClockPolarityT clockParity,
                    SPI_ClockPhaseT clockPhase);
uint8_t SPI_S_IsInitialized (void);
void    SPI_S_SetCallback (SPI_S_CallbackT callback);
void    SPI_S_SetSendByte (uint8_t byte);


#endif // SPI_S_H
