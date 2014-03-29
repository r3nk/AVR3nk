/*!
*******************************************************************************
*******************************************************************************
** \brief   BUFFER interface declarations for buffer with 8-bit addressing
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

#ifndef BUFFER_H
#define BUFFER_H

#include <stdint.h>

//*****************************************************************************
//*************************** DEFINES AND MACROS ******************************
//*****************************************************************************

/*! BUFFER specific error base */
#ifndef BUFFER_ERR_BASE
#define BUFFER_ERR_BASE             10
#endif

/*! BUFFER returns with no errors. */
#define BUFFER_OK                   0

/*! A bad parameter has been passed. */
#define BUFFER_ERR_BAD_PARAMETER    BUFFER_ERR_BASE + 0

/*! The buffer was empty. */
#define BUFFER_ERR_EMPTY            BUFFER_ERR_BASE + 1

/*! Buffer overflow. */
#define BUFFER_ERR_FULL             BUFFER_ERR_BASE + 2

/*! Bad byteCount argument. */
#define BUFFER_ERR_BYTE_COUNT       BUFFER_ERR_BASE + 3

//*****************************************************************************
//******************************** DATA TYPES *********************************
//*****************************************************************************

typedef struct
{
    uint8_t* startPtr; //!< start address of buffer field
    uint8_t  length;   //!< buffer length in bytes
    uint8_t  readPos;  //!< read position in the buffer
    uint8_t  used;     //!< count of used bytes in the buffer
}BUFFER_BufT;

//*****************************************************************************
//************************* FUNCTION DECLARATIONS *****************************
//*****************************************************************************

void        BUFFER_InitBuffer (BUFFER_BufT* bufPtr,     \
                               uint8_t* startPtr,       \
                               uint8_t length);
uint8_t     BUFFER_GetUsedSize(BUFFER_BufT* bufPtr);
uint8_t     BUFFER_GetFreeSize(BUFFER_BufT* bufPtr);
uint8_t     BUFFER_ReadByte   (BUFFER_BufT* bufPtr,     \
                               uint8_t* errorCodePtr);
uint8_t     BUFFER_ReadByteFromTail(BUFFER_BufT* bufPtr,\
                                    uint8_t* errorCodePtr);
void        BUFFER_WriteByte  (BUFFER_BufT* bufPtr,     \
                               uint8_t byte,            \
                               uint8_t* errorCodePtr);
uint8_t     BUFFER_ReadField  (BUFFER_BufT* bufPtr,     \
                               uint8_t* dstPtr,         \
                               uint8_t byteCount,       \
                               uint8_t* errorCodePtr);
uint8_t     BUFFER_WriteField (BUFFER_BufT* bufPtr,     \
                               uint8_t* srcPtr,         \
                               uint8_t byteCount,       \
                               uint8_t* errorCodePtr);
void        BUFFER_Discard (BUFFER_BufT* bufPtr);


#endif // BUFFER_H
