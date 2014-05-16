/*!
*******************************************************************************
*******************************************************************************
** \brief   BUFFER is a cyclic buffer for the AVR microcontroller family.
**          It supports up to 8 bit address range for buffered
**          data (i.e. you can store at most 256 entries).
**
**          A buffer must be initialized by #BUFFER_InitBuffer before it can
**          be used. BUFFER provides functions to read/write single bytes and
**          to read/write bursts of multiple bytes.
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
#include "buffer.h"

//*****************************************************************************
//*************************** DEFINES AND MACROS ******************************
//*****************************************************************************


//*****************************************************************************
//**************************** LOCAL VARIABLES ********************************
//*****************************************************************************


//*****************************************************************************
//*************************** PUBLIC FUNCTIONS ********************************
//*****************************************************************************

/*!
*******************************************************************************
** \brief   Initializes a passed buffer structure.
**
** \attention
**          Pointer arguments are not checked for NULL pointers. Make sure that
**          valid pointers are passed.
**
** \param   bufPtr      Points to the buffer structure which will be
**                      initialized.
** \param   startPtr    Start address for the buffer data.
** \param   length      Length of the buffer.
**
*******************************************************************************
*/
void BUFFER_InitBuffer(BUFFER_BufT* bufPtr, uint8_t* startPtr, uint8_t length)
{
    bufPtr->startPtr = startPtr;
    bufPtr->length   = length;
    bufPtr->readPos  = 0;
    bufPtr->used     = 0;
    return;
}

/*!
*******************************************************************************
** \brief   Get the number of used bytes in the buffer.
**
** \attention
**          Pointer arguments are not checked for NULL pointers. Make sure that
**          valid pointers are passed.
**
** \param   bufPtr      Points to the buffer structure which will be
**                      analysed.
**
** \return  Count of used bytes in the buffer.
*******************************************************************************
*/
uint8_t BUFFER_GetUsedSize(BUFFER_BufT* bufPtr)
{
    uint8_t result;

    result = bufPtr->used;
    return(result);
}

/*!
*******************************************************************************
** \brief   Get the number of free bytes in the buffer.
**
** \attention
**          Pointer arguments are not checked for NULL pointers. Make sure that
**          valid pointers are passed.
**
** \param   bufPtr      Points to the buffer structure which will be
**                      analysed.
**
** \return  Count of free bytes in the buffer.
*******************************************************************************
*/
uint8_t BUFFER_GetFreeSize(BUFFER_BufT* bufPtr)
{
    uint8_t result;

    result = bufPtr->length - bufPtr->used;
    return(result);
}

/*!
*******************************************************************************
** \brief   Reads a single byte from the buffer and removes it from the
**          buffer (FIFO).
**
** \attention
**          Pointer arguments are not checked for NULL pointers. Make sure that
**          valid pointers are passed. Only errorCodePtr may be NULL.
**
** \param   bufPtr      Points to the buffer structure of the buffer
**                      which will be read.
** \param   errorCodePtr
**                      Optional argument that will receive error codes.
**                      Possible values are:
**                          - #BUFFER_OK on success
**                          - #BUFFER_ERR_EMPTY if the buffer was empty.
**
** \return
**          - The read value on success. (0x00 - 0xFF)
**          - 0 if an error occured. However, 0 does not necessarily indicate
**            an error.
**
*******************************************************************************
*/
uint8_t BUFFER_ReadByte(BUFFER_BufT* bufPtr, uint8_t* errorCodePtr)
{
    uint8_t val;

    if(bufPtr->used == 0)
    {
        if(errorCodePtr) *errorCodePtr = BUFFER_ERR_EMPTY;
        return(0);
    }
    val = *(bufPtr->startPtr + bufPtr->readPos);
    bufPtr->readPos = (bufPtr->readPos + 1) % bufPtr->length;
    bufPtr->used--;
    if(errorCodePtr) *errorCodePtr = BUFFER_OK;
    return(val);
}

/*!
*******************************************************************************
** \brief   Reads a single byte from the TAIL of the buffer and removes it.
**
** \attention
**          Pointer arguments are not checked for NULL pointers. Make sure that
**          valid pointers are passed. Only errorCodePtr may be NULL.
**
** \param   bufPtr      Points to the buffer structure of the buffer
**                      which will be read.
** \param   errorCodePtr
**                      Optional argument that will receive error codes.
**                      Possible values are:
**                          - #BUFFER_OK on success
**                          - #BUFFER_ERR_EMPTY if the buffer was empty.
**
** \return
**          - The read value on success. (0x00 - 0xFF)
**          - 0 if an error occured. However, 0 does not necessarily indicate
**            an error.
**
*******************************************************************************
*/
uint8_t BUFFER_ReadByteFromTail(BUFFER_BufT* bufPtr, uint8_t* errorCodePtr)
{
    uint8_t val;

    if(bufPtr->used == 0)
    {
        if(errorCodePtr) *errorCodePtr = BUFFER_ERR_EMPTY;
        return(0);
    }
    val = *(bufPtr->startPtr + ((bufPtr->readPos + (bufPtr->used - 1)) % bufPtr->length));
    bufPtr->used--;
    if(errorCodePtr) *errorCodePtr = BUFFER_OK;
    return(val);
}

/*!
*******************************************************************************
** \brief   Writes a single byte to the buffer (FIFO).
**
** \attention
**          Pointer arguments are not checked for NULL pointers. Make sure that
**          valid pointers are passed. Only errorCodePtr may be NULL.
**
** \param   bufPtr      Points to the buffer structure of the buffer
**                      which will be written.
** \param   byte        The byte that will be written to the FIFO.
** \param   errorCodePtr
**                      Optional argument that will receive error codes.
**                      Possible values are:
**                          - #BUFFER_OK on success
**                          - #BUFFER_ERR_FULL if the buffer was already full.
**
*******************************************************************************
*/
void BUFFER_WriteByte(BUFFER_BufT* bufPtr, uint8_t byte, uint8_t* errorCodePtr)
{
    if(bufPtr->used == bufPtr->length)
    {
        if(errorCodePtr) *errorCodePtr = BUFFER_ERR_FULL;
        return;
    }
    *(bufPtr->startPtr + ((bufPtr->readPos + bufPtr->used++) % bufPtr->length)) = byte;
    if(errorCodePtr) *errorCodePtr = BUFFER_OK;
    return;
}

/*!
*******************************************************************************
** \brief   Reads a couple of bytes from the buffer and removes it from the
**          buffer (FIFO).
**
** \attention
**          Pointer arguments are not checked for NULL pointers. Make sure that
**          valid pointers are passed. Only errorCodePtr may be NULL.
**
** \param   bufPtr      Points to the buffer structure of the buffer
**                      which will be read.
** \param   dstPtr      Points to the field which will be written.
**                      Make sure that at least #byteCount bytes are available
**                      in the field. If #byteCount is set to 0, make sure that
**                      at least bufPtr->length bytes are available in the
**                      field.
** \param   byteCount   Count of bytes that will be copied. Choose 0 to read
**                      the whole buffer, i.e. all used bytes in the buffer.
**                      If byteCount is greater than the count of used bytes
**                      in the buffer, only the count of used bytes will be
**                      copied.
** \param   errorCodePtr
**                      Optional argument that will receive error codes.
**                      Possible values are:
**                          - #BUFFER_OK on success
**
** \return
**          Count of actually read values.
**
*******************************************************************************
*/
uint8_t BUFFER_ReadField (BUFFER_BufT* bufPtr,
                          uint8_t* dstPtr,
                          uint8_t byteCount,
                          uint8_t* errorCodePtr)
{
    uint8_t ii;         // temporary counter
    uint8_t count;      // count of bytes actually copied

    if(byteCount == 0)
    {
        count = bufPtr->used;
    }
    else if(bufPtr->used < byteCount)
    {
        count = bufPtr->used;
    }
    else
    {
        count = byteCount;
    }
    ii = count;
    while(ii--)
    {
        *dstPtr++ = *(bufPtr->startPtr + bufPtr->readPos++);
        bufPtr->readPos %= bufPtr->length;
    }
    bufPtr->used -= count;
    if(errorCodePtr) *errorCodePtr = BUFFER_OK;
    return (count);
}

/*!
*******************************************************************************
** \brief   Writes a couple of bytes to the buffer.
**
** \attention
**          Pointer arguments are not checked for NULL pointers. Make sure that
**          valid pointers are passed. Only errorCodePtr may be NULL.
**
** \param   bufPtr      Points to the buffer structure of the buffer
**                      which will be written.
** \param   srcPtr      Points to the field which will be read.
** \param   byteCount   Count of bytes that will be copied. Choose 0 to fill
**                      the whole buffer.
**                      If byteCount is greater than the count of free bytes
**                      in the buffer, only the count of free bytes will be
**                      copied.
** \param   errorCodePtr
**                      Optional argument that will receive error codes.
**                      Possible values are:
**                          - #BUFFER_OK on success
**
** \return
**          Count of actually written values.
**
*******************************************************************************
*/
uint8_t BUFFER_WriteField (BUFFER_BufT* bufPtr,
                           uint8_t* srcPtr,
                           uint8_t byteCount,
                           uint8_t* errorCodePtr)
{
    uint8_t ii;         // temporary counter
    uint8_t count = 0;  // count of bytes actually copied

    if(byteCount == 0)
    {
        ii = bufPtr->length - bufPtr->used;
    }
    else if (byteCount > (bufPtr->length - bufPtr->used))
    {
        ii = bufPtr->length - bufPtr->used;
    }
    else
    {
        ii = byteCount;
    }
    count = ii;
    while(ii--)
    {
        *(bufPtr->startPtr + ((bufPtr->readPos + bufPtr->used++) % bufPtr->length)) = *srcPtr++;
    }
    if(errorCodePtr) *errorCodePtr = BUFFER_OK;
    return(count);
}

/*!
*******************************************************************************
** \brief   Discard all values in the buffer.
**
** \attention
**          Pointer arguments are not checked for NULL pointers. Make sure that
**          valid pointers are passed.
**
*******************************************************************************
*/
void BUFFER_Discard(BUFFER_BufT* bufPtr)
{
    bufPtr->used = 0;
    return;
}
