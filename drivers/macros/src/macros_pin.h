/*!
*******************************************************************************
*******************************************************************************
** \brief   Macros that operate on single pins.
**
**          This header provides the following macros:
**          - SET_OUTPUT
**          - SET_INPUT
**          - SET_HIGH
**          - SET_LOW
**          - PIN_IS
**          - IS_SET
**          - IS_UNSET
**
** \author  Markus Betz, Robin Klose
**
** Copyright (C) 2009-2013 Markus Betz, Robin Klose
**
** This file is part of AVR3nk, available at https://github.com/r3nk/AVR3nk
**
*******************************************************************************
*******************************************************************************
*/

#ifndef MACROS_PIN_H
#define MACROS_PIN_H

//! PORT macro:
#define PORT(x)             _xPORT(x)
#define _xPORT(x)           PORT ## x

//! DDR macro:
#define DDR(x)              _xDDR(x)
#define _xDDR(x)            DDR ## x

//! PIN macro:
#define PIN(x)              _xPIN(x)
#define _xPIN(x)            PIN ## x

//! Setup a pin as output:
#define SET_OUTPUT(x)       _xSET_OUTPUT(x)
#define _xSET_OUTPUT(x,y)   DDR(x) |= (1 << (y))

//! Setup a pin as input:
#define SET_INPUT(x)        _xSET_INPUT(x)
#define _xSET_INPUT(x,y)    DDR(x) &= ~(1 << (y))

//! Drive an output pin high / enable input pin pullup:
#define SET_HIGH(x)         _xSET_HIGH(x)
#define _xSET_HIGH(x,y)     PORT(x) |= (1 << (y))

//! Drive an output pin low / disable input pin pullup:
#define SET_LOW(x)          _xSET_LOW(x)
#define _xSET_LOW(x,y)      PORT(x) &= ~(1 << (y))

//! Determine pin setting
#define PIN_IS(x)           _xPIN_IS(x)
#define _xPIN_IS(x,y)       ( ( PIN(x) & ( 1 << (y) ) ) >> (y) )

//! See if the pin is high
#define IS_HIGH(x)          _xIS_HIGH(x)
#define _xIS_HIGH(x,y)      ((PIN(x) & (1 << (y))) != 0)

//! See if the pin is low
#define IS_LOW(x)           _xIS_LOW(x)
#define _xIS_LOW(x,y)       ((PIN(x) & (1 << (y))) == 0)


#endif // MACROS_PIN_H
