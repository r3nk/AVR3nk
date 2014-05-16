################################################################
##
## Mandatory settings for a LIBRARY:
## - LIBRARY
## - TOPDIR
## - SUBDIR
## - SOURCES
## - HEADERS
## - DEPENDENCIES
## - MCU
##
## Available Make targets:
## all [default], clean, headers, build, install
##
## Copyright (C) 2009-2013 Robin Klose
##
## This file is part of AVR3nk, available at
## https://github.com/r3nk/AVR3nk
##
################################################################

LIBRARY := libuart

################################################################
## Directory Settings
################################################################

TOPDIR := ../..
SUBDIR := drivers/uart

################################################################
## Sources and Headers
################################################################

SOURCES := src/uart.c
HEADERS := src/uart.h

################################################################
## Dependencies
################################################################

DEPENDENCIES := drivers/buffer

################################################################
## Supported MCUs
################################################################

MCU := atmega644p
MCU += atmega644
MCU += atmega16
MCU += atmega8

################################################################
## Load Configuration
################################################################

include $(TOPDIR)/env/make/Make.conf
