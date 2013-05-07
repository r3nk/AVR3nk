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

LIBRARY := libmcp2515

################################################################
## Directory Settings
################################################################

TOPDIR := ../..
SUBDIR := drivers/mcp2515

################################################################
## Sources and Headers
################################################################

SOURCES := src/mcp2515.c
HEADERS := src/mcp2515.h
HEADERS += src/mcp2515_config.h

################################################################
## Dependencies
################################################################

DEPENDENCIES := drivers/spi

################################################################
## Supported MCUs
################################################################

MCU := atmega644p
MCU += atmega644

################################################################
## Load Configuration
################################################################

include $(TOPDIR)/env/make/Make.conf
