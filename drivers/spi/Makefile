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

LIBRARY := libspi

################################################################
## Directory Settings
################################################################

TOPDIR := ../..
SUBDIR := drivers/spi

################################################################
## Sources and Headers
################################################################

SOURCES := src/spi_m.c
SOURCES += src/spi_s.c
HEADERS := src/spi_m.h
HEADERS += src/spi_s.h
HEADERS += src/spi_common.h

################################################################
## Dependencies
################################################################

DEPENDENCIES := 

################################################################
## Supported MCUs
################################################################

MCU := atmega644p
MCU += atmega644
MCU += atmega16

################################################################
## Load Configuration
################################################################

include $(TOPDIR)/env/make/Make.conf
