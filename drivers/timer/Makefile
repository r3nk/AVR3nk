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
## Copyright (C) <year> <author>
##
## This file is part of AVR3nk, available at
## https://github.com/r3nk/AVR3nk
##
################################################################

LIBRARY := timer

################################################################
## Directory Settings
################################################################

TOPDIR := ../..
SUBDIR := drivers/timer

################################################################
## Sources and Headers
################################################################

SOURCES := src/timer.c
HEADERS := src/timer.h

################################################################
## Dependencies
################################################################

DEPENDENCIES :=

################################################################
## Supported MCUs
################################################################

MCU := atmega644p
#MCU += atmega644
#MCU += atmega16

################################################################
## Load Configuration
################################################################

include $(TOPDIR)/env/make/Make.conf
