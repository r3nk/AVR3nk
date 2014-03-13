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
## Copyright (C) 2014-2014 Robin Klose
##
## This file is part of AVR3nk, available at
## https://github.com/r3nk/AVR3nk
##
################################################################

LIBRARY := runloop

################################################################
## Directory Settings
################################################################

TOPDIR := ../..
SUBDIR := subsystems/runloop

################################################################
## Sources and Headers
################################################################

SOURCES := src/runloop.c
HEADERS := src/runloop.h

################################################################
## Dependencies
################################################################

DEPENDENCIES := drivers/timer
DEPENDENCIES += drivers/uart
DEPENDENCIES += subsystems/cmdl

################################################################
## Supported MCUs
################################################################

MCU := atmega644p

################################################################
## Load Configuration
################################################################

include $(TOPDIR)/env/make/Make.conf
