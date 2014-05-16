################################################################
##
## Mandatory settings for an APPLICATION:
## - APPLICATION
## - TOPDIR
## - SUBDIR
## - SOURCES
## - HEADERS
## - LIBRARIES
## - DEPBUILDS_ITERATIVE
## - DEPBUILDS_RECURSIVE
## - MCU
##
## Available Make targets:
## all [default], clean, build, size, program
##
## Copyright (C) 2014-2014 Robin Klose
##
## This file is part of AVR3nk, available at
## https://github.com/r3nk/AVR3nk
##
################################################################

APPLICATION := timertest

################################################################
## Directory Settings
################################################################

TOPDIR := ../../..
SUBDIR := drivers/timer/test

################################################################
## Sources
################################################################

SOURCES := src/main.c

################################################################
## Module Configuration
################################################################

APP_MACROS := F_CPU=18432000
APP_MACROS += UART_ERROR_HANDLING=0
APP_MACROS += UART_BUFFER_LENGTH_RX=128
APP_MACROS += UART_BUFFER_LENGTH_TX=128
APP_MACROS += UART_CALLBACK_COUNT=3
APP_MACROS += UART_INTERRUPT_SAFETY=0
APP_MACROS += CMDL_MAX_COMMAND_LENGTH=64
APP_MACROS += CMDL_MAX_COMMAND_COUNT=16
APP_MACROS += CMDL_MAX_ARGUMENT_COUNT=16
APP_MACROS += CMDL_DEBUG=0
APP_MACROS += CMDL_USAGE_STRING_SUPPORT=0
APP_MACROS += TIMER_INTERRUPT_SAFETY=0
APP_MACROS += TIMER_WITH_COUNTDOWN=1
APP_MACROS += TIMER_COUNTDOWN_IMPRECISION=64

################################################################
## Pre-built Libraries
################################################################

#LIBRARIES := buffer
#LIBRARIES += uart

# Add floating point support for printf:
LIBRARIES := printf_flt
LIBRARIES += m
LDFLAGS_USER += -Wl,-u,vfprintf

################################################################
## Dependency Builds
################################################################

# Either specify DEPBUILDS_ITERATIVE or DEPBUILDS_RECURSIVE:

# Specify all dependencies in the correct build order:
DEPBUILDS_ITERATIVE := drivers/buffer
DEPBUILDS_ITERATIVE += drivers/uart
DEPBUILDS_ITERATIVE += drivers/timer
DEPBUILDS_ITERATIVE += subsystems/cmdl

################################################################
## Target MCU(s) (first MCU in list is used for programming)
################################################################

MCU := atmega644p

################################################################
## Load Configuration
################################################################

include $(TOPDIR)/env/make/Make.conf
