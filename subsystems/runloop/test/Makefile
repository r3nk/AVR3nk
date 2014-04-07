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

APPLICATION := runloop-test

################################################################
## Directory Settings
################################################################

TOPDIR := ../../..
SUBDIR := subsystems/runloop/test

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
APP_MACROS += UART_RX_CALLBACK_COUNT=5
APP_MACROS += UART_INTERRUPT_SAFETY=0
APP_MACROS += CMDL_MAX_COMMAND_LENGTH=64
APP_MACROS += CMDL_MAX_COMMAND_COUNT=16
APP_MACROS += CMDL_MAX_ARGUMENT_COUNT=16
APP_MACROS += CMDL_DEBUG=0
APP_MACROS += CMDL_USAGE_STRING_SUPPORT=0
APP_MACROS += TIMER_INTERRUPT_SAFETY=0
APP_MACROS += TIMER_WITH_COUNTDOWN=0
APP_MACROS += TIMER_COUNTDOWN_IMPRECISION=256
APP_MACROS += RUNLOOP_MAX_NUMBER_OF_TASKS=4
APP_MACROS += RUNLOOP_WITH_UPTIME=1
APP_MACROS += RUNLOOP_UPTIME_UPDATE_INTERVAL_MS=1000
APP_MACROS += RUNLOOP_INTERRUPT_SAFETY=1
APP_MACROS += RUNLOOP_DEBUG=0

################################################################
## Pre-built Libraries
################################################################

# Add floating point support for printf:
LIBRARIES := printf_flt
LIBRARIES += m
LDFLAGS_USER += -Wl,-u,vfprintf

################################################################
## Dependency Builds
################################################################

# Specify all dependencies in the correct build order:
DEPBUILDS_ITERATIVE := drivers/buffer
DEPBUILDS_ITERATIVE += drivers/uart
DEPBUILDS_ITERATIVE += drivers/timer
DEPBUILDS_ITERATIVE += subsystems/cmdl
DEPBUILDS_ITERATIVE += subsystems/runloop

################################################################
## Target MCU(s) (first MCU in list is used for programming)
################################################################

MCU := atmega644p

################################################################
## Load Configuration
################################################################

include $(TOPDIR)/env/make/Make.conf
