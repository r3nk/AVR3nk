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
## Copyright (C) 2009-2013 Robin Klose
##
## This file is part of AVR3nk, available at 
## https://github.com/r3nk/AVR3nk
##
################################################################

APPLICATION := commandline

################################################################
## Directory Settings
################################################################

TOPDIR := ../..
SUBDIR := applications/demo_cmdl

################################################################
## Sources
################################################################

SOURCES := src/main.c

################################################################
## Module Configuration
################################################################

APP_MACROS := F_CPU=18432000
APP_MACROS += UART_ERROR_HANDLING=1
APP_MACROS += UART_BUFFER_LENGTH_RX=128
APP_MACROS += UART_BUFFER_LENGTH_TX=128
APP_MACROS += UART_CALLBACK_COUNT=3
APP_MACROS += UART_INTERRUPT_SAFETY=0
APP_MACROS += UART_ENABLE_RX_CALLBACK_NESTED_INTERRUPTS=0
APP_MACROS += CMDL_MAX_COMMAND_LENGTH=64
APP_MACROS += CMDL_MAX_COMMAND_COUNT=16
APP_MACROS += CMDL_MAX_ARGUMENT_COUNT=16
APP_MACROS += CMDL_DEBUG=0
APP_MACROS += CMDL_USAGE_STRING_SUPPORT=0

################################################################
## Pre-built Libraries
################################################################

# Add floating point support for printf:
LIBRARIES := printf_flt
LIBRARIES += m
LDFLAGS_USER += -Wl,-u,vfprintf

# Alternatively: 
#LDFLAGS_USER += -Wl,-u,vfprintf -lprintf_flt -lm

################################################################
## Dependency Builds
################################################################

# Specify all dependencies in the correct build order: 
#DEPBUILDS_ITERATIVE := drivers/buffer
#DEPBUILDS_ITERATIVE += drivers/uart
#DEPBUILDS_ITERATIVE += subsystems/cmdl

# Specify the latest dependency to build dependencies recursively: 
DEPBUILDS_RECURSIVE := subsystems/cmdl

################################################################
## Target MCU(s) (first MCU in list is used for programming)
################################################################

MCU := atmega644p
MCU += atmega644
MCU += atmega16

################################################################
## Load Configuration
################################################################

include $(TOPDIR)/env/make/Make.conf
