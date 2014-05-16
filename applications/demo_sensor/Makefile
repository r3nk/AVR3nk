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

APPLICATION := demo_sensor

################################################################
## Directory Settings
################################################################

TOPDIR := ../..
SUBDIR := applications/demo_sensor

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
APP_MACROS += SPI_M_LED_MODE=0
APP_MACROS += SPI_M_LED=B,1
APP_MACROS += SPI_M_DEBUG=0
APP_MACROS += MCP2515_CAN_2_B_SUPPORT=0
APP_MACROS += MCP2515_ERROR_CALLBACK_SUPPORT=0
APP_MACROS += MCP2515_CS=B,4
APP_MACROS += MCP2515_INT_MAIN=B,2
APP_MACROS += MCP2515_INTNO_MAIN=2
APP_MACROS += MCP2515_USE_RX_INT=0
APP_MACROS += MCP2515_INT_RXB0=D,2
APP_MACROS += MCP2515_INTNO_RXB0=0
APP_MACROS += MCP2515_INT_RXB1=D,3
APP_MACROS += MCP2515_INTNO_RXB1=1
APP_MACROS += MCP2515_DEBUG=0

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
DEPBUILDS_ITERATIVE := drivers/buffer
DEPBUILDS_ITERATIVE += drivers/uart
DEPBUILDS_ITERATIVE += drivers/spi
DEPBUILDS_ITERATIVE += drivers/mcp2515
DEPBUILDS_ITERATIVE += subsystems/cmdl

# Specify the latest dependency to build dependencies recursively:
#DEPBUILDS_RECURSIVE := subsystems/cmdl
#DEPBUILDS_RECURSIVE += drivers/mcp2515

################################################################
## Target MCU(s) (first MCU in list is used for programming)
################################################################

MCU := atmega644p

################################################################
## Load Configuration
################################################################

include $(TOPDIR)/env/make/Make.conf
