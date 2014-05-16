################################################################
##
## General and make specific variables.
##
## Copyright (C) 2009-2013 Robin Klose
##
## This file is part of AVR3nk, available at
## https://github.com/r3nk/AVR3nk
##
################################################################

################################################################
## SUBMAKE specific settings
################################################################

SUBMAKE := $(MAKE) --no-print-directory
export VERBOSE
export CFLAGS_USER
export LDFLAGS_USER
export APP_MACROS

################################################################
## MAKE variables
################################################################
ifeq ($(VERBOSE),0)
AT := @
else ifeq ($(VERBOSE),1)
AT := @
else
AT :=
endif

EMPTY :=
SPACE := $(EMPTY) $(EMPTY)
NEWLINE := \n
PREFIX := \[$(SUBDIR)\]

# Behavior for avrdude specific targets:
ifndef TARGET_MCU
TARGET_MCU := $(firstword $(MCU))
endif

################################################################
## BUILD process specific variables
################################################################

SOURCES_c     := $(filter %.c, $(SOURCES))
SOURCES_C     := $(filter %.C, $(SOURCES))


ifdef BASE_SUBDIR
BUILD_TOP_DIR     := $(TOPDIR)/$(BASE_SUBDIR)/build
BUILD_DIR         := $(TOPDIR)/$(BASE_SUBDIR)/build/$(TARGET_MCU)/$(SUBDIR)
else
BASE_SUBDIR       := $(SUBDIR)
BUILD_TOP_DIR     := build
BUILD_DIR         := build/$(TARGET_MCU)/$(SUBDIR)
endif

BUILD_OBJECTS := $(SOURCES_c:%.c=$(BUILD_DIR)/%.o)
BUILD_OBJECTS += $(SOURCES_C:%.C=$(BUILD_DIR)/%.o)

BUILD_LIBRARY := $(BUILD_DIR)/$(LIBRARY).a

IDEPS_LIBRARIES = $(foreach DIR, $(DEPBUILDS_ITERATIVE), $(wildcard build/$(TARGET_MCU)/$(DIR)/*.a))

RDEPS_LIBRARY_DIR := $(BUILD_TOP_DIR)/$(TARGET_MCU)/rdeplibs
RDEPS_LIBRARY     := $(RDEPS_LIBRARY_DIR)/$(LIBRARY).a
RDEPS_LIBRARIES   := $(wildcard $(RDEPS_LIBRARY_DIR)/*.a)

BUILD_PROGRAM_ELF    := $(BUILD_DIR)/$(APPLICATION).elf
BUILD_PROGRAM_MAP    := $(BUILD_DIR)/$(APPLICATION).map
BUILD_PROGRAM_HEX    := $(BUILD_DIR)/$(APPLICATION).hex
BUILD_PROGRAM_EEPROM := $(BUILD_DIR)/$(APPLICATION).eep
BUILD_PROGRAM_LSS    := $(BUILD_DIR)/$(APPLICATION).lss


################################################################
## INSTALL process specific variables
################################################################

PACKAGE                    := $(firstword $(subst /, $(SPACE), $(SUBDIR)))

INSTALL_SUBDIR             := install/$(TARGET_MCU)
INSTALL_LIBRARY_SUBDIR     := $(INSTALL_SUBDIR)/lib
INSTALL_HEADERS_SUBDIR     := $(INSTALL_SUBDIR)/headers
INSTALL_HEADERS_PKG_SUBDIR := $(INSTALL_HEADERS_SUBDIR)/$(PACKAGE)

INSTALL_DIR                := $(TOPDIR)/$(INSTALL_SUBDIR)
INSTALL_LIBRARY_DIR        := $(TOPDIR)/$(INSTALL_LIBRARY_SUBDIR)
INSTALL_HEADERS_DIR        := $(TOPDIR)/$(INSTALL_HEADERS_SUBDIR)
INSTALL_HEADERS_PKG_DIR    := $(TOPDIR)/$(INSTALL_HEADERS_PKG_SUBDIR)

INSTALL_HEADERS := $(patsubst %.h, $(INSTALL_HEADERS_PKG_DIR)/%.h, $(notdir $(HEADERS)))
INSTALL_LIBRARY := $(INSTALL_LIBRARY_DIR)/$(LIBRARY).a

################################################################
## Device READ specific variables
################################################################

READ_DIR          := read/$(TARGET_MCU)
READ_PROGRAM_FILE := $(READ_DIR)/program.hex
READ_EEPROM_FILE  := $(READ_DIR)/eeprom.eep
