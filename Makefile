################################################################
##
## Mandatory settings for a DIRECTORY:
## - TOPDIR
## - SUBDIR
## - DIRECTORIES
##
## Available Make targets:
## all [default], clean, headers, build, install
##
## Copyright (C) 2009-2014 Robin Klose
##
## This file is part of AVR3nk, available at
## https://github.com/r3nk/AVR3nk
##
################################################################

################################################################
## Directory Settings
################################################################

TOPDIR = .
SUBDIR = .

DIRECTORIES := drivers
DIRECTORIES += subsystems
DIRECTORIES += applications

################################################################
## Override the default targets in the top directory
################################################################

OVERRIDE_DEFAULT_TARGETS = 1

default: all

clean: clean-install $(DIRECTORIES:%=subdir-clean-%)

headers: $(DIRECTORIES:%=subdir-headers-%)

build: $(DIRECTORIES:%=subdir-build-%)

install: $(DIRECTORIES:%=subdir-install-%)

all: clean-install $(DIRECTORIES:%=subdir-all-%)


################################################################
## Load Configuration
################################################################

include $(TOPDIR)/env/make/Make.conf
