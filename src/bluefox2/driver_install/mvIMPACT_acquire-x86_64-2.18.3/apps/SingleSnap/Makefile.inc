#! /bin/make
##############################################
# Makefile for the 'SingleSnap' sample,
##############################################
#-------------------------------------------
ifndef MVIMPACT_ACQUIRE_SOURCE_DIR
    ifndef MVIMPACT_ACQUIRE_DIR
        ROOT_PATH=../..
    else
        ROOT_PATH=$(MVIMPACT_ACQUIRE_DIR)/apps
    endif
else
    ROOT_PATH=$(MVIMPACT_ACQUIRE_SOURCE_DIR)/apps
endif

#-------------------------------------------
MODULE_NAME=SingleSnap

#-------------------------------------------
OOBJ =										\
	PGMFileFunctions.o

#-------------------------------------------
vpath %.cpp ..:$(ROOT_PATH)/../common

#-------------------------------------------
USR_INCLS =					\

#-------------------------------------------
USR_LIBS =					\

#-------------------------------------------
CPPFLAGS_1 = 				\
	-pedantic -Wno-format

#		-DMALLOC_TRACE		\

#-------------------------------------------
include $(ROOT_PATH)/Makefile.samp.inc

#-------------------------------------------
