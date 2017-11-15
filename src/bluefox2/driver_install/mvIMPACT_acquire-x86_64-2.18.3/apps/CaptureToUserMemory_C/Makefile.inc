#! /bin/make
##############################################
# Makefile for the 'CaptureToUserMemory_C' sample,
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
MODULE_NAME=CaptureToUserMemory

#-------------------------------------------
SOURCES = exampleHelper_C.c \

#-------------------------------------------
OOBJ = exampleHelper_C.o \

#-------------------------------------------
vpath %.c ../:../../Common

#-------------------------------------------
USR_INCLS = \

#-------------------------------------------
USR_LIBS = \

#-------------------------------------------
CPPFLAGS_1 = -DMALLOC_TRACE

#-------------------------------------------
include $(ROOT_PATH)/Makefile.samp.inc

#-------------------------------------------
CTYPE=$(CC)

#-------------------------------------------

