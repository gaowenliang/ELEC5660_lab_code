#! /bin/make
##############################################
# Makefile for the 'FirmwareUpgrade' sample
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
MODULE_NAME=FirmwareUpgrade

#-------------------------------------------
OOBJ = \
	Epcs.o \
	Timeout.o

#-------------------------------------------
vpath %.cpp ..:../../Common/FirmwareUpdate_mvHYPERION

#-------------------------------------------
USR_INCLS =					\

#-------------------------------------------
USR_LIBS =					\

#-------------------------------------------
CPPFLAGS_1 = 				\
	-pedantic

#		-DMALLOC_TRACE		\

#-------------------------------------------
include $(ROOT_PATH)/Makefile.samp.inc
