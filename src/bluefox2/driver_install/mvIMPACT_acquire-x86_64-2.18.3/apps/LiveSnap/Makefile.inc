#! /bin/make
##############################################
# Makefile for the 'LiveSnap' sample
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
MODULE_NAME=LiveSnap

#-------------------------------------------
OOBJ =										\

#-------------------------------------------
vpath %.cpp ..

#-------------------------------------------
USR_INCLS =					\

#-------------------------------------------
USR_LIBS =					\

#-------------------------------------------
CPPFLAGS_1 = 				\
	-pedantic -Wno-long-long \
	-DMALLOC_TRACE

#-------------------------------------------
include $(ROOT_PATH)/Makefile.samp.inc
