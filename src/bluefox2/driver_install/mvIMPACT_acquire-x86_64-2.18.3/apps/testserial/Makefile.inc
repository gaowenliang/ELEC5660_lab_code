#! /bin/make
##############################################
# Makefile for the 'testserial' sample
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
MODULE_NAME=testserial

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
	-pedantic

#-------------------------------------------
NO_MV_DEVICE_LIBS=yes

#-------------------------------------------
include $(ROOT_PATH)/Makefile.samp.inc
