#! /bin/make
##############################################
# Makefile for the 'LiveSnapFLTK' sample
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
MODULE_NAME=LiveSnapFLTK

#-------------------------------------------
OOBJ =										\

#-------------------------------------------
vpath %.cpp ..

#-------------------------------------------
USR_INCLS =					\

#-------------------------------------------
USR_LIBS =					\
		$(FLTK_LDFLAGS)		\

#-------------------------------------------
CPPFLAGS_1 =                                \
	-pedantic -Wno-long-long                \
	$(FLTK_CPPFLAGS)                        \

#		-DMALLOC_TRACE

#-------------------------------------------
include $(ROOT_PATH)/Makefile.samp.inc
